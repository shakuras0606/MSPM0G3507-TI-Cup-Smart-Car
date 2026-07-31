/**
 * @file    line_sensor16.c
 * @brief   Hiwonder 8 路红外巡线模块 UART2 + DMA 底层
 *
 * 手册协议：
 *   0：上电后配置为手动读取模式；
 *   2：读取模拟值，返回 55 AA 02 10 + 16B 数据 + checksum。
 *
 * 本驱动不读取模块自动计算的数字状态，也不依赖模块灰度学习结果。
 * DMA_CH2 一次搬运完整 21 字节模拟帧，MSPM0 再按 project_config.h
 * 的阈值生成黑线掩码和启停线统计。位置算法可在
 * project_config.h 中选择原二值等权平均或原始ADC直接加权平均。
 */

#include "line_sensor16.h"

#include <stddef.h>
#include <string.h>

#include "bsp_time.h"
#include "byte_ring.h"
#include "project_config.h"
#include "ti_msp_dl_config.h"

#define LINE_UART_MODE_MANUAL           (0u)
#define LINE_UART_COMMAND_ANALOG        (2u)
#define LINE_UART_FRAME_HEADER_1        (0x55u)
#define LINE_UART_FRAME_HEADER_2        (0xAAu)
#define LINE_UART_ANALOG_PAYLOAD_SIZE   (16u)
#define LINE_UART_ANALOG_FRAME_SIZE     (21u)
#define LINE_UART_RX_RING_SIZE          (64u)
#define LINE_SENSOR_VALID_MASK          (0x00FFu)

typedef enum
{
    LINE_REQUEST_NONE = 0,
    LINE_REQUEST_ANALOG
} LineRequest;

static LineSensor16Data g_data;

/** S1 最左、S8 最右；保持与旧位置环相近的 ±1400 满量程。 */
static const int16_t k_channel_position_weights[
    LINE_SENSOR_PHYSICAL_CHANNEL_COUNT] = {
    CONFIG_LINE_SENSOR_WEIGHT_S1,
    CONFIG_LINE_SENSOR_WEIGHT_S2,
    CONFIG_LINE_SENSOR_WEIGHT_S3,
    CONFIG_LINE_SENSOR_WEIGHT_S4,
    CONFIG_LINE_SENSOR_WEIGHT_S5,
    CONFIG_LINE_SENSOR_WEIGHT_S6,
    CONFIG_LINE_SENSOR_WEIGHT_S7,
    CONFIG_LINE_SENSOR_WEIGHT_S8
};

static volatile uint8_t g_dma_rx_buffer[LINE_UART_ANALOG_FRAME_SIZE];
static volatile uint8_t g_dma_rx_length;
static volatile uint8_t g_dma_tx_byte;
static volatile bool g_tx_busy;

static uint8_t g_rx_storage[LINE_UART_RX_RING_SIZE];
static ByteRing g_rx_ring;

static LineRequest g_pending_request;
static uint32_t g_request_start_ms;
static uint32_t g_last_mode_command_ms;

static uint8_t g_frame[LINE_UART_ANALOG_FRAME_SIZE];
static uint8_t g_frame_index;

static uint16_t g_analog_values[LINE_SENSOR_PHYSICAL_CHANNEL_COUNT];
static bool g_analog_valid;

static int16_t channel_position(uint8_t channel)
{
    if (channel >= LINE_SENSOR_PHYSICAL_CHANNEL_COUNT) {
        return 0;
    }
    return k_channel_position_weights[channel];
}

/** 根据统一阈值生成“1=黑线”的低8位掩码。 */
static uint16_t build_black_mask(const uint16_t *values)
{
    uint16_t black_mask = 0u;
    uint8_t index;

    for (index = 0u; index < LINE_SENSOR_PHYSICAL_CHANNEL_COUNT; ++index) {
        bool is_black;

        if (CONFIG_LINE_SENSOR_ANALOG_BLACK_HIGH != 0u) {
            is_black = values[index] > CONFIG_LINE_SENSOR_ADC_THRESHOLD;
        } else {
            is_black = values[index] < CONFIG_LINE_SENSOR_ADC_THRESHOLD;
        }
        if (is_black) {
            black_mask |= (uint16_t)1u << index;
        }
    }
    return black_mask;
}

static void start_rx_dma(uint8_t length)
{
    DL_DMA_disableChannel(DMA, DMA_CH2_CHAN_ID);
    g_dma_rx_length = length;
    DL_DMA_setSrcAddr(DMA, DMA_CH2_CHAN_ID,
                      (uint32_t)&LINE_UART_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH2_CHAN_ID,
                       (uint32_t)&g_dma_rx_buffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH2_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, DMA_CH2_CHAN_ID);
}

static bool send_byte_dma(uint8_t value)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (g_tx_busy) {
        if (primask == 0u) {
            __enable_irq();
        }
        return false;
    }
    g_tx_busy = true;
    g_dma_tx_byte = value;
    if (primask == 0u) {
        __enable_irq();
    }

    DL_DMA_setSrcAddr(DMA, DMA_CH3_CHAN_ID,
                      (uint32_t)&g_dma_tx_byte);
    DL_DMA_setDestAddr(DMA, DMA_CH3_CHAN_ID,
                       (uint32_t)&LINE_UART_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH3_CHAN_ID, 1u);
    DL_DMA_enableChannel(DMA, DMA_CH3_CHAN_ID);
    return true;
}

#if (CONFIG_LINE_SENSOR_POSITION_MODE == \
     CONFIG_LINE_SENSOR_POSITION_DIGITAL)
static void restore_analog_values(void)
{
    uint8_t index;

    if (!g_analog_valid) {
        return;
    }
    for (index = 0u; index < LINE_SENSOR_PHYSICAL_CHANNEL_COUNT; ++index) {
        g_data.values[index] = g_analog_values[index];
    }
    for (; index < LINE_SENSOR16_CHANNEL_COUNT; ++index) {
        g_data.values[index] = 0u;
    }
}
#endif

static uint8_t frame_checksum(const uint8_t *frame, uint8_t payload_size)
{
    uint8_t sum = 0u;
    uint8_t index;

    for (index = 2u; index < (uint8_t)(4u + payload_size); ++index) {
        sum = (uint8_t)(sum + frame[index]);
    }
    return (uint8_t)~sum;
}

static void apply_analog_threshold(uint32_t now_ms)
{
#if (CONFIG_LINE_SENSOR_POSITION_MODE == \
     CONFIG_LINE_SENSOR_POSITION_ANALOG_WEIGHTED)
    uint16_t values[LINE_SENSOR16_CHANNEL_COUNT] = {0u};
    uint8_t index;

    for (index = 0u; index < LINE_SENSOR_PHYSICAL_CHANNEL_COUNT; ++index) {
        values[index] = g_analog_values[index];
    }
    LineSensor16_UpdateAnalog(values);
#else
    uint16_t black_mask = build_black_mask(g_analog_values);

    /*
     * black_mask 已经由 MSPM0 归一化为“1=黑线”，因此不再使用模块
     * 自动数字量的极性。UpdateDigital 只负责加权位置和活跃通道统计。
     */
    LineSensor16_UpdateDigital(black_mask, false);
    restore_analog_values();
#endif
    g_data.online = true;
    g_data.last_update_ms = now_ms;
    ++g_data.state_frames;
    g_data.source = LINE_SENSOR16_SOURCE_UART;
}

static void complete_analog_frame(uint32_t now_ms)
{
    uint8_t index;

    if ((g_frame[2] != LINE_UART_COMMAND_ANALOG) ||
        (g_frame[3] != LINE_UART_ANALOG_PAYLOAD_SIZE) ||
        (frame_checksum(g_frame, g_frame[3]) !=
         g_frame[LINE_UART_ANALOG_FRAME_SIZE - 1u])) {
        ++g_data.protocol_errors;
        return;
    }

    for (index = 0u; index < LINE_SENSOR_PHYSICAL_CHANNEL_COUNT; ++index) {
        uint8_t logical_index =
            (CONFIG_LINE_SENSOR_REVERSE_ORDER != 0u) ?
            (uint8_t)(LINE_SENSOR_PHYSICAL_CHANNEL_COUNT - 1u - index) :
            index;
        uint8_t offset = (uint8_t)(4u + index * 2u);

        g_analog_values[logical_index] =
            (uint16_t)g_frame[offset] |
            ((uint16_t)g_frame[(uint8_t)(offset + 1u)] << 8);
    }
    g_analog_valid = true;
    ++g_data.analog_frames;
    apply_analog_threshold(now_ms);
}

static void parse_analog_byte(uint8_t value, uint32_t now_ms)
{
    if (g_frame_index == 0u) {
        if (value == LINE_UART_FRAME_HEADER_1) {
            g_frame[0] = value;
            g_frame_index = 1u;
        }
        return;
    }

    if (g_frame_index == 1u) {
        if (value == LINE_UART_FRAME_HEADER_2) {
            g_frame[1] = value;
            g_frame_index = 2u;
        } else if (value != LINE_UART_FRAME_HEADER_1) {
            g_frame_index = 0u;
        }
        return;
    }

    g_frame[g_frame_index++] = value;
    if ((g_frame_index == 4u) &&
        (g_frame[3] > LINE_UART_ANALOG_PAYLOAD_SIZE)) {
        g_frame_index = 0u;
        g_pending_request = LINE_REQUEST_NONE;
        ++g_data.protocol_errors;
        return;
    }

    if ((g_frame_index >= 4u) &&
        (g_frame_index == (uint8_t)(5u + g_frame[3]))) {
        complete_analog_frame(now_ms);
        g_frame_index = 0u;
        g_pending_request = LINE_REQUEST_NONE;
    }
}

static void process_rx_bytes(uint32_t now_ms)
{
    uint8_t value;

    while (ByteRing_Pop(&g_rx_ring, &value)) {
        if (g_pending_request == LINE_REQUEST_ANALOG) {
            parse_analog_byte(value, now_ms);
        } else {
            /* 没有模拟量请求时出现的字节不参与巡线，避免误用模块数字量。 */
            ++g_data.protocol_errors;
        }
    }
    g_data.rx_overflows = ByteRing_OverflowCount(&g_rx_ring);
}

static bool start_analog_request(uint32_t now_ms)
{
    if (g_tx_busy) {
        return false;
    }
    start_rx_dma(LINE_UART_ANALOG_FRAME_SIZE);
    if (!send_byte_dma(LINE_UART_COMMAND_ANALOG)) {
        start_rx_dma(1u);
        return false;
    }
    g_pending_request = LINE_REQUEST_ANALOG;
    g_request_start_ms = now_ms;
    g_frame_index = 0u;
    return true;
}

void LineSensor16_Init(void)
{
    uint32_t now_ms = BSP_Time_Millis();

    memset(&g_data, 0, sizeof(g_data));
    memset(g_analog_values, 0, sizeof(g_analog_values));
    ByteRing_Init(&g_rx_ring, g_rx_storage, (uint16_t)sizeof(g_rx_storage));
    g_data.line_lost = true;
    g_data.source = LINE_SENSOR16_SOURCE_NONE;
    g_analog_valid = false;
    g_pending_request = LINE_REQUEST_NONE;
    g_frame_index = 0u;
    g_tx_busy = false;

    NVIC_DisableIRQ(LINE_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(LINE_UART_INST_INT_IRQN);
    start_rx_dma(1u);
    NVIC_EnableIRQ(LINE_UART_INST_INT_IRQN);

    g_last_mode_command_ms = now_ms;
    (void)send_byte_dma(LINE_UART_MODE_MANUAL);
}

void LineSensor16_Scan(void)
{
    uint32_t now_ms = BSP_Time_Millis();

    process_rx_bytes(now_ms);

    if ((g_pending_request != LINE_REQUEST_NONE) &&
        ((uint32_t)(now_ms - g_request_start_ms) >=
         CONFIG_LINE_SENSOR_UART_RESPONSE_TIMEOUT_MS)) {
        g_pending_request = LINE_REQUEST_NONE;
        g_frame_index = 0u;
        ++g_data.protocol_errors;
    }

    if (!g_data.online ||
        ((uint32_t)(now_ms - g_data.last_update_ms) >
         CONFIG_LINE_SENSOR_UART_STALE_TIMEOUT_MS)) {
        g_data.online = false;
        g_data.line_lost = true;
        g_data.active_count = 0u;
        g_data.active_mask = 0u;

        /*
         * 模块比主控晚供电或运行中复位时，周期重发手动模式配置。
         * 模块已在线后不重复配置，符合手册“模式仅复位后重配”的要求。
         */
        if ((g_pending_request == LINE_REQUEST_NONE) &&
            ((uint32_t)(now_ms - g_last_mode_command_ms) >=
             CONFIG_LINE_SENSOR_UART_MODE_RETRY_MS) &&
            send_byte_dma(LINE_UART_MODE_MANUAL)) {
            g_last_mode_command_ms = now_ms;
            return;
        }
    }

    if (g_pending_request != LINE_REQUEST_NONE) {
        return;
    }

    (void)start_analog_request(now_ms);
}

void LineSensor16_UpdateDigital(uint16_t raw_mask, bool active_low)
{
    uint16_t masked_raw = raw_mask & LINE_SENSOR_VALID_MASK;
    uint16_t active_mask = active_low ?
        ((uint16_t)~masked_raw & LINE_SENSOR_VALID_MASK) : masked_raw;
    int32_t weighted_sum = 0;
    uint8_t index;

    memset(g_data.values, 0, sizeof(g_data.values));
    g_data.raw_mask = masked_raw;
    g_data.active_mask = active_mask;
    g_data.active_count = 0u;

    for (index = 0u; index < LINE_SENSOR_PHYSICAL_CHANNEL_COUNT; ++index) {
        if ((active_mask & ((uint16_t)1u << index)) != 0u) {
            g_data.values[index] = 1u;
            weighted_sum += channel_position(index);
            ++g_data.active_count;
        }
    }

    g_data.total_strength = g_data.active_count;
    g_data.peak_strength = (g_data.active_count == 0u) ? 0u : 1u;
    g_data.line_lost = (g_data.active_count == 0u);
    if (!g_data.line_lost) {
        g_data.position =
            (int16_t)(weighted_sum / (int32_t)g_data.active_count);
    }
    g_data.source = LINE_SENSOR16_SOURCE_DIGITAL;
}

void LineSensor16_UpdateAnalog(
    const uint16_t values[LINE_SENSOR16_CHANNEL_COUNT])
{
    uint16_t samples[LINE_SENSOR_PHYSICAL_CHANNEL_COUNT];
    uint16_t black_mask;
    uint32_t total = 0u;
    int64_t weighted_sum = 0;
    uint16_t peak = 0u;
    uint8_t index;

    if (values == NULL) {
        return;
    }

    /*
     * 先复制再清g_data.values，允许测试代码把g_data.values本身传回来，
     * 也避免调用者数组与内部数组重叠时样本被提前清零。
     */
    for (index = 0u; index < LINE_SENSOR_PHYSICAL_CHANNEL_COUNT; ++index) {
        samples[index] = values[index];
    }
    black_mask = build_black_mask(samples);

    memset(g_data.values, 0, sizeof(g_data.values));
    g_data.raw_mask = black_mask;
    g_data.active_mask = black_mask;
    g_data.active_count = 0u;
    for (index = 0u; index < LINE_SENSOR_PHYSICAL_CHANNEL_COUNT; ++index) {
        uint16_t value = samples[index];

        g_data.values[index] = samples[index];
        total += value;
        weighted_sum +=
            (int64_t)channel_position(index) * (int64_t)value;
        if (value > peak) {
            peak = value;
        }
        if ((black_mask & ((uint16_t)1u << index)) != 0u) {
            ++g_data.active_count;
        }
    }

    g_data.total_strength =
        (total > UINT16_MAX) ? UINT16_MAX : (uint16_t)total;
    g_data.peak_strength = peak;
    /* 模拟位置完全不依赖ADC阈值；只有八路全为0才无法求质心。 */
    g_data.line_lost = (total == 0u);
    if (!g_data.line_lost) {
        g_data.position = (int16_t)(weighted_sum / (int64_t)total);
    }
    g_data.source = LINE_SENSOR16_SOURCE_ANALOG;
}

const LineSensor16Data *LineSensor16_GetData(void)
{
    return &g_data;
}

void LINE_UART_INST_IRQHandler(void)
{
    uint8_t index;

    switch (DL_UART_Main_getPendingInterrupt(LINE_UART_INST)) {
    case DL_UART_MAIN_IIDX_DMA_DONE_RX:
        for (index = 0u; index < g_dma_rx_length; ++index) {
            (void)ByteRing_PushFromIsr(
                &g_rx_ring, g_dma_rx_buffer[index]);
        }
        /*
         * 默认立即恢复1字节接收，避免两次主循环调度之间UART无DMA接收。
         * 发起模拟量命令前，start_analog_request()会切换为完整21字节DMA。
         */
        start_rx_dma(1u);
        break;

    case DL_UART_MAIN_IIDX_DMA_DONE_TX:
        g_tx_busy = false;
        break;

    default:
        break;
    }
}
