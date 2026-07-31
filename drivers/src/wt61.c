/**
 * @file    wt61.c
 * @brief   WT61TTL UART1 RX DMA 驱动及 WIT 标准 11 字节协议解析
 *
 * 数据路径：
 *   UART1 RX FIFO -> DMA_CH1(11B) -> DMA 暂存数组
 *   -> UART1 DMA_DONE_RX ISR -> 256B 环形缓冲区
 *   -> 主循环 WT61_Process() -> 角度/角速度快照
 *
 * ISR 只复制固定 11 字节并立即重启 DMA，不进行浮点换算。
 */

#include "wt61.h"

#include <stddef.h>
#include <string.h>

#include "bsp_time.h"
#include "byte_ring.h"
#include "project_config.h"
#include "ti_msp_dl_config.h"

#define WT61_HEADER             (0x55u)
#define WT61_TYPE_ACCEL         (0x51u)
#define WT61_TYPE_GYRO          (0x52u)
#define WT61_TYPE_ANGLE         (0x53u)
#define WT61_RAW_SCALE          (32768.0f)
#define WT61_ACCEL_RANGE_G      (16.0f)
#define WT61_ANGLE_RANGE_DEG    (180.0f)
#define WT61_GYRO_RANGE_DPS     (2000.0f)
#define WT61_TEMPERATURE_SCALE  (100.0f)

/** DMA 每次直接写入的固定 11 字节暂存区。 */
static volatile uint8_t g_dma_frame[CONFIG_WT61_FRAME_SIZE];

/** ISR 与主循环之间的无锁字节环形缓冲区。 */
static uint8_t g_rx_storage[CONFIG_WT61_RX_RING_SIZE];
static ByteRing g_rx_ring;

/** 流式协议解析器状态。 */
static uint8_t g_parser_frame[CONFIG_WT61_FRAME_SIZE];
static uint8_t g_parser_index;

/** 仅主循环修改、通过快照接口读取的最新传感器状态。 */
static WT61Snapshot g_state;

/** 将两个小端字节转换为有符号 16 位原始量。 */
static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/** 重新装载并启动下一次 UART1 RX DMA。 */
static void start_rx_dma(void)
{
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID,
                      (uint32_t)&WT61_UART_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID,
                       (uint32_t)&g_dma_frame[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID,
                           (uint16_t)CONFIG_WT61_FRAME_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
}

/** 校验一帧并更新对应物理量。 */
static void parse_frame(const uint8_t *frame)
{
    uint8_t checksum = 0u;
    uint8_t i;
    uint32_t now;

    for (i = 0u; i < (CONFIG_WT61_FRAME_SIZE - 1u); ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    if (checksum != frame[CONFIG_WT61_FRAME_SIZE - 1u]) {
        ++g_state.checksum_errors;
        return;
    }

    now = BSP_Time_Millis();
    if (frame[1] == WT61_TYPE_ACCEL) {
        /*
         * WIT 0x51 帧：Ax、Ay、Az、温度，均为有符号小端16位。
         * 官方协议固定按 ±16 g 满量程换算。这里保留传感器坐标系和
         * 重力分量，不擅自做安装方向映射或重力补偿。
         */
        g_state.accel_x_g =
            (float)read_i16_le(&frame[2]) * WT61_ACCEL_RANGE_G /
            WT61_RAW_SCALE;
        g_state.accel_y_g =
            (float)read_i16_le(&frame[4]) * WT61_ACCEL_RANGE_G /
            WT61_RAW_SCALE;
        g_state.accel_z_g =
            (float)read_i16_le(&frame[6]) * WT61_ACCEL_RANGE_G /
            WT61_RAW_SCALE;
        g_state.temperature_c =
            (float)read_i16_le(&frame[8]) / WT61_TEMPERATURE_SCALE;
        g_state.last_accel_ms = now;
        g_state.accel_valid = true;
        ++g_state.accel_frames;
    } else if (frame[1] == WT61_TYPE_ANGLE) {
        /*
         * 0x53 帧顺序为 Roll、Pitch、Yaw，原始有符号数按 ±180° 满量程换算。
         * Pitch 的有效物理范围通常为 ±90°，但协议缩放仍使用 180/32768。
         */
        g_state.roll_deg =
            (float)read_i16_le(&frame[2]) * WT61_ANGLE_RANGE_DEG /
            WT61_RAW_SCALE;
        g_state.pitch_deg =
            (float)read_i16_le(&frame[4]) * WT61_ANGLE_RANGE_DEG /
            WT61_RAW_SCALE;
        g_state.yaw_deg =
            (float)read_i16_le(&frame[6]) * WT61_ANGLE_RANGE_DEG /
            WT61_RAW_SCALE;
        g_state.last_angle_ms = now;
        g_state.angle_valid = true;
        ++g_state.angle_frames;
    } else if (frame[1] == WT61_TYPE_GYRO) {
        g_state.gyro_x_dps =
            (float)read_i16_le(&frame[2]) * WT61_GYRO_RANGE_DPS /
            WT61_RAW_SCALE;
        g_state.gyro_y_dps =
            (float)read_i16_le(&frame[4]) * WT61_GYRO_RANGE_DPS /
            WT61_RAW_SCALE;
        g_state.gyro_z_dps =
            (float)read_i16_le(&frame[6]) * WT61_GYRO_RANGE_DPS /
            WT61_RAW_SCALE;
        g_state.last_gyro_ms = now;
        g_state.gyro_valid = true;
        ++g_state.gyro_frames;
    }
}

void WT61_Init(void)
{
    ByteRing_Init(&g_rx_ring, g_rx_storage, (uint16_t)sizeof(g_rx_storage));
    memset(&g_state, 0, sizeof(g_state));
    g_parser_index = 0u;

    NVIC_DisableIRQ(WT61_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(WT61_UART_INST_INT_IRQN);
    start_rx_dma();
    NVIC_EnableIRQ(WT61_UART_INST_INT_IRQN);
}

void WT61_Process(void)
{
    uint8_t value;

    while (ByteRing_Pop(&g_rx_ring, &value)) {
        if (g_parser_index == 0u) {
            if (value == WT61_HEADER) {
                g_parser_frame[0] = value;
                g_parser_index = 1u;
            }
            continue;
        }

        g_parser_frame[g_parser_index++] = value;
        if (g_parser_index == CONFIG_WT61_FRAME_SIZE) {
            parse_frame(g_parser_frame);
            g_parser_index = 0u;
        }
    }
    g_state.rx_overflows = ByteRing_OverflowCount(&g_rx_ring);
}

bool WT61_GetSnapshot(WT61Snapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = g_state;
    if (primask == 0u) {
        __enable_irq();
    }
    return true;
}

bool WT61_IsAngleFresh(uint32_t timeout_ms)
{
    WT61Snapshot snapshot;

    (void)WT61_GetSnapshot(&snapshot);
    return snapshot.angle_valid &&
           ((uint32_t)(BSP_Time_Millis() - snapshot.last_angle_ms) <=
            timeout_ms);
}

bool WT61_IsAccelerationFresh(uint32_t timeout_ms)
{
    WT61Snapshot snapshot;

    (void)WT61_GetSnapshot(&snapshot);
    return snapshot.accel_valid &&
           ((uint32_t)(BSP_Time_Millis() - snapshot.last_accel_ms) <=
            timeout_ms);
}

/**
 * @brief UART1 DMA 接收完成中断。
 *
 * 固定复制 11 字节约几十条指令；9600 baud 下相邻字节约 1 ms，ISR 有
 * 足够时间在 FIFO 溢出前重新装载 DMA。协议解析和浮点计算均留在主循环。
 */
void WT61_UART_INST_IRQHandler(void)
{
    uint8_t i;

    switch (DL_UART_Main_getPendingInterrupt(WT61_UART_INST)) {
    case DL_UART_MAIN_IIDX_DMA_DONE_RX:
        for (i = 0u; i < CONFIG_WT61_FRAME_SIZE; ++i) {
            (void)ByteRing_PushFromIsr(&g_rx_ring, g_dma_frame[i]);
        }
        start_rx_dma();
        break;
    default:
        break;
    }
}
