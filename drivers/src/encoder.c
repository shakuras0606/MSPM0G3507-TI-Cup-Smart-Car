/**
 * @file    encoder.c
 * @brief   双路 N20 正交编码器驱动实现（GPIO 双边沿中断，x4 解码）
 *
 * 设计要点：
 *   1. PB0-PB3 的方向、边沿和中断由 empty.syscfg 统一配置；
 *   2. GROUP1_IRQHandler 只读取 GPIO、查表并累计计数；
 *   3. SysTick ISR 按CONFIG_TASK_ENCODER_CAPTURE_HZ保存累计计数快照；
 *   4. RPM 除法和诊断输出全部留在主循环；
 *   5. RPM 按窗口真实时间和电机到车轮减速比计算。
 */

#include "encoder.h"

#include <stddef.h>

#include "bsp_time.h"
#include "project_config.h"
#include "ti_msp_dl_config.h"

/** 两路编码器四个引脚的总掩码。 */
#define ENCODER_ALL_PINS \
    (ENCODER_IO_ENC_M1_A_PIN | ENCODER_IO_ENC_M1_B_PIN | \
     ENCODER_IO_ENC_M2_A_PIN | ENCODER_IO_ENC_M2_B_PIN)

/** M1 两相引脚掩码，用于判断本次中断是否与 M1 有关。 */
#define ENCODER_M1_PINS \
    (ENCODER_IO_ENC_M1_A_PIN | ENCODER_IO_ENC_M1_B_PIN)

/** M2 两相引脚掩码，用于判断本次中断是否与 M2 有关。 */
#define ENCODER_M2_PINS \
    (ENCODER_IO_ENC_M2_A_PIN | ENCODER_IO_ENC_M2_B_PIN)

/**
 * 滑动窗口需要保存“窗口时长 × 快照频率 + 1”个端点。
 */
#define ENCODER_SPEED_HISTORY_LENGTH \
    (((CONFIG_ENCODER_MEASUREMENT_WINDOW_MS * \
       CONFIG_TASK_ENCODER_CAPTURE_HZ) / CONFIG_SCHEDULER_TICK_HZ) + 1u)

/** 滑动测速窗口中的一个累计计数快照。 */
typedef struct
{
    uint32_t count;             /**< 采样时刻的模 2^32 累计计数。 */
    uint32_t timestamp_ms;      /**< 采样时刻的 1 ms 系统时间。 */
} EncoderSpeedSample;

/**
 * @brief 驱动内部状态。
 *
 * pulse_count、invalid_transitions 和测速历史在中断中写，因此声明为
 * volatile；RPM 换算结果由主循环写。
 */
typedef struct
{
    volatile uint32_t pulse_count;
    volatile uint32_t invalid_transitions;
    int32_t last_delta_count;
    int32_t rpm;
    uint32_t last_sample_time_ms;
    volatile EncoderSpeedSample speed_history[ENCODER_SPEED_HISTORY_LENGTH];
    volatile uint32_t captured_sequence;
    uint32_t processed_sequence;
    volatile uint8_t history_head;
    volatile uint8_t history_count;
} EncoderData;

static EncoderData g_encoders[ENCODER_COUNT];

/** 编码器初始化完成前，SysTick 不写测速历史。 */
static volatile bool g_encoder_initialized;

/** 从 Encoder_Init() 时刻起累计的 1 ms tick。 */
static volatile uint32_t g_last_capture_ms;

/** 每路最近一次有效的 AB 二位状态，bit0=A、bit1=B。 */
static volatile uint8_t g_last_ab[ENCODER_COUNT];

/**
 * x4 正交解码查表。
 *
 * 索引：(last_ab << 2) | current_ab。
 * 合法的单步相位变化返回 +1 或 -1；未变化和非法跨两步变化返回 0。
 */
static const int8_t k_quadrature_table[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

/** @brief 判断编码器索引是否可以安全访问数组。 */
static bool is_valid_encoder(EncoderIndex encoder)
{
    return (encoder >= ENCODER_M1) && (encoder < ENCODER_COUNT);
}

/**
 * @brief 从一次 GPIOB 输入快照提取指定编码器的 AB 状态。
 */
static uint8_t read_ab_from_snapshot(EncoderIndex encoder, uint32_t gpio_b)
{
    uint32_t a_pin;
    uint32_t b_pin;
    uint8_t ab = 0u;

    if (encoder == ENCODER_M1) {
        a_pin = ENCODER_IO_ENC_M1_A_PIN;
        b_pin = ENCODER_IO_ENC_M1_B_PIN;
    } else {
        a_pin = ENCODER_IO_ENC_M2_A_PIN;
        b_pin = ENCODER_IO_ENC_M2_B_PIN;
    }

    if ((gpio_b & a_pin) != 0u) {
        ab |= 1u;
    }
    if ((gpio_b & b_pin) != 0u) {
        ab |= 2u;
    }
    return ab;
}

/**
 * @brief 返回指定编码器的软件方向系数。
 *
 * 方向反了只需要修改 project_config.h，不改变接线和状态表。
 */
static int8_t direction_sign(EncoderIndex encoder)
{
    if (encoder == ENCODER_M1) {
        return (CONFIG_ENCODER_M1_INVERT_DIRECTION != 0u) ? -1 : +1;
    }
    return (CONFIG_ENCODER_M2_INVERT_DIRECTION != 0u) ? -1 : +1;
}

/** @brief 返回电机转子转数/车轮转数这一减速比的分子。 */
static uint32_t motor_to_wheel_ratio_num(EncoderIndex encoder)
{
    return (encoder == ENCODER_M1) ?
           (uint32_t)CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_NUM :
           (uint32_t)CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_NUM;
}

/** @brief 返回电机转子转数/车轮转数这一减速比的分母。 */
static uint32_t motor_to_wheel_ratio_den(EncoderIndex encoder)
{
    return (encoder == ENCODER_M1) ?
           (uint32_t)CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_DEN :
           (uint32_t)CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_DEN;
}

/**
 * @brief 在 ISR 中解码一路编码器的新状态。
 *
 * 如果 AB 两位同时变化，说明 ISR 响应期间至少跨过了一个中间状态，
 * 或信号存在干扰；该事件不会计数，但会累加 invalid_transitions。
 */
static void decode_transition(EncoderIndex encoder, uint32_t gpio_b)
{
    uint8_t previous = g_last_ab[encoder];
    uint8_t current = read_ab_from_snapshot(encoder, gpio_b);
    uint8_t table_index = (uint8_t)((previous << 2) | current);
    int8_t step = k_quadrature_table[table_index];
    int8_t directed_step;

    g_last_ab[encoder] = current;

    if ((step == 0) && (current != previous)) {
        ++g_encoders[encoder].invalid_transitions;
        return;
    }

    /*
     * 累计计数内部使用无符号数，使运行数天后跨过 32 位边界仍是定义明确的
     * 模运算。对外快照再解释为有符号计数，正反转使用保持不变。
     */
    directed_step = (int8_t)(step * direction_sign(encoder));
    if (directed_step > 0) {
        ++g_encoders[encoder].pulse_count;
    } else if (directed_step < 0) {
        --g_encoders[encoder].pulse_count;
    }
}

/**
 * @brief 进入临界区并返回进入前的 PRIMASK。
 *
 * 使用 PRIMASK 而不是直接 Disable/Enable，可保持调用者原本的中断状态。
 */
static uint32_t enter_critical(void)
{
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return previous_primask;
}

/** @brief 恢复进入临界区前的中断状态。 */
static void leave_critical(uint32_t previous_primask)
{
    __DMB();
    if (previous_primask == 0u) {
        __enable_irq();
    }
}

/**
 * @brief 以当前计数和时间重新建立一路测速历史。
 *
 * 滑动窗口在收集满CONFIG_ENCODER_MEASUREMENT_WINDOW_MS数据前保持RPM为0，
 * 避免启动阶段使用单次短窗口产生很大的量化跳变。
 */
static void reset_speed_history(EncoderData *encoder, uint32_t now)
{
    encoder->last_delta_count = 0;
    encoder->rpm = 0;
    encoder->last_sample_time_ms = 0u;
    encoder->history_head = 0u;
    encoder->history_count = 1u;
    encoder->captured_sequence = 0u;
    encoder->processed_sequence = 0u;
    encoder->speed_history[0].count = encoder->pulse_count;
    encoder->speed_history[0].timestamp_ms = now;
}

void Encoder_Init(void)
{
    uint32_t gpio_b;
    uint32_t now;
    uint8_t index;

    /*
     * SysConfig 已配置 GPIO 和边沿。这里先关闭 NVIC，再用真实引脚电平
     * 初始化状态，最后清挂起标志并开启中断，避免产生虚假第一步。
     */
    g_encoder_initialized = false;
    NVIC_DisableIRQ(ENCODER_IO_INT_IRQN);
    gpio_b = ENCODER_IO_PORT->DIN31_0;
    now = BSP_Time_Millis();

    for (index = 0u; index < ENCODER_COUNT; ++index) {
        g_encoders[index].pulse_count = 0;
        g_encoders[index].invalid_transitions = 0u;
        reset_speed_history(&g_encoders[index], now);
        g_last_ab[index] =
            read_ab_from_snapshot((EncoderIndex)index, gpio_b);
    }

    DL_GPIO_clearInterruptStatus(ENCODER_IO_PORT, ENCODER_ALL_PINS);
    NVIC_ClearPendingIRQ(ENCODER_IO_INT_IRQN);
    g_last_capture_ms = now;
    g_encoder_initialized = true;
    NVIC_EnableIRQ(ENCODER_IO_INT_IRQN);
}

void Encoder_Tick1msFromIsr(uint32_t now_ms)
{
    uint8_t index;

    if (!g_encoder_initialized ||
        ((uint32_t)(now_ms - g_last_capture_ms) <
         CONFIG_TICKS_FROM_HZ(CONFIG_TASK_ENCODER_CAPTURE_HZ))) {
        return;
    }

    /*
     * 只前进一步，而不是追赶多个历史点：SysTick 本身每 1 ms 调用，
     * 除非全局中断被错误地关闭超过一个快照周期，否则不会漏采。
     */
    g_last_capture_ms = now_ms;
    for (index = 0u; index < ENCODER_COUNT; ++index) {
        EncoderData *encoder = &g_encoders[index];
        uint8_t next_head = (uint8_t)((encoder->history_head + 1u) %
                                      ENCODER_SPEED_HISTORY_LENGTH);

        encoder->speed_history[next_head].count = encoder->pulse_count;
        encoder->speed_history[next_head].timestamp_ms = now_ms;
        encoder->history_head = next_head;
        if (encoder->history_count < ENCODER_SPEED_HISTORY_LENGTH) {
            ++encoder->history_count;
        }
        ++encoder->captured_sequence;
    }
}

void Encoder_Update(void)
{
    uint8_t index;

    for (index = 0u; index < ENCODER_COUNT; ++index) {
        EncoderData *encoder = &g_encoders[index];
        uint32_t primask;
        uint32_t newest_count;
        uint32_t newest_ms;
        uint32_t oldest_count;
        uint32_t oldest_ms;
        uint32_t elapsed_ms;
        uint32_t ratio_num;
        uint32_t ratio_den;
        uint32_t captured_sequence;
        uint8_t head;
        uint8_t count;
        uint8_t oldest_index;
        int32_t delta_count;
        int64_t numerator;
        int64_t denominator;

        /*
         * SysTick 可能随时推进环形缓冲区，因此在极短临界区内复制两个
         * 端点；64 位除法明确放在开中断的主循环区域。
         */
        primask = enter_critical();
        captured_sequence = encoder->captured_sequence;
        if (captured_sequence == encoder->processed_sequence) {
            leave_critical(primask);
            continue;
        }
        encoder->processed_sequence = captured_sequence;
        head = encoder->history_head;
        count = encoder->history_count;
        newest_count = encoder->speed_history[head].count;
        newest_ms = encoder->speed_history[head].timestamp_ms;
        oldest_index = (uint8_t)((head + 1u) %
                                 ENCODER_SPEED_HISTORY_LENGTH);
        oldest_count = encoder->speed_history[oldest_index].count;
        oldest_ms = encoder->speed_history[oldest_index].timestamp_ms;
        leave_critical(primask);

        if (count < ENCODER_SPEED_HISTORY_LENGTH) {
            encoder->last_delta_count = 0;
            encoder->last_sample_time_ms = 0u;
            encoder->rpm = 0;
            continue;
        }

        elapsed_ms = (uint32_t)(newest_ms - oldest_ms);
        delta_count = (int32_t)(newest_count - oldest_count);
        ratio_num = motor_to_wheel_ratio_num((EncoderIndex)index);
        ratio_den = motor_to_wheel_ratio_den((EncoderIndex)index);

        numerator = (int64_t)delta_count * 60000 * ratio_den;
        denominator = (int64_t)CONFIG_ENCODER_COUNTS_PER_MOTOR_REV *
                      ratio_num * elapsed_ms;
        if (denominator != 0) {
            numerator += (numerator >= 0) ?
                         (denominator / 2) : -(denominator / 2);
            encoder->rpm = (int32_t)(numerator / denominator);
        } else {
            encoder->rpm = 0;
        }
        encoder->last_delta_count = delta_count;
        encoder->last_sample_time_ms = elapsed_ms;
    }
}

int32_t Encoder_GetPulses(EncoderIndex encoder)
{
    EncoderSnapshot snapshot;
    return Encoder_GetSnapshot(encoder, &snapshot) ? snapshot.count : 0;
}

int32_t Encoder_GetRPM(EncoderIndex encoder)
{
    EncoderSnapshot snapshot;
    return Encoder_GetSnapshot(encoder, &snapshot) ? snapshot.rpm : 0;
}

bool Encoder_GetSnapshot(EncoderIndex encoder, EncoderSnapshot *snapshot)
{
    EncoderData *source;
    uint32_t primask;

    if (!is_valid_encoder(encoder) || (snapshot == NULL)) {
        return false;
    }

    source = &g_encoders[encoder];
    primask = enter_critical();
    snapshot->count = (int32_t)source->pulse_count;
    snapshot->delta_count = source->last_delta_count;
    snapshot->rpm = source->rpm;
    snapshot->sample_time_ms = source->last_sample_time_ms;
    snapshot->invalid_transitions = source->invalid_transitions;
    leave_critical(primask);
    return true;
}

void Encoder_Reset(EncoderIndex encoder)
{
    EncoderData *target;
    uint32_t primask;
    uint32_t gpio_b;

    if (!is_valid_encoder(encoder)) {
        return;
    }

    primask = enter_critical();
    target = &g_encoders[encoder];
    gpio_b = ENCODER_IO_PORT->DIN31_0;
    target->pulse_count = 0;
    target->invalid_transitions = 0u;
    reset_speed_history(target, BSP_Time_Millis());
    g_last_ab[encoder] = read_ab_from_snapshot(encoder, gpio_b);
    leave_critical(primask);
}

/**
 * @brief GPIOA/GPIOB 共用的中断组 1 服务程序。
 *
 * 当前工程只在 GROUP1 中启用了 GPIOB 的四个编码器脚。ISR 保持最短：
 * 获取挂起位、读取一次 GPIOB 快照、分别解码、清中断标志。
 */
void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_IO_PORT, ENCODER_ALL_PINS);
    uint32_t gpio_b;

    if (pending == 0u) {
        return;
    }

    gpio_b = ENCODER_IO_PORT->DIN31_0;

    if ((pending & ENCODER_M1_PINS) != 0u) {
        decode_transition(ENCODER_M1, gpio_b);
    }
    if ((pending & ENCODER_M2_PINS) != 0u) {
        decode_transition(ENCODER_M2, gpio_b);
    }

    DL_GPIO_clearInterruptStatus(ENCODER_IO_PORT, pending);
}
