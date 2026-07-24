/**
 * @file    encoder.h
 * @brief   双路 N20 正交编码器驱动 API（GPIO 双边沿中断，x4 解码）
 *
 * 物理连接：
 *   - M1：PB0=A，PB1=B
 *   - M2：PB2=A，PB3=B
 *
 * 计数在 GPIOB GROUP1 中断中完成；10 ms 计数快照由 SysTick 中断完成；
 * 主循环调用 Encoder_Update() 时只对最近 100 ms 快照执行 RPM 除法。
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

/** 编码器索引，与左右电机编号保持一致。 */
typedef enum
{
    ENCODER_M1 = 0,             /**< M1 电机编码器。 */
    ENCODER_M2,                 /**< M2 电机编码器。 */
    ENCODER_COUNT               /**< 编码器数量，不是有效索引。 */
} EncoderIndex;

/**
 * @brief 某一路编码器的一致性快照。
 *
 * 使用快照可以避免应用分别读取 count 和 rpm 时，中断刚好插入，
 * 从而得到来自两个不同时刻的数据。
 */
typedef struct
{
    int32_t count;              /**< 上电或最近一次复位后的累计 x4 计数。 */
    int32_t delta_count;        /**< 最近一个 RPM 采样窗口内的计数增量。 */
    int32_t rpm;                /**< 最近一次计算得到的有符号车轮 RPM。 */
    uint32_t sample_time_ms;    /**< 本次 RPM 滑动窗口的实际时间。 */
    uint32_t invalid_transitions; /**< 发现 AB 两位同时跳变的次数。 */
} EncoderSnapshot;

/**
 * @brief 初始化两路编码器并使能 GPIOB 中断。
 *
 * 必须在 SYSCFG_DL_init() 和 BSP_Time_Init() 之后调用一次。
 * 初始化会读取真实 AB 电平作为初始状态，避免上电凭空增加一个计数。
 */
void Encoder_Init(void);

/**
 * @brief 由 1 ms SysTick 中断调用的固定周期计数采样钩子。
 * @param now_ms 当前系统毫秒时间
 *
 * 只进行计数快照和数组索引更新，不做除法、浮点或显示操作。
 * 应用层不得主动调用。
 */
void Encoder_Tick1msFromIsr(uint32_t now_ms);

/**
 * @brief 在主循环中更新两路车轮 RPM。
 *
 * 10 ms 快照已由 SysTick 准时采集。本函数仅在有新快照时使用最近
 * CONFIG_ENCODER_MEASUREMENT_WINDOW_MS 的端点计算，不需要主循环准时调用。
 */
void Encoder_Update(void);

/** @brief 获取累计计数；索引无效时返回 0。 */
int32_t Encoder_GetPulses(EncoderIndex encoder);

/** @brief 获取最近计算的有符号车轮 RPM；索引无效时返回 0。 */
int32_t Encoder_GetRPM(EncoderIndex encoder);

/**
 * @brief 原子读取一路编码器的完整状态。
 * @return true 表示读取成功，false 表示参数或索引无效。
 */
bool Encoder_GetSnapshot(EncoderIndex encoder, EncoderSnapshot *snapshot);

/**
 * @brief 清零一路累计计数、RPM 和错误统计。
 *
 * 清零过程会短暂关闭全局中断，保证不会丢失“读-改-写”期间的计数。
 */
void Encoder_Reset(EncoderIndex encoder);

#endif /* ENCODER_H_ */
