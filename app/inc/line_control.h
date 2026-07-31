/**
 * @file    line_control.h
 * @brief   三串级巡线的最外层位置控制器
 *
 * 数据流：
 *   16路巡线位置 -> 位置PID输出目标Yaw修正角
 *   -> Yaw角度环输出左右轮差速RPM
 *   -> 两个车轮速度PI输出DRV8871 PWM
 */

#ifndef LINE_CONTROL_H_
#define LINE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"

typedef enum
{
    LINE_CONTROL_FAULT_NONE = 0,
    LINE_CONTROL_FAULT_LINE_LOST = 1,
    LINE_CONTROL_FAULT_YAW = 2
} LineControlFault;

typedef struct
{
    float target_position;     /**< 目标线位置，默认0。 */
    float measured_position;   /**< 线位置，Y0=-1000，Y15=+1000。 */
    float position_error;      /**< target-measured。 */
    float yaw_offset_deg;      /**< 位置PID输出的目标Yaw修正角。 */
    float target_yaw_deg;      /**< 实际送入Yaw中环的全局目标角。 */
    float base_rpm;            /**< 两轮共同基础前进速度。 */
    PidTerms terms;            /**< 位置环P/I/D分项。 */
    uint8_t active_count;      /**< 当前检测到黑线的通道数。 */
    bool enabled;              /**< true=三串级巡线运行。 */
    bool stopping;             /**< true=终点柔和减速过程。 */
    bool line_lost;            /**< 当前扫描未检测到黑线。 */
    LineControlFault fault;    /**< 丢线超时或Yaw故障。 */
} LineControlSnapshot;

/** 初始化位置PID；上电默认不驱动车轮。 */
void LineControl_Init(uint32_t now_ms);

/** 设置共同基础速度比例；1.0为普通速度，切换时由现有斜坡平滑过渡。 */
void LineControl_SetSpeedScale(float scale);

/** B21调用：停止状态开始巡线，运行状态立即停止。 */
void LineControl_Toggle(uint32_t now_ms);

/** 看到有效起跑线后开始巡线；无普通线/IMU异常时返回false。 */
bool LineControl_Start(uint32_t now_ms);

/** 终点使用：保持直行并按CONFIG_LINE_STOP_RPM_PER_S柔和减速。 */
void LineControl_BeginStop(uint32_t now_ms);

/** 停止前进并让Yaw中环锁住当前角度。 */
void LineControl_Stop(uint32_t now_ms);

/** 主循环高频调用，内部按CONFIG_PID_LINE_HZ门控。 */
void LineControl_Update(uint32_t now_ms);

/** 获取位置外环调试快照。 */
bool LineControl_GetSnapshot(LineControlSnapshot *snapshot);

#endif /* LINE_CONTROL_H_ */
