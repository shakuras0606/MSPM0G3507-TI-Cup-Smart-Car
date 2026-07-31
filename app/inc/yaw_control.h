/**
 * @file    yaw_control.h
 * @brief   WT61 Yaw角度外环：PID输出双轮差速目标RPM
 */

#ifndef YAW_CONTROL_H_
#define YAW_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"

typedef enum
{
    YAW_FAULT_NONE = 0,
    YAW_FAULT_IMU_OFFLINE = 1,
    YAW_FAULT_SPEED_LOOP = 2
} YawControlFault;

typedef struct
{
    float base_rpm;             /**< 两轮共同的车体前进基础速度，单位RPM。 */
    float target_yaw_deg;       /**< 相对上电基准的目标Yaw，范围[-180,180]。 */
    float current_yaw_deg;      /**< 相对上电基准的当前Yaw，单位degree。 */
    float error_deg;            /**< wrap(target-current)，范围[-180,180]。 */
    float gyro_z_dps;           /**< WT61 Z轴角速度，单位degree/s。 */
    float turn_rpm;             /**< 外环最终差速命令，带方向，单位RPM。 */
    float m1_target_rpm;        /**< 下发给M1速度内环的有符号目标。 */
    float m2_target_rpm;        /**< 下发给M2速度内环的有符号目标。 */
    PidTerms terms;             /**< 角度环P/I/D分项，输出单位RPM。 */
    bool active;                /**< true=当前偏差超出锁定显示范围。 */
    bool completed;             /**< true=当前处于锁定范围；外环仍持续计算。 */
    YawControlFault fault;      /**< 非0表示IMU或速度内环安全故障。 */
} YawControlSnapshot;

void YawControl_Init(uint32_t now_ms);

/**
 * @brief 设置相对上电零点的全局目标航向。
 *
 * 后续巡线方向环可连续调用本接口更新参考航向；不会停止速度内环或复位PI。
 */
void YawControl_SetTargetYaw(float target_yaw_deg, uint32_t now_ms);

/**
 * @brief 设置两轮共同的基础前进速度。
 *
 * 最终目标为base_rpm加上Yaw差速项；0表示原地航向控制。
 */
void YawControl_SetBaseRPM(float base_rpm);

/**
 * @brief 处理一次消抖后的B21按下事件。
 *
 * 每次按下都把长期锁定目标增加CONFIG_YAW_BUTTON_STEP_DEG；允许动作中追加。
 */
void YawControl_OnButtonPressed(uint32_t now_ms);

/** 主循环每轮调用，内部按CONFIG_PID_YAW_HZ门控。 */
void YawControl_Update(uint32_t now_ms);

bool YawControl_GetSnapshot(YawControlSnapshot *snapshot);

#endif /* YAW_CONTROL_H_ */
