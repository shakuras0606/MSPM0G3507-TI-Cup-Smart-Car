/**
 * @file    speed_control.h
 * @brief   双车轮独立“速度前馈 + PI反馈”控制器
 *
 * 输入为两路有符号目标/实测车轮RPM，输出为DRV8871千分比PWM命令。
 * 正RPM统一表示车体前进方向；负RPM表示后退。每一路使用独立前馈和PID。
 *
 * 线程/中断约束：
 *   - 所有SpeedControl_*接口都在主循环调用；
 *   - 编码器GPIO ISR只累计脉冲，不直接调用PID；
 *   - 屏幕和VOFA只读取Snapshot，不直接修改控制器内部状态。
 */

#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"

/**
 * @brief 供屏幕、VOFA和调试器读取的速度环快照。
 *
 * output_permille是启动补偿之后的实际命令幅值，始终为非负数；最终写入
 * DRV8871前还会乘COMMAND_SIGN。m1_terms/m2_terms中的feedforward、
 * proportional、integral、derivative和output是启动补偿之前的控制器分项。
 * 因此调试强度看output_permille，调模型看feedforward，调反馈看P/I/D，
 * 调电气方向看project_config.h中的COMMAND_SIGN。
 */
typedef struct
{
    float target_rpm;               /**< 两路目标绝对值的较大者，兼容原调试显示。 */
    float m1_target_rpm;            /**< M1有符号目标，正=车体前进，负=后退。 */
    float m2_target_rpm;            /**< M2有符号目标，正=车体前进，负=后退。 */
    float m1_rpm;                   /**< M1最近一次测速窗口车轮RPM。 */
    float m2_rpm;                   /**< M2最近一次测速窗口车轮RPM。 */
    float m1_output_permille;       /**< M1实际命令幅值，0~900千分比。 */
    float m2_output_permille;       /**< M2实际命令幅值，0~900千分比。 */
    PidTerms m1_terms;              /**< M1前馈/PID分项，不含启动补偿。 */
    PidTerms m2_terms;              /**< M2前馈/PID分项，不含启动补偿。 */
    bool running;                   /**< true=非零目标、速度环正在控制。 */
    bool direction_fault;           /**< 实测方向持续违背对应目标，已安全停机。 */
    uint8_t direction_fault_mask;    /**< bit0=M1目标/反馈反向，bit1=M2反向。 */
    bool m1_stall_active;            /**< M1正在执行无积分的堵转助推。 */
    bool m2_stall_active;            /**< M2正在执行无积分的堵转助推。 */
    bool stall_fault;                /**< 助推超时，双轮已安全停机。 */
    uint8_t stall_fault_mask;        /**< bit0=M1堵转超时，bit1=M2堵转超时。 */
} SpeedControlSnapshot;

/**
 * @brief 初始化两路前馈+PID控制器并确保电机处于滑行/睡眠状态。
 * @param now_ms 当前系统毫秒时间。
 */
void SpeedControl_Init(uint32_t now_ms);

/**
 * @brief 设置两个车轮的共同有符号目标RPM。
 * @param target_rpm 正=车体前进，负=后退，0=停止。
 */
void SpeedControl_SetTargetRPM(float target_rpm);

/**
 * @brief 分别设置两路有符号车轮目标，供Yaw/巡线外环调用。
 * @param m1_target_rpm M1目标，正=车体前进，负=后退。
 * @param m2_target_rpm M2目标，正=车体前进，负=后退。
 */
void SpeedControl_SetWheelTargets(float m1_target_rpm, float m2_target_rpm);

/** 双轮立即滑行停止并复位两路速度PID，不清除已锁存故障。 */
void SpeedControl_Stop(void);

/** 在电机停止时清除方向/堵转故障，开始新动作前调用。 */
void SpeedControl_ClearFaults(void);

/** 板载PB21按键使用：目标按固定步长增加，超过最大值后回到0。 */
void SpeedControl_CycleTarget(void);

/**
 * @brief 主循环每轮调用；内部按配置周期执行一次双轮闭环。
 * @param now_ms 当前系统毫秒时间。
 */
void SpeedControl_Update(uint32_t now_ms);

/**
 * @brief 获取当前目标、测量值、输出、前馈和PID分项。
 * @param snapshot 输出目标，不能为NULL。
 * @return 成功复制返回true，空指针返回false。
 */
bool SpeedControl_GetSnapshot(SpeedControlSnapshot *snapshot);

#endif /* SPEED_CONTROL_H_ */
