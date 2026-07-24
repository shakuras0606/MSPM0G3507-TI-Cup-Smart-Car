/**
 * @file    speed_control.h
 * @brief   双车轮独立定速PID控制器
 *
 * 输入为编码器计算的车轮RPM，输出为DRV8871千分比PWM命令。
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
 * DRV8871前还会乘COMMAND_SIGN。因此调试“控制强度”看output，调试
 * “电气方向”看project_config.h中的COMMAND_SIGN。
 */
typedef struct
{
    float target_rpm;               /**< 两个车轮共同目标，单位：车轮RPM。 */
    float m1_rpm;                   /**< M1最近一次100 ms窗口车轮RPM。 */
    float m2_rpm;                   /**< M2最近一次100 ms窗口车轮RPM。 */
    float m1_output_permille;       /**< M1实际命令幅值，0~900千分比。 */
    float m2_output_permille;       /**< M2实际命令幅值，0~900千分比。 */
    PidTerms m1_terms;              /**< M1原始PID分项，不含启动补偿。 */
    PidTerms m2_terms;              /**< M2原始PID分项，不含启动补偿。 */
    bool running;                   /**< true=非零目标、速度环正在控制。 */
    bool direction_fault;           /**< 正目标却测得明显负RPM，已安全停机。 */
    uint8_t direction_fault_mask;    /**< bit0=M1方向错，bit1=M2方向错。 */
} SpeedControlSnapshot;

/**
 * @brief 初始化两路PID并确保两个电机处于滑行/睡眠状态。
 * @param now_ms 当前系统毫秒时间。
 */
void SpeedControl_Init(uint32_t now_ms);

/**
 * @brief 设置两个车轮的共同目标RPM，自动限制到允许范围。
 * @param target_rpm 期望车轮RPM；当前仅支持非负目标。
 */
void SpeedControl_SetTargetRPM(float target_rpm);

/** 板载PB21按键使用：目标按固定步长增加，超过最大值后回到0。 */
void SpeedControl_CycleTarget(void);

/**
 * @brief 主循环每轮调用；内部按配置周期执行一次双轮闭环。
 * @param now_ms 当前系统毫秒时间。
 */
void SpeedControl_Update(uint32_t now_ms);

/**
 * @brief 获取当前目标、测量值、输出和PID分项。
 * @param snapshot 输出目标，不能为NULL。
 * @return 成功复制返回true，空指针返回false。
 */
bool SpeedControl_GetSnapshot(SpeedControlSnapshot *snapshot);

#endif /* SPEED_CONTROL_H_ */
