/**
 * @file    screen_task.h
 * @brief   ST7735 低优先级状态页
 *
 * 屏幕只负责显示已有快照，不参与编码器采样、WT61 解包或控制计算。
 */

#ifndef SCREEN_TASK_H_
#define SCREEN_TASK_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SCREEN_RACE_WAIT = 0,
    SCREEN_RACE_RUN = 1,
    SCREEN_RACE_BRAKE = 2,
    SCREEN_RACE_DONE = 3,
    SCREEN_RACE_FAULT = 4
} ScreenRaceState;

/** 初始化 ST7735 并绘制固定标签。 */
void ScreenTask_Init(void);

/**
 * 局部刷新 RPM、姿态、8路原始模拟量、阈值状态和当前算法的黑线位置。
 * 模拟加权模式显示为APOS，原数字等权模式显示为DPOS。
 */
void ScreenTask_Update(int32_t rpm_m1, int32_t rpm_m2,
                       float yaw_deg, bool wt61_online,
                       const uint16_t line_adc_values[16],
                       uint16_t line_raw_mask,
                       int16_t line_position, bool line_lost,
                       uint32_t race_elapsed_ms,
                       ScreenRaceState race_state,
                       bool infinite_mode);

#endif /* SCREEN_TASK_H_ */
