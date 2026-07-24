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

/** 初始化 ST7735 并绘制固定标签。 */
void ScreenTask_Init(void);

/** 局部刷新 RPM 和姿态数据；建议 200 ms 调用一次。 */
void ScreenTask_Update(int32_t target_rpm, int32_t rpm_m1, int32_t rpm_m2,
                       float yaw_deg, float pitch_deg, float roll_deg,
                       bool wt61_online);

#endif /* SCREEN_TASK_H_ */
