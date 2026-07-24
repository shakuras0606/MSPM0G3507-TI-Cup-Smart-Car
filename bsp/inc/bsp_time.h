/**
 * @file    bsp_time.h
 * @brief   系统时基 BSP（SysTick 1ms 心跳）
 *
 * 使用 ARM Cortex-M0+ 内建 SysTick 定时器产生精确的 1ms 时间基准。
 *
 * 特性：
 *   - BSP_Time_Millis() 返回一个 32 位无符号毫秒计数器（约 49.7 天回绕）
 *   - BSP_Time_HasElapsed() 使用无符号减法实现回绕安全的延时判断
 *   - BSP_Time_DelayMs() 为忙等待阻塞延迟（仅用于初始化阶段，禁止在 ISR 中调用）
 *   - SysTick_Handler() 在中断上下文中递增计数器（ISR 安全）
 */

#ifndef BSP_TIME_H_
#define BSP_TIME_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化 SysTick，使能 1ms 定时中断
 * @note  必须在全局中断使能前调用；SysTick 配置为 CPUCLK_FREQ / 1000
 */
void BSP_Time_Init(void);

/**
 * @brief 获取自启动以来的毫秒数
 * @return 32 位无符号毫秒计数器值（自动回绕）
 */
uint32_t BSP_Time_Millis(void);

/**
 * @brief 回绕安全的延时判断
 * @param start_ms    起始时刻的毫秒值（来自 BSP_Time_Millis()）
 * @param duration_ms 需要等待的毫秒数
 * @return true  自 start_ms 起已过去 duration_ms 毫秒
 * @return false 尚未到达
 */
bool BSP_Time_HasElapsed(uint32_t start_ms, uint32_t duration_ms);

/**
 * @brief 忙等待阻塞延迟
 * @param milliseconds 延迟毫秒数
 * @warning 此函数不进入低功耗模式，仅在初始化阶段使用；禁止在 ISR 中调用
 */
void BSP_Time_DelayMs(uint32_t milliseconds);

#endif /* BSP_TIME_H_ */
