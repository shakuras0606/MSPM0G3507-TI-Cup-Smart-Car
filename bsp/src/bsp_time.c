/**
 * @file    bsp_time.c
 * @brief   系统时基 BSP 实现（基于 Cortex-M0+ SysTick）
 *
 * SysTick 是 ARM Cortex-M 内核的内建 24 位递减定时器，
 * 本模块将其配置为 1ms 周期中断以产生毫秒级心跳：
 *
 *   CPUCLK_FREQ = 32,000,000 Hz
 *   SysTick reload value = 32,000,000 / 1,000 = 32,000
 *
 * 回绕安全性：BSP_Time_Millis() 返回的 32 位计数器在 49.7 天后回绕，
 * BSP_Time_HasElapsed() 通过无符号减法正确处理回绕情况。
 */

#include "bsp_time.h"

#include "encoder.h"
#include "ti_msp_dl_config.h"

/**
 * @brief 毫秒计数器（volatile 保证 ISR 写 / 主循环读的可见性）
 *
 * SysTick_Handler() 中递增，主循环通过 BSP_Time_Millis() 读取
 */
static volatile uint32_t g_milliseconds;

void BSP_Time_Init(void)
{
    g_milliseconds = 0u;

    /*
     * SysTick_Config() 是 CMSIS 标准函数：
     *   - 设置重装值为 (reload - 1) = 31999
     *   - 清零当前值寄存器
     *   - 设置优先级为最低 (Cortex-M0+ 仅 2 位优先级)
     *   - 使能 SysTick 中断和计数器
     *
     * (void) 丢弃返回值（本实现不处理 reload 值超出 24 位的情况，
     *  32 MHz / 1000 = 32000 恰好小于 2^24 = 16777216）
     */
    (void) SysTick_Config(CPUCLK_FREQ / 1000u);
}

uint32_t BSP_Time_Millis(void)
{
    /*
     * 在 Cortex-M0+ 上，32 位读取是原子的（字对齐前提下），
     * 因此无需临界区保护即可安全地从主循环读取 ISR 递增的 g_milliseconds
     */
    return g_milliseconds;
}

bool BSP_Time_HasElapsed(uint32_t start_ms, uint32_t duration_ms)
{
    /*
     * 无符号减法的回绕安全判断：
     * 假设 start_ms = 0xFFFFFFF0, duration_ms = 0x20
     *   现在 BSP_Time_Millis() = 0x00000010
     *   (uint32_t)(0x00000010 - 0xFFFFFFF0) = 0x00000020 >= 0x20 -> true
     *
     * 在 32 位无符号算术下，即使计数器回绕也能正确判断
     */
    return (uint32_t)(BSP_Time_Millis() - start_ms) >= duration_ms;
}

void BSP_Time_DelayMs(uint32_t milliseconds)
{
    /*
     * 忙等待延迟——CPU 空转消耗周期，不依赖中断。
     * 延迟期间 CPU 不可做其他工作，仅适合初始化阶段的短延时。
     * delay_cycles() 是 TI DriverLib 提供的精确周期延迟函数。
     */
    while (milliseconds-- != 0u) {
        delay_cycles(CPUCLK_FREQ / 1000u);
    }
}

/**
 * @brief SysTick 中断服务例程
 *
 * 每 1ms 触发一次，递增全局毫秒计数器并调用编码器定时采样钩子。
 * 钩子只有每 10 ms 才复制两路累计计数，不执行 RPM 除法。
 */
void SysTick_Handler(void)
{
    ++g_milliseconds;
    Encoder_Tick1msFromIsr(g_milliseconds);
}
