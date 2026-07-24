/**
 * @file    board.h
 * @brief   板级硬件抽象层 API
 *
 * Board 层位于 BSP 和应用层之间，负责：
 *   - 调用 SysConfig 自动生成的 SYSCFG_DL_init() 完成外设时钟/引脚初始化
 *   - 依次初始化 BSP 子系统（SysTick、UART）
 *   - 提供统一的休眠入口 Board_EnterIdle()
 *   - 板载蜂鸣器 GPIO 控制
 *
 * SysConfig 管理的引脚请勿在代码中手动配置，一律通过 empty.syscfg 修改。
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>

/**
 * @brief 初始化整个硬件平台
 *
 * 调用顺序：
 *   1. SYSCFG_DL_init()   -- SysConfig 生成的外设/引脚/时钟初始化
 *   2. BSP_Time_Init()    -- SysTick 1ms 时基
 *   3. BSP_Uart_Init()    -- 双 UART 环形缓冲区 + 中断使能
 */
void Board_Init(void);

/**
 * @brief 进入低功耗空闲状态
 *
 * 执行 ARM Cortex-M0+ __WFI() 指令，CPU 暂停直到被中断唤醒。
 * 通常由 SysTick (1ms) 或 UART RX 中断唤醒。
 */
void Board_EnterIdle(void);

/**
 * @brief 控制板载蜂鸣器
 * @param enabled  非零值开启蜂鸣器，0 关闭
 */
void Board_SetBuzzer(uint8_t enabled);

#endif /* BOARD_H_ */
