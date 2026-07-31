/**
 * @file    board.c
 * @brief   板级硬件初始化与基础控制实现
 *
 * 提供 Board_Init() 为中心的硬件平台初始化流程，
 * 以及板载蜂鸣器的 GPIO 控制。
 */

#include "board.h"

#include "bsp_time.h"
#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

void Board_Init(void)
{
    /*
     * 初始化顺序：
     *   1. SYSCFG_DL_init() -- SysConfig 生成的代码，配置电源、时钟树、
     *                          GPIO、PWM、UART、MCAN和40MHz CAN时钟等
     *   2. BSP_Time_Init()  -- 配置 SysTick 为 1ms 中断
     *   3. BSP_Uart_Init()  -- 初始化 UART0 TX DMA 状态与完成中断
     */
    SYSCFG_DL_init();
    BSP_Time_Init();
    BSP_Uart_Init();
}

void Board_EnterIdle(void)
{
    /*
     * WFI (Wait For Interrupt): ARMv6-M 特权指令
     * CPU 暂停取指，直到被 NVIC 挂起的中断唤醒。
     * 唤醒源：SysTick (1ms) / UART DMA / 编码器 GPIO
     */
    __WFI();
}

void Board_SetBuzzer(uint8_t enabled)
{
    /*
     * 蜂鸣器 GPIO (PA7) 控制
     * 通过 DriverLib 直接操作引脚电平，无 PWM 调频
     */
    if (enabled != 0u) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_PIN);   /* 高电平 -> 蜂鸣器响 */
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_PIN); /* 低电平 -> 蜂鸣器关 */
    }
}
