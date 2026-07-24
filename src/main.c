/**
 * @file    main.c
 * @brief   TI3507-CAR 电赛车固件入口点
 *
 * 本文件为 MSPM0G3507 电赛车控制器的程序入口。固件采用裸机超级循环
 * (bare-metal superloop) 架构，不依赖任何 RTOS：
 *
 *   1. Board_Init()      -- 硬件初始化（SysConfig 外设、SysTick、UART）
 *   2. App_Init()        -- 应用层初始化（协议解析器、驱动、开机画面）
 *   3. 主循环 App_RunOnce() -> Board_EnterIdle() 交替执行
 *
 * 主循环中通过 __WFI() 进入低功耗休眠，由 SysTick / UART 中断唤醒，
 * 实现事件驱动的运行模型。
 *
 * @author  TI3507 Team
 * @version 0.5.0
 * @date    2026
 */

#include "app.h"
#include "board.h"

int main(void)
{
    Board_Init();   /* 初始化系统时钟、GPIO、PWM、UART、SysTick */
    App_Init();     /* 初始化协议解析、传感器驱动、显示开机画面 */

    while (1) {

        App_RunOnce();      /* WT61解析、RPM换算、VOFA DMA和低频刷屏 */

        Board_EnterIdle();  /* __WFI() 进入低功耗休眠，等待中断唤醒 */
    }
}
