/**
 * @file    board_pins.h
 * @brief   MSPM0G3507 LQFP-64 引脚分配 (v0.5.0 WT61/DMA 版)
 *
 * ========================== 引脚分配总览 ==========================
 * M1 电机 (TIMG7):
 *   DRV8871 IN1/IN2:  PA26 / PA27
 *   编码器 A/B:       PB0  / PB1
 *
 * M2 电机 (TIMG8):
 *   DRV8871 BIN1/BIN2: PB15 / PB16
 *   编码器 A/B:       PB2  / PB3
 *
 * 编码器 PB0-PB3 均由 empty.syscfg 配置为双边沿 GPIO 中断，
 * 使用 GROUP1_IRQHandler 完成 x4 正交解码。
 *
 * 显示 (ST7735):
 *   SCLK PB9, MOSI PB8, RES PB10, DC PB11, CS PB14, BLK PB26
 *
 * 通信:
 *   PC UART TX/RX:    PA10 / PA11 (UART0, 921600, TX DMA_CH0)
 *   WT61 TX/RX:       PA9  / PA8  (UART1, 115200, RX DMA_CH1)
 *
 * 传感器:
 *   WT61TTL:          模块 TX -> PA9，模块 RX -> PA8
 *   BMI270:           暂停（原 PA8/PA9 与 WT61 冲突）
 *
 * 板载/扩展板:
 *   Buzzer PA7, 板载速度按键PB21(低有效), SWDIO/SWCLK PA19/PA20
 *   扩展板KEY1 PB12和KEY2 PB13当前不参与速度控制。
 *   扩展板将PB21引出为IMU_CS_A，使用板载按键时该网络不可被外设主动驱动。
 *
 * 线传感器 16ch: 待定
 *
 * 注意：PA8/PA9 和 PB6/PB7 是 UART1 的两组复用位置，并不是两路 UART。
 * 当前 UART1 已由 WT61 独占，原 PB6/PB7 蓝牙不能同时启用。
 * ===================================================================
 */

#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

typedef enum
{
    BOARD_LINE_SENSOR_TRANSPORT_UNASSIGNED = 0,
    BOARD_LINE_SENSOR_TRANSPORT_UART,
    BOARD_LINE_SENSOR_TRANSPORT_ADC,
    BOARD_LINE_SENSOR_TRANSPORT_GPIO,
    BOARD_LINE_SENSOR_TRANSPORT_SPI
} BoardLineSensorTransport;

#define BOARD_LINE_SENSOR_TRANSPORT BOARD_LINE_SENSOR_TRANSPORT_UNASSIGNED

#endif /* BOARD_PINS_H_ */
