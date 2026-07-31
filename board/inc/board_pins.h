/**
 * @file    board_pins.h
 * @brief   MSPM0G3507 LQFP-64 引脚分配 (v0.6.0 8路UART巡线版)
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
 *   SCLK PB9, MOSI PB8, RES PB10, DC PB11, CS PB14, BLK直接接3.3V
 *
 * 通信:
 *   VOFA UART TX/RX:  PA10 / PA11 (UART0, 波特率见project_config.h, TX DMA_CH0)
 *   WT61 TX/RX:       PA9  / PA8  (UART1, 115200, RX DMA_CH1)
 *   8路巡线 TX/RX:    PA22 / PA21 (UART2, 115200, RX/TX DMA_CH2/CH3)
 *   CAN CANTX/CANRX:  PA12 / PA13 (CANFD0经典CAN，500kbps)
 *
 * 传感器:
 *   WT61TTL:          模块 TX -> PA9，模块 RX -> PA8
 *   BMI270:           暂停（PA8/PA9与WT61、PA12与CAN冲突）
 *
 * CAN:
 *   扩展板U3已把PA12/PA13逻辑电平转换为CAN_H/CAN_L，并带可接入的120Ω终端。
 *   两块MCU不能把CANTX/CANRX逻辑脚直接相连，应通过各自CAN收发器连接总线。
 *
 * 板载/扩展板:
 *   Buzzer PA7, 板载巡线启停键PB21(低有效), SWDIO/SWCLK PA19/PA20
 *   扩展板KEY1 PB12=单圈/无限巡线切换；KEY2 PB13仍保留。
 *   扩展板将PB21引出为IMU_CS_A，使用板载按键时该网络不可被外设主动驱动。
 *
 * Hiwonder 8路红外巡线模块:
 *   模块TX -> PA22/UART2_RX，模块RX -> PA21/UART2_TX，115200-8-N-1。
 *   模块按手册使用5V供电并与主控共地；软件读取原始模拟量并自行判线。
 *   原16路模块的PA24、PA28、PA31、PB26、PB27已全部释放。
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
    BOARD_LINE_SENSOR_TRANSPORT_I2C,
    BOARD_LINE_SENSOR_TRANSPORT_ADC,
    BOARD_LINE_SENSOR_TRANSPORT_GPIO,
    BOARD_LINE_SENSOR_TRANSPORT_SPI
} BoardLineSensorTransport;

#define BOARD_LINE_SENSOR_TRANSPORT BOARD_LINE_SENSOR_TRANSPORT_UART

#endif /* BOARD_PINS_H_ */
