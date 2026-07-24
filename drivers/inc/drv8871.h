/**
 * @file    drv8871.h
 * @brief   TI DRV8871 H桥双电机驱动API
 *
 * M1 (TIMG7): PA26=CCP0/IN1, PA27=CCP1/IN2 — SysConfig 管理
 * M2 (TIMG8): PB15=CCP0/BIN1, PB16=CCP1/BIN2 — SysConfig 管理
 *
 * 按TI数据手册真值表：
 *   00=High-Z滑行/约1 ms后睡眠，10=正转，01=反转，11=低侧制动。
 * 默认PWM采用TI推荐的驱动/制动慢衰减，频率20 kHz。
 */

#ifndef DRV8871_H_
#define DRV8871_H_

#include <stdint.h>

typedef enum
{
    DRV8871_MOTOR_M1 = 0,   /**< 左/第一路：TIMG7，PA26/PA27。 */
    DRV8871_MOTOR_M2,       /**< 右/第二路：TIMG8，PB15/PB16。 */
    DRV8871_MOTOR_COUNT     /**< 电机数量哨兵，不是有效电机编号。 */
} Drv8871Motor;

typedef enum
{
    DRV8871_STOP_COAST = 0, /**< IN1=IN2=0，高阻滑行并可进入睡眠。 */
    DRV8871_STOP_BRAKE      /**< IN1=IN2=1，低侧制动。 */
} Drv8871StopMode;

/** 初始化一路PWM计数器并确保输出为00。 */
void Drv8871_Init(Drv8871Motor motor);

/**
 * 下发[-1000,+1000]有符号千分比命令；正负号选择方向，绝对值选择驱动比例。
 */
void Drv8871_SetPermille(Drv8871Motor motor, int16_t command);

/** 使用高阻滑行或低侧制动停止指定电机。 */
void Drv8871_Stop(Drv8871Motor motor, Drv8871StopMode mode);

/** 返回最近一次保存的命令，仅用于状态显示和调试。 */
int16_t Drv8871_GetCommand(Drv8871Motor motor);

#endif /* DRV8871_H_ */
