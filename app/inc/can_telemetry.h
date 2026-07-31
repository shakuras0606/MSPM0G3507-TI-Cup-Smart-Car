/**
 * @file    can_telemetry.h
 * @brief   车体板到钢珠控制板的CAN遥测协议
 *
 * 当前主帧：
 *   标准ID 0x180，经典CAN，DLC=8，100 Hz。
 *
 *   Byte0..1  AX_mg，int16，小端
 *   Byte2..3  AY_mg，int16，小端
 *   Byte4..5  AZ_mg，int16，小端
 *   Byte6     sequence，每次成功提交发送后加1
 *   Byte7     状态/协议版本：
 *               bit0 ACCEL_VALID
 *               bit1 ACCEL_FRESH
 *               bit2 GYRO_FRESH
 *               bit3 ANGLE_FRESH
 *               bit4 保留，必须发送0
 *               bit7..5 协议版本
 *
 * 加速度是WT61传感器坐标系的原始物理量，包含重力分量。钢珠控制板必须
 * 根据实际安装方向选择纵向/横向轴，并在ACCEL_FRESH=0时关闭前馈。
 */

#ifndef CAN_TELEMETRY_H_
#define CAN_TELEMETRY_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t sent_frames;    /**< 成功提交的0x180帧数。 */
    uint32_t dropped_frames; /**< 本层因CAN忙或未就绪丢弃的帧数。 */
    uint32_t last_tx_ms;     /**< 最近成功提交发送的系统时间。 */
    uint8_t sequence;        /**< 下一帧将使用的序号。 */
} CanTelemetryStats;

void CanTelemetry_Init(void);

/** 读取最新WT61快照，组装并尝试发送一帧0x180加速度报文。 */
bool CanTelemetry_SendAcceleration(uint32_t now_ms);

bool CanTelemetry_GetStats(CanTelemetryStats *stats);

#endif /* CAN_TELEMETRY_H_ */
