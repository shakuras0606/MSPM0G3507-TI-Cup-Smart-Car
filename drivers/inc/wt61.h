/**
 * @file    wt61.h
 * @brief   WT61TTL 姿态传感器驱动（UART1 RX DMA + 11 字节协议解析）
 *
 * 接线：
 *   WT61 TX -> MSPM0 PA9/UART1_RX
 *   WT61 RX -> MSPM0 PA8/UART1_TX（当前驱动只接收，保留用于后续配置）
 *   GND     -> GND
 *
 * 当前串口参数为 115200-8-N-1，必须与传感器上位机设置一致。
 * 标准输出帧固定 11 字节，以 0x55 开头。
 */

#ifndef WT61_H_
#define WT61_H_

#include <stdbool.h>
#include <stdint.h>

/** 对主循环公开的一致性姿态快照。 */
typedef struct
{
    float yaw_deg;                 /**< Z 轴航向角，单位 degree。 */
    float pitch_deg;               /**< Y 轴俯仰角，单位 degree。 */
    float roll_deg;                /**< X 轴横滚角，单位 degree。 */
    float accel_x_g;               /**< X 轴加速度，单位 g，包含重力分量。 */
    float accel_y_g;               /**< Y 轴加速度，单位 g，包含重力分量。 */
    float accel_z_g;               /**< Z 轴加速度，单位 g，包含重力分量。 */
    float temperature_c;           /**< 0x51 帧温度，单位 degree Celsius。 */
    float gyro_x_dps;              /**< X 轴角速度，单位 degree/s。 */
    float gyro_y_dps;              /**< Y 轴角速度，单位 degree/s。 */
    float gyro_z_dps;              /**< Z 轴角速度，单位 degree/s。 */
    uint32_t last_accel_ms;        /**< 最近有效 0x51 加速度帧的系统时间。 */
    uint32_t last_angle_ms;        /**< 最近有效 0x53 角度帧的系统时间。 */
    uint32_t last_gyro_ms;         /**< 最近有效 0x52 角速度帧的系统时间。 */
    uint32_t accel_frames;         /**< 校验正确的加速度帧累计数。 */
    uint32_t angle_frames;         /**< 校验正确的角度帧累计数。 */
    uint32_t gyro_frames;          /**< 校验正确的角速度帧累计数。 */
    uint32_t checksum_errors;      /**< 校验失败帧累计数。 */
    uint32_t rx_overflows;         /**< DMA 到解析器环形缓冲区的溢出次数。 */
    bool accel_valid;              /**< 至少收到过一帧正确加速度数据。 */
    bool angle_valid;              /**< 至少收到过一帧正确角度数据。 */
    bool gyro_valid;               /**< 至少收到过一帧正确角速度数据。 */
} WT61Snapshot;

/** 初始化 UART1 RX DMA、接收环形缓冲区和协议解析器。 */
void WT61_Init(void);

/**
 * @brief 在主循环解析 DMA 已接收的数据。
 *
 * 本函数不阻塞。应在 App_RunOnce() 每轮调用，尤其是软件 SPI 刷屏前后。
 */
void WT61_Process(void);

/** 原子取得当前姿态与诊断信息。 */
bool WT61_GetSnapshot(WT61Snapshot *snapshot);

/** 判断角度数据是否存在且未超过 timeout_ms。 */
bool WT61_IsAngleFresh(uint32_t timeout_ms);

/** 判断加速度数据是否存在且未超过 timeout_ms。 */
bool WT61_IsAccelerationFresh(uint32_t timeout_ms);

#endif /* WT61_H_ */
