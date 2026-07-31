/**
 * @file    can_telemetry.c
 * @brief   WT61加速度到经典CAN固定点报文的转换
 */

#include "can_telemetry.h"

#include <stddef.h>
#include <string.h>

#include "bsp_can.h"
#include "project_config.h"
#include "wt61.h"

#define CAN_STATUS_ACCEL_VALID    (1u << 0)
#define CAN_STATUS_ACCEL_FRESH    (1u << 1)
#define CAN_STATUS_GYRO_FRESH     (1u << 2)
#define CAN_STATUS_ANGLE_FRESH    (1u << 3)
#define CAN_STATUS_VERSION_SHIFT  (5u)

static CanTelemetryStats g_telemetry;

static int16_t acceleration_to_mg(float value_g)
{
    float value_mg = value_g * 1000.0f;

    if (value_mg >= 32767.0f) {
        return INT16_MAX;
    }
    if (value_mg <= -32768.0f) {
        return INT16_MIN;
    }
    value_mg += (value_mg >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)value_mg;
}

static void put_i16_le(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[0] = (uint8_t)(raw & 0xFFu);
    data[1] = (uint8_t)(raw >> 8u);
}

void CanTelemetry_Init(void)
{
    memset(&g_telemetry, 0, sizeof(g_telemetry));
    BSP_Can_Init();
}

bool CanTelemetry_SendAcceleration(uint32_t now_ms)
{
    WT61Snapshot imu;
    uint8_t payload[8] = {0u};
    uint8_t status;
    bool accel_fresh;
    bool gyro_fresh;
    bool angle_fresh;

    (void)WT61_GetSnapshot(&imu);
    accel_fresh = imu.accel_valid &&
                  ((uint32_t)(now_ms - imu.last_accel_ms) <=
                   CONFIG_CAN_ACCEL_STALE_TIMEOUT_MS);
    gyro_fresh = imu.gyro_valid &&
                 ((uint32_t)(now_ms - imu.last_gyro_ms) <=
                  CONFIG_WT61_STALE_TIMEOUT_MS);
    angle_fresh = imu.angle_valid &&
                  ((uint32_t)(now_ms - imu.last_angle_ms) <=
                   CONFIG_WT61_STALE_TIMEOUT_MS);

    if (imu.accel_valid) {
        put_i16_le(&payload[0], acceleration_to_mg(imu.accel_x_g));
        put_i16_le(&payload[2], acceleration_to_mg(imu.accel_y_g));
        put_i16_le(&payload[4], acceleration_to_mg(imu.accel_z_g));
    }
    payload[6] = g_telemetry.sequence;

    status = (uint8_t)(CONFIG_CAN_PROTOCOL_VERSION <<
                       CAN_STATUS_VERSION_SHIFT);
    if (imu.accel_valid) {
        status |= CAN_STATUS_ACCEL_VALID;
    }
    if (accel_fresh) {
        status |= CAN_STATUS_ACCEL_FRESH;
    }
    if (gyro_fresh) {
        status |= CAN_STATUS_GYRO_FRESH;
    }
    if (angle_fresh) {
        status |= CAN_STATUS_ANGLE_FRESH;
    }
    payload[7] = status;

    if (!BSP_Can_TrySend(CONFIG_CAN_ID_ACCEL, payload,
                         (uint8_t)sizeof(payload))) {
        ++g_telemetry.dropped_frames;
        return false;
    }

    ++g_telemetry.sequence;
    ++g_telemetry.sent_frames;
    g_telemetry.last_tx_ms = now_ms;
    return true;
}

bool CanTelemetry_GetStats(CanTelemetryStats *stats)
{
    if (stats == NULL) {
        return false;
    }
    *stats = g_telemetry;
    return true;
}
