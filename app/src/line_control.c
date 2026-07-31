/**
 * @file    line_control.c
 * @brief   巡线位置外环：线位置PID输出Yaw目标修正角
 *
 * 通道方向已经固定：
 *   Y0在车体左侧  -> position=-1000
 *   Y15在车体右侧 -> position=+1000
 *
 * 位置PID统一使用error=target-position。由于现有Yaw正方向和车轮安装极性
 * 已经实测调通，二者之间只通过CONFIG_LINE_OUTPUT_SIGN做一次方向映射。
 */

#include "line_control.h"

#include <stddef.h>

#include "line_sensor16.h"
#include "project_config.h"
#include "yaw_control.h"

typedef struct
{
    PidController position_pid;
    LineControlSnapshot snapshot;
    uint32_t last_update_ms;
    uint32_t line_lost_since_ms;
    float last_target_yaw_deg;
    float filtered_position;
    float commanded_base_rpm;
    bool line_lost_timing;
} LineControlState;

static LineControlState g_line;

static float wrap_degrees(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float absolute_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/** 按真实dt限制基础速度变化，避免滚球受到阶跃纵向加速度。 */
static float move_toward(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

/**
 * 根据线偏差连续降速。只调共同基础速度，不削弱Yaw差速纠偏。
 * 直道使用BASE_RPM，偏差达到FULL后使用CURVE_MIN_RPM。
 */
static float calculate_curve_base(float position)
{
    float error = absolute_float(position - CONFIG_LINE_TARGET_POSITION);
    float base_magnitude = absolute_float(CONFIG_LINE_BASE_RPM);
    float minimum_magnitude = absolute_float(CONFIG_LINE_CURVE_MIN_RPM);
    float ratio;
    float magnitude;

    if (minimum_magnitude > base_magnitude) {
        minimum_magnitude = base_magnitude;
    }
    if (CONFIG_LINE_CURVE_SLOW_FULL_POSITION <=
        CONFIG_LINE_CURVE_SLOW_START_POSITION) {
        magnitude = (error >= CONFIG_LINE_CURVE_SLOW_FULL_POSITION) ?
                    minimum_magnitude : base_magnitude;
    } else if (error <= CONFIG_LINE_CURVE_SLOW_START_POSITION) {
        magnitude = base_magnitude;
    } else if (error >= CONFIG_LINE_CURVE_SLOW_FULL_POSITION) {
        magnitude = minimum_magnitude;
    } else {
        ratio = (error - CONFIG_LINE_CURVE_SLOW_START_POSITION) /
                (CONFIG_LINE_CURVE_SLOW_FULL_POSITION -
                 CONFIG_LINE_CURVE_SLOW_START_POSITION);
        magnitude = base_magnitude -
                    ratio * (base_magnitude - minimum_magnitude);
    }
    return (CONFIG_LINE_BASE_RPM >= 0.0f) ? magnitude : -magnitude;
}

static float update_base_ramp(float requested_base, float dt_s)
{
    float current_magnitude = absolute_float(g_line.commanded_base_rpm);
    float requested_magnitude = absolute_float(requested_base);
    float rate = (requested_magnitude > current_magnitude) ?
                 CONFIG_LINE_ACCEL_RPM_PER_S :
                 CONFIG_LINE_DECEL_RPM_PER_S;

    g_line.commanded_base_rpm = move_toward(
        g_line.commanded_base_rpm, requested_base, rate * dt_s);
    return g_line.commanded_base_rpm;
}

/** 停车时不关闭Yaw中环，而是把目标改为当前角，保持原有抗扰航向锁定。 */
static void hold_current_yaw(uint32_t now_ms)
{
    YawControlSnapshot yaw;

    YawControl_SetBaseRPM(0.0f);
    if (YawControl_GetSnapshot(&yaw) &&
        (yaw.fault == YAW_FAULT_NONE)) {
        YawControl_SetTargetYaw(yaw.current_yaw_deg, now_ms);
    }
}

void LineControl_Init(uint32_t now_ms)
{
    static const PidConfig line_pid_config = {
        .kp = CONFIG_LINE_KP,
        .ki = CONFIG_LINE_KI,
        .kd = CONFIG_LINE_KD,
        .output_min = CONFIG_LINE_YAW_OFFSET_MIN_DEG,
        .output_max = CONFIG_LINE_YAW_OFFSET_MAX_DEG,
        .integral_min = CONFIG_LINE_INTEGRAL_MIN_DEG,
        .integral_max = CONFIG_LINE_INTEGRAL_MAX_DEG,
        .integral_separation_threshold =
            CONFIG_LINE_INTEGRAL_SEPARATION,
        .derivative_filter_tau_s =
            CONFIG_LINE_DERIVATIVE_FILTER_TAU_S,
        .derivative_on_measurement = true
    };

    (void)Pid_Init(&g_line.position_pid, &line_pid_config);
    g_line.snapshot.target_position = CONFIG_LINE_TARGET_POSITION;
    g_line.snapshot.measured_position = 0.0f;
    g_line.snapshot.position_error = 0.0f;
    g_line.snapshot.yaw_offset_deg = 0.0f;
    g_line.snapshot.target_yaw_deg = 0.0f;
    g_line.snapshot.base_rpm = 0.0f;
    g_line.snapshot.active_count = 0u;
    g_line.snapshot.enabled = false;
    g_line.snapshot.stopping = false;
    g_line.snapshot.line_lost = true;
    g_line.snapshot.fault = LINE_CONTROL_FAULT_NONE;
    (void)Pid_GetTerms(&g_line.position_pid, &g_line.snapshot.terms);
    g_line.last_update_ms = now_ms;
    g_line.line_lost_since_ms = now_ms;
    g_line.last_target_yaw_deg = 0.0f;
    g_line.filtered_position = 0.0f;
    g_line.commanded_base_rpm = 0.0f;
    g_line.line_lost_timing = false;
    YawControl_SetBaseRPM(0.0f);
}

void LineControl_Stop(uint32_t now_ms)
{
    const LineSensor16Data *line = LineSensor16_GetData();

    g_line.snapshot.enabled = false;
    g_line.snapshot.stopping = false;
    g_line.snapshot.base_rpm = 0.0f;
    g_line.snapshot.yaw_offset_deg = 0.0f;
    g_line.snapshot.line_lost = line->line_lost;
    g_line.snapshot.active_count = line->active_count;
    g_line.line_lost_timing = false;
    g_line.commanded_base_rpm = 0.0f;
    Pid_Reset(&g_line.position_pid, (float)line->position);
    (void)Pid_GetTerms(&g_line.position_pid, &g_line.snapshot.terms);
    hold_current_yaw(now_ms);
}

bool LineControl_Start(uint32_t now_ms)
{
    const LineSensor16Data *line = LineSensor16_GetData();

    if (g_line.snapshot.enabled) {
        return true;
    }

    /* 没看到线时禁止突然起步；把车放回线上后需要再次按B21确认。 */
    if (line->line_lost) {
        g_line.snapshot.line_lost = true;
        g_line.snapshot.fault = LINE_CONTROL_FAULT_LINE_LOST;
        hold_current_yaw(now_ms);
        return false;
    }

    Pid_Reset(&g_line.position_pid, (float)line->position);
    g_line.snapshot.enabled = true;
    g_line.snapshot.stopping = false;
    g_line.snapshot.line_lost = false;
    g_line.snapshot.fault = LINE_CONTROL_FAULT_NONE;
    g_line.snapshot.measured_position = (float)line->position;
    g_line.snapshot.position_error =
        CONFIG_LINE_TARGET_POSITION - (float)line->position;
    g_line.snapshot.active_count = line->active_count;
    g_line.snapshot.base_rpm = 0.0f;
    g_line.filtered_position = (float)line->position;
    g_line.commanded_base_rpm = 0.0f;
    g_line.last_update_ms = now_ms;
    g_line.line_lost_timing = false;
    return true;
}

void LineControl_BeginStop(uint32_t now_ms)
{
    if (!g_line.snapshot.enabled) {
        return;
    }
    g_line.snapshot.stopping = true;
    g_line.last_update_ms = now_ms;
}

void LineControl_Toggle(uint32_t now_ms)
{
    if (g_line.snapshot.enabled) {
        g_line.snapshot.fault = LINE_CONTROL_FAULT_NONE;
        LineControl_Stop(now_ms);
        return;
    }
    (void)LineControl_Start(now_ms);
}

void LineControl_Update(uint32_t now_ms)
{
    const LineSensor16Data *line;
    YawControlSnapshot yaw;
    uint32_t elapsed_ms;
    float dt_s;
    float pid_output;
    float yaw_offset;
    float target_yaw;
    float requested_base;
    float filter_alpha;

    if (!g_line.snapshot.enabled) {
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - g_line.last_update_ms);
    if (elapsed_ms < CONFIG_TICKS_FROM_HZ(CONFIG_PID_LINE_HZ)) {
        return;
    }
    g_line.last_update_ms = now_ms;
    dt_s = (float)elapsed_ms * 0.001f;

    line = LineSensor16_GetData();
    g_line.snapshot.line_lost = line->line_lost;
    g_line.snapshot.active_count = line->active_count;

    if (!YawControl_GetSnapshot(&yaw) ||
        (yaw.fault != YAW_FAULT_NONE)) {
        g_line.snapshot.fault = LINE_CONTROL_FAULT_YAW;
        LineControl_Stop(now_ms);
        return;
    }

    /*
     * 终点减速期间不再追随横向质心，保持本拍航向并仅收小共同基础速度。
     * 差速环仍提供角速度阻尼，车体不会在终点线上突然甩尾。
     */
    if (g_line.snapshot.stopping) {
        g_line.commanded_base_rpm = move_toward(
            g_line.commanded_base_rpm, 0.0f,
            CONFIG_LINE_STOP_RPM_PER_S * dt_s);
        YawControl_SetTargetYaw(yaw.current_yaw_deg, now_ms);
        YawControl_SetBaseRPM(g_line.commanded_base_rpm);
        g_line.snapshot.base_rpm = g_line.commanded_base_rpm;
        g_line.snapshot.yaw_offset_deg = 0.0f;
        g_line.snapshot.target_yaw_deg = yaw.current_yaw_deg;
        if (absolute_float(g_line.commanded_base_rpm) <= 0.05f) {
            LineControl_Stop(now_ms);
        }
        return;
    }

    if (line->line_lost) {
        if (!g_line.line_lost_timing) {
            g_line.line_lost_since_ms = now_ms;
            g_line.line_lost_timing = true;
        }

        /* 短缝隙沿用最后航向，但基础速度由宏限制；默认0最安全。 */
        YawControl_SetTargetYaw(g_line.last_target_yaw_deg, now_ms);
        requested_base = update_base_ramp(
            CONFIG_LINE_LOST_BASE_RPM, dt_s);
        YawControl_SetBaseRPM(requested_base);
        g_line.snapshot.base_rpm = requested_base;

        if ((uint32_t)(now_ms - g_line.line_lost_since_ms) >=
            CONFIG_LINE_LOST_TIMEOUT_MS) {
            g_line.snapshot.fault = LINE_CONTROL_FAULT_LINE_LOST;
            LineControl_Stop(now_ms);
        }
        return;
    }
    g_line.line_lost_timing = false;

    filter_alpha = dt_s / (CONFIG_LINE_POSITION_FILTER_TAU_S + dt_s);
    g_line.filtered_position +=
        filter_alpha * ((float)line->position - g_line.filtered_position);
    g_line.snapshot.measured_position = g_line.filtered_position;
    pid_output = Pid_Update(
        &g_line.position_pid,
        CONFIG_LINE_TARGET_POSITION,
        g_line.snapshot.measured_position,
        dt_s);
    (void)Pid_GetTerms(&g_line.position_pid, &g_line.snapshot.terms);
    g_line.snapshot.position_error = g_line.snapshot.terms.error;

    /*
     * 目标Yaw采用“当前Yaw+修正角”，可连续通过弯道；如果固定加在上电
     * 航向上，小车只能回到一条绝对直线，无法跟随赛道曲率。
     */
    yaw_offset = pid_output * (float)CONFIG_LINE_OUTPUT_SIGN;
    target_yaw = wrap_degrees(yaw.current_yaw_deg + yaw_offset);
    YawControl_SetTargetYaw(target_yaw, now_ms);

    /* SetTarget可能因本拍WT61离线进入故障，确认后才允许给基础速度。 */
    if (!YawControl_GetSnapshot(&yaw) ||
        (yaw.fault != YAW_FAULT_NONE)) {
        g_line.snapshot.fault = LINE_CONTROL_FAULT_YAW;
        LineControl_Stop(now_ms);
        return;
    }
    requested_base = update_base_ramp(
        calculate_curve_base(g_line.snapshot.measured_position), dt_s);
    YawControl_SetBaseRPM(requested_base);

    g_line.snapshot.yaw_offset_deg = yaw_offset;
    g_line.snapshot.target_yaw_deg = target_yaw;
    g_line.snapshot.base_rpm = requested_base;
    g_line.snapshot.fault = LINE_CONTROL_FAULT_NONE;
    g_line.last_target_yaw_deg = target_yaw;
}

bool LineControl_GetSnapshot(LineControlSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_line.snapshot;
    return true;
}
