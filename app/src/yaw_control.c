/**
 * @file    yaw_control.c
 * @brief   永久在线、以WT61 Yaw为反馈的航向-轮速串级控制
 *
 * 第一次收到有效WT61角度时，将当时航向定义为相对0度。此后角度外环每
 * 按CONFIG_PID_YAW_HZ永久计算，不存在“动作完成后退出”或
 * “误差过大停止追踪”的状态。
 * 外环使用角度PI产生差速轮速，并直接使用WT61 Z轴角速度提供阻尼；两个
 * 车轮的速度PI是内环。B21只负责将长期航向目标增加90度。
 */

#include "yaw_control.h"

#include <stddef.h>

#include "project_config.h"
#include "speed_control.h"
#include "wt61.h"

typedef struct
{
    PidController angle_pi;
    YawControlSnapshot snapshot;
    uint32_t last_update_ms;
    float yaw_zero_raw_deg;
    bool reference_valid;
} YawControlState;

static YawControlState g_yaw;

static float absolute_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

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

static bool angle_is_fresh(const WT61Snapshot *imu, uint32_t now_ms)
{
    return imu->angle_valid &&
           ((uint32_t)(now_ms - imu->last_angle_ms) <=
            CONFIG_WT61_STALE_TIMEOUT_MS);
}

static bool gyro_is_fresh(const WT61Snapshot *imu, uint32_t now_ms)
{
    return imu->gyro_valid &&
           ((uint32_t)(now_ms - imu->last_gyro_ms) <=
            CONFIG_WT61_STALE_TIMEOUT_MS);
}

static float relative_yaw(float raw_yaw_deg)
{
    return wrap_degrees(raw_yaw_deg - g_yaw.yaw_zero_raw_deg);
}

static void clear_wheel_command(void)
{
    SpeedControl_Stop();
    g_yaw.snapshot.base_rpm = 0.0f;
    g_yaw.snapshot.turn_rpm = 0.0f;
    g_yaw.snapshot.m1_target_rpm = 0.0f;
    g_yaw.snapshot.m2_target_rpm = 0.0f;
}

static void enter_fault(YawControlFault fault, bool invalidate_reference)
{
    clear_wheel_command();
    g_yaw.snapshot.active = false;
    g_yaw.snapshot.completed = false;
    g_yaw.snapshot.fault = fault;
    if (invalidate_reference) {
        g_yaw.reference_valid = false;
    }
}

/**
 * 上电或WT61离线恢复时，以当前原始Yaw建立相对0度坐标。离线期间无法
 * 确认车体是否被转动，因此恢复时重新归零比追逐失效前目标更安全。
 */
static void capture_zero_reference(const WT61Snapshot *imu, uint32_t now_ms)
{
    clear_wheel_command();
    SpeedControl_ClearFaults();
    Pid_Reset(&g_yaw.angle_pi, 0.0f);
    (void)Pid_GetTerms(&g_yaw.angle_pi, &g_yaw.snapshot.terms);

    g_yaw.yaw_zero_raw_deg = imu->yaw_deg;
    g_yaw.snapshot.target_yaw_deg = 0.0f;
    g_yaw.snapshot.current_yaw_deg = 0.0f;
    g_yaw.snapshot.error_deg = 0.0f;
    g_yaw.snapshot.gyro_z_dps =
        gyro_is_fresh(imu, now_ms) ? imu->gyro_z_dps : 0.0f;
    g_yaw.snapshot.active = false;
    g_yaw.snapshot.completed = true;
    g_yaw.snapshot.fault = YAW_FAULT_NONE;
    g_yaw.last_update_ms = now_ms;
    g_yaw.reference_valid = true;
}

void YawControl_Init(uint32_t now_ms)
{
    /*
     * 通用PID实例只负责角度P/I。D项不对离散Yaw差分，而在Update中直接
     * 使用WT61的gyro_z_dps，避免角度量化、±180度换向造成微分尖峰。
     */
    static const PidConfig angle_pi_config = {
        .kp = CONFIG_YAW_KP,
        .ki = CONFIG_YAW_KI,
        .kd = 0.0f,
        .output_min = CONFIG_YAW_OUTPUT_MIN_RPM,
        .output_max = CONFIG_YAW_OUTPUT_MAX_RPM,
        .integral_min = CONFIG_YAW_INTEGRAL_MIN_RPM,
        .integral_max = CONFIG_YAW_INTEGRAL_MAX_RPM,
        .integral_separation_threshold =
            CONFIG_YAW_INTEGRAL_SEPARATION_DEG,
        .derivative_filter_tau_s = 0.0f,
        .derivative_on_measurement = true
    };

    (void)Pid_Init(&g_yaw.angle_pi, &angle_pi_config);
    g_yaw.snapshot.base_rpm = 0.0f;
    g_yaw.snapshot.target_yaw_deg = 0.0f;
    g_yaw.snapshot.current_yaw_deg = 0.0f;
    g_yaw.snapshot.error_deg = 0.0f;
    g_yaw.snapshot.gyro_z_dps = 0.0f;
    g_yaw.snapshot.turn_rpm = 0.0f;
    g_yaw.snapshot.m1_target_rpm = 0.0f;
    g_yaw.snapshot.m2_target_rpm = 0.0f;
    g_yaw.snapshot.active = false;
    g_yaw.snapshot.completed = false;
    g_yaw.snapshot.fault = YAW_FAULT_NONE;
    (void)Pid_GetTerms(&g_yaw.angle_pi, &g_yaw.snapshot.terms);
    g_yaw.last_update_ms = now_ms;
    g_yaw.yaw_zero_raw_deg = 0.0f;
    g_yaw.reference_valid = false;
}

void YawControl_SetBaseRPM(float base_rpm)
{
    g_yaw.snapshot.base_rpm = clamp_float(
        base_rpm,
        -CONFIG_SPEED_TARGET_MAX_RPM,
        CONFIG_SPEED_TARGET_MAX_RPM);
}

void YawControl_SetTargetYaw(float target_yaw_deg, uint32_t now_ms)
{
    WT61Snapshot imu;

    (void)WT61_GetSnapshot(&imu);
    if (!angle_is_fresh(&imu, now_ms)) {
        enter_fault(YAW_FAULT_IMU_OFFLINE, true);
        return;
    }
    if (!g_yaw.reference_valid) {
        capture_zero_reference(&imu, now_ms);
    }

    /*
     * 只更新参考角，不停止速度内环，也不复位角度PI。巡线模块可以每个
     * 周期平滑更新目标，控制器的积分和连续状态不会被反复清空。
     */
    g_yaw.snapshot.target_yaw_deg = wrap_degrees(target_yaw_deg);
    g_yaw.snapshot.current_yaw_deg = relative_yaw(imu.yaw_deg);
    g_yaw.snapshot.error_deg =
        wrap_degrees(g_yaw.snapshot.target_yaw_deg -
                     g_yaw.snapshot.current_yaw_deg);
    g_yaw.snapshot.gyro_z_dps =
        gyro_is_fresh(&imu, now_ms) ? imu.gyro_z_dps : 0.0f;
    g_yaw.snapshot.fault = YAW_FAULT_NONE;
    g_yaw.snapshot.active = true;
    g_yaw.snapshot.completed = false;
}

void YawControl_OnButtonPressed(uint32_t now_ms)
{
    WT61Snapshot imu;
    float next_target;

    (void)WT61_GetSnapshot(&imu);
    if (!angle_is_fresh(&imu, now_ms)) {
        enter_fault(YAW_FAULT_IMU_OFFLINE, true);
        return;
    }
    if (!g_yaw.reference_valid) {
        capture_zero_reference(&imu, now_ms);
    }

    /*
     * B21也是速度故障后的人工确认入口。先停车再清故障，正常运行时不
     * 停止速度环，目标角可连续累加。
     */
    if (g_yaw.snapshot.fault == YAW_FAULT_SPEED_LOOP) {
        clear_wheel_command();
        SpeedControl_ClearFaults();
    }
    /* 90度阶跃测试从零积分开始，便于比较每次响应并避免旧偏置。 */
    Pid_Reset(&g_yaw.angle_pi, 0.0f);
    next_target = g_yaw.snapshot.target_yaw_deg +
                  CONFIG_YAW_BUTTON_STEP_DEG;
    YawControl_SetTargetYaw(next_target, now_ms);
}

void YawControl_Update(uint32_t now_ms)
{
    WT61Snapshot imu;
    SpeedControlSnapshot speed;
    uint32_t elapsed_ms =
        (uint32_t)(now_ms - g_yaw.last_update_ms);
    float dt_s;
    float absolute_error;
    float absolute_gyro;
    float angle_pi_output;
    float gyro_damping;
    float unclamped_turn_rpm;
    float turn_rpm;
    float turn_limit;

    (void)WT61_GetSnapshot(&imu);
    if (!angle_is_fresh(&imu, now_ms)) {
        if (g_yaw.snapshot.fault != YAW_FAULT_IMU_OFFLINE) {
            enter_fault(YAW_FAULT_IMU_OFFLINE, true);
        }
        return;
    }
    if (!g_yaw.reference_valid) {
        capture_zero_reference(&imu, now_ms);
        return;
    }

    g_yaw.snapshot.current_yaw_deg = relative_yaw(imu.yaw_deg);
    g_yaw.snapshot.error_deg =
        wrap_degrees(g_yaw.snapshot.target_yaw_deg -
                     g_yaw.snapshot.current_yaw_deg);
    g_yaw.snapshot.gyro_z_dps =
        gyro_is_fresh(&imu, now_ms) ? imu.gyro_z_dps : 0.0f;

    if (elapsed_ms < CONFIG_TICKS_FROM_HZ(CONFIG_PID_YAW_HZ)) {
        return;
    }
    g_yaw.last_update_ms = now_ms;
    dt_s = (float)elapsed_ms * 0.001f;

    /* 电机安全故障锁存；角度大小本身永远不会让Yaw外环退出。 */
    if (g_yaw.snapshot.fault == YAW_FAULT_SPEED_LOOP) {
        return;
    }
    (void)SpeedControl_GetSnapshot(&speed);
    if (speed.direction_fault || speed.stall_fault) {
        enter_fault(YAW_FAULT_SPEED_LOOP, false);
        return;
    }

    absolute_error = absolute_float(g_yaw.snapshot.error_deg);
    absolute_gyro = absolute_float(g_yaw.snapshot.gyro_z_dps);

    /*
     * 连续航向控制律：
     *   turn_rpm = Kp*Yaw误差 + Ki*误差积分 - Kd*实际Yaw角速度
     *
     * 前两项负责追回目标，最后一项像电子阻尼一样抵抗快速外力扰动并
     * 减少过冲。无论误差为1度还是150度，本段都会在每个周期执行。
     */
    angle_pi_output = Pid_Update(
        &g_yaw.angle_pi, g_yaw.snapshot.error_deg, 0.0f, dt_s);
    (void)Pid_GetTerms(&g_yaw.angle_pi, &g_yaw.snapshot.terms);
    /*
     * 实测WT61 gyro_z与Yaw角度增加方向相反：
     * corr(dYaw/dt, gyro_z)=-0.945。先用GYRO_SIGN统一成Yaw正方向，
     * 再取负反馈；不能直接假定模块角速度和欧拉角使用相同符号。
     */
    gyro_damping =
        -CONFIG_YAW_KD *
        ((float)CONFIG_YAW_GYRO_SIGN * g_yaw.snapshot.gyro_z_dps);
    unclamped_turn_rpm = angle_pi_output + gyro_damping;
    turn_rpm = clamp_float(
        unclamped_turn_rpm,
        CONFIG_YAW_OUTPUT_MIN_RPM,
        CONFIG_YAW_OUTPUT_MAX_RPM);

    /* 把直接角速度阻尼写入快照，调试器仍能看到完整P/I/D贡献。 */
    g_yaw.snapshot.terms.derivative = gyro_damping;
    g_yaw.snapshot.terms.output = turn_rpm;
    if (turn_rpm != unclamped_turn_rpm) {
        g_yaw.snapshot.terms.saturated = true;
    }

    /*
     * 只有小于浮点命令阈值才输出0，防止传感器末位噪声反复换向。这不是
     * 航向误差死区：角度PI仍持续运行，误差积分也会继续积累。
     */
    if (absolute_float(turn_rpm) < CONFIG_YAW_ZERO_COMMAND_RPM) {
        turn_rpm = 0.0f;
    }

    /*
     * 已确认存在实际扰动但PI输出尚不足以克服静摩擦时，施加最小差速。
     * 大误差和快速角速度扰动都会触发；接近静止零点时不会用固定PWM敲击。
     */
    if (((absolute_error >= CONFIG_YAW_STATIC_COMP_ERROR_DEG) ||
         (absolute_gyro >= CONFIG_YAW_STATIC_COMP_GYRO_DPS)) &&
        (absolute_float(turn_rpm) < CONFIG_YAW_MIN_TURN_RPM)) {
        if (turn_rpm > 0.0f) {
            turn_rpm = CONFIG_YAW_MIN_TURN_RPM;
        } else if (turn_rpm < 0.0f) {
            turn_rpm = -CONFIG_YAW_MIN_TURN_RPM;
        } else if (unclamped_turn_rpm > 0.0f) {
            turn_rpm = CONFIG_YAW_MIN_TURN_RPM;
        } else if (unclamped_turn_rpm < 0.0f) {
            turn_rpm = -CONFIG_YAW_MIN_TURN_RPM;
        } else {
            turn_rpm = (g_yaw.snapshot.error_deg >= 0.0f) ?
                       CONFIG_YAW_MIN_TURN_RPM :
                      -CONFIG_YAW_MIN_TURN_RPM;
        }
    }
    turn_rpm *= (float)CONFIG_YAW_OUTPUT_SIGN;

    /*
     * 预留基础前进速度后再限制差速量，保证base±turn均不超过速度环目标
     * 范围。这样左右轮不会因各自独立削顶而改变期望曲率。
     */
    turn_limit = CONFIG_SPEED_TARGET_MAX_RPM -
                 absolute_float(g_yaw.snapshot.base_rpm);
    if (turn_limit < 0.0f) {
        turn_limit = 0.0f;
    }

    turn_rpm = clamp_float(turn_rpm, -turn_limit, turn_limit);

    g_yaw.snapshot.turn_rpm = turn_rpm;
    g_yaw.snapshot.m1_target_rpm =
        g_yaw.snapshot.base_rpm +
        turn_rpm * (float)CONFIG_YAW_M1_WHEEL_SIGN;
    g_yaw.snapshot.m2_target_rpm =
        g_yaw.snapshot.base_rpm +
        turn_rpm * (float)CONFIG_YAW_M2_WHEEL_SIGN;

    /*
     * active/completed仅用于VOFA显示“纠偏/锁定”，不控制算法启停。即使
     * completed=true，上面的PI和角速度阻尼下一周期仍会照常计算。
     */
    g_yaw.snapshot.completed =
        (absolute_error <= CONFIG_YAW_LOCK_ERROR_DEG) &&
        (absolute_gyro <= CONFIG_YAW_GYRO_TOLERANCE_DPS) &&
        (absolute_float(turn_rpm) <= CONFIG_YAW_ZERO_COMMAND_RPM);
    g_yaw.snapshot.active = !g_yaw.snapshot.completed;
    g_yaw.snapshot.fault = YAW_FAULT_NONE;

    SpeedControl_SetWheelTargets(g_yaw.snapshot.m1_target_rpm,
                                 g_yaw.snapshot.m2_target_rpm);
}

bool YawControl_GetSnapshot(YawControlSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_yaw.snapshot;
    return true;
}
