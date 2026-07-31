/**
 * @file    speed_control.c
 * @brief   双轮定频“转速前馈 + PI反馈”控制实现
 *
 * 每个车轮拥有独立积分和历史状态，不能共享同一个PID实例。
 * 左右轮也拥有独立前馈系数，用实测稳态“目标RPM->所需千分比输出”
 * 拟合。反馈初始使用PI（Kd=0），因为低线数编码器的滑动测速窗口会
 * 使微分项对量化台阶十分敏感。
 *
 * 一次有效控制更新的完整数据流：
 *
 *   编码器累计计数
 *       -> 滑动窗口RPM
 *       -> 按目标方向对齐目标/实际RPM
 *       -> 目标RPM线性前馈 + 左右轮独立PI修正
 *       -> 总输出限幅/抗积分饱和
 *       -> 静止区最小启动补偿
 *       -> 电机方向符号映射
 *       -> DRV8871千分比命令
 *
 * 本文件不直接操作PWM寄存器。应用层只处理“车轮RPM”和“千分比命令”，
 * 真值表与具体TIMG通道由drv8871.c封装，避免控制算法和硬件细节耦合。
 */

#include "speed_control.h"

#include <stddef.h>

#include "drv8871.h"
#include "encoder.h"
#include "project_config.h"

/**
 * @brief 一路车轮的堵转恢复状态。
 *
 * low_time_ms只在尚未进入助推时累计连续低速时间；active_time_ms只在
 * 助推状态累计，用于硬件卡死超时保护。状态完全由主循环维护，ISR不访问。
 */
typedef struct
{
    uint32_t low_time_ms;
    uint32_t active_time_ms;
    bool active;
} StallRecoveryState;

typedef struct
{
    PidController m1_pid;       /**< M1独立PID状态：积分、前次测量、D滤波。 */
    PidController m2_pid;       /**< M2独立PID状态，绝不能与M1共享积分。 */

    /**
     * 给屏幕和VOFA读取的最近一次完整控制快照。
     * 所有字段在一次Update中成组更新，外部模块无需访问内部PID对象。
     */
    SpeedControlSnapshot snapshot;

    /** 最近一次真正执行速度环的时间戳，用于周期门控和计算真实dt。 */
    uint32_t last_update_ms;

    /** 本次从停止切换到运行的时间戳，供方向保护启动宽限使用。 */
    uint32_t run_start_ms;

    /** M1连续反向样本数，达到配置阈值后置fault mask bit0。 */
    uint8_t m1_reverse_samples;

    /** M2连续反向样本数，达到配置阈值后置fault mask bit1。 */
    uint8_t m2_reverse_samples;

    /** M1堵转检测、助推和超时状态。 */
    StallRecoveryState m1_stall;

    /** M2堵转检测、助推和超时状态。 */
    StallRecoveryState m2_stall;
} SpeedControlState;

/** 模块唯一实例；裸机单线程使用，ISR不直接修改该对象。 */
static SpeedControlState g_speed;

/**
 * @brief 将外部目标约束在本工程支持的双向速度范围。
 * @param target 请求的车轮目标RPM。
 * @return [-CONFIG_SPEED_TARGET_MAX_RPM,+CONFIG_SPEED_TARGET_MAX_RPM]内目标。
 */
static float clamp_target(float target)
{
    if (target < -CONFIG_SPEED_TARGET_MAX_RPM) {
        return -CONFIG_SPEED_TARGET_MAX_RPM;
    }
    if (target > CONFIG_SPEED_TARGET_MAX_RPM) {
        return CONFIG_SPEED_TARGET_MAX_RPM;
    }
    return target;
}

/** 无libm依赖的浮点绝对值。 */
static float absolute_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/** 返回目标方向：前进=+1、后退=-1、停止=0。 */
static int32_t target_direction(float target)
{
    if (target > 0.0f) {
        return +1;
    }
    if (target < 0.0f) {
        return -1;
    }
    return 0;
}

/**
 * @brief 根据目标车轮RPM计算一路电机的线性前馈输出。
 * @param target_rpm 目标车轮速度，单位RPM；调用处保证它非负。
 * @param static_output 线性模型截距，单位输出千分比。
 * @param kv 线性模型斜率，单位输出千分比/RPM。
 * @return 未经过总输出限幅的模型基础输出。
 *
 * 前馈只依赖目标值，不依赖误差和历史：
 *   feedforward = static_output + kv * target_rpm
 *
 * 这里故意不单独把前馈限制到0~900。Pid_UpdateWithFeedforward()会把
 * “前馈+P+I+D”的总和统一限幅，并让抗积分饱和看到真实总输出。如果
 * 标定系数本身算出超过900，VOFA中的FF通道也能直接暴露配置不合理。
 */
static float calculate_feedforward(float target_rpm, float static_output,
                                   float kv)
{
#if (CONFIG_SPEED_FEEDFORWARD_ENABLE == 1u)
    return static_output + kv * target_rpm;
#else
    (void)target_rpm;
    (void)static_output;
    (void)kv;
    return 0.0f;
#endif
}

/**
 * @brief 将“前馈+PID”浮点输出转换为DRV8871整数千分比命令。
 * @param output 总控制器/启动补偿产生的非负千分比输出。
 * @param direction_sign 实物方向映射，只能是+1或-1。
 * @return 限制在[-1000,+1000]的整数命令。
 *
 * 先四舍五入再乘方向符号，保证+250和-250具有相同幅值。这里再次限幅，
 * 即使上层配置错误也不会把超范围比较值传给电机驱动。
 */
static int16_t output_to_command(float output, int32_t direction_sign)
{
    int32_t rounded = (int32_t)(output + 0.5f);

    rounded *= direction_sign;
    if (rounded > CONFIG_MOTOR_COMMAND_MAX) {
        rounded = CONFIG_MOTOR_COMMAND_MAX;
    } else if (rounded < -CONFIG_MOTOR_COMMAND_MAX) {
        rounded = -CONFIG_MOTOR_COMMAND_MAX;
    }
    return (int16_t)rounded;
}

/**
 * @brief 静止附近提供有限启动占空比。
 *
 * 前馈和P/I/D状态已经在通用控制器中按总输出限幅更新；这里只提高最终
 * 下发到H桥的命令，因此不会把启动补偿累加到积分器中。
 *
 * 只在以下条件同时成立时补偿：
 *   1. 车轮仍处于正负启动阈值之间；
 *   2. 前馈+PID确实请求正向驱动；
 *   3. 总输出低于可克服静摩擦的最小值。
 *
 * 仅当目标RPM本身达到配置门槛才启用托底，避免航向外环给出很小目标时
 * 直接产生250‰冲击。一旦车轮速度越过阈值，恢复使用原始总控制输出。
 */
static float apply_startup_minimum(float output, float rpm,
                                   float target_rpm)
{
    float release_rpm =
        target_rpm * CONFIG_SPEED_STARTUP_RELEASE_RATIO;

    if (release_rpm > CONFIG_SPEED_STARTUP_RPM_THRESHOLD) {
        release_rpm = CONFIG_SPEED_STARTUP_RPM_THRESHOLD;
    }
    if ((target_rpm >= CONFIG_SPEED_STARTUP_MIN_TARGET_RPM) &&
        (rpm > -CONFIG_SPEED_STARTUP_RPM_THRESHOLD) &&
        (rpm < release_rpm) &&
        (output > 0.0f) &&
        (output < CONFIG_SPEED_STARTUP_MIN_OUTPUT)) {
        return CONFIG_SPEED_STARTUP_MIN_OUTPUT;
    }
    return output;
}

/** 清除一路堵转检测的计时器和活动状态，不修改锁存故障位。 */
static void reset_stall_recovery(StallRecoveryState *state)
{
    state->low_time_ms = 0u;
    state->active_time_ms = 0u;
    state->active = false;
}

/**
 * @brief 把毫秒计时器饱和累加到指定上限，避免uint32_t回绕。
 */
static void accumulate_time(uint32_t *timer, uint32_t elapsed_ms,
                            uint32_t limit_ms)
{
    uint32_t remaining;

    if (*timer >= limit_ms) {
        return;
    }
    remaining = limit_ms - *timer;
    *timer += (elapsed_ms < remaining) ? elapsed_ms : remaining;
}

/**
 * @brief 更新一路堵转状态机。
 * @return true表示助推已超过安全时限，应锁存故障并停机。
 *
 * 状态转换：
 *   NORMAL
 *     -> RPM处于进入阈值且正误差足够大，连续DETECT_MS
 *   BOOST
 *     -> RPM达到EXIT_RPM：恢复NORMAL
 *     -> 持续TIMEOUT_MS仍未恢复：返回超时故障
 *
 * 进入/退出使用10/20 RPM迟滞。助推本身不写入PID积分；积分分离则负责
 * 在大误差期间冻结同方向积分，两层机制共同避免松手后的长时间超调。
 */
static bool update_stall_recovery(float target_rpm, float rpm,
                                  uint32_t elapsed_ms,
                                  StallRecoveryState *state)
{
#if (CONFIG_SPEED_STALL_RECOVERY_ENABLE == 1u)
    float error = target_rpm - rpm;
    bool near_zero =
        (rpm > -CONFIG_SPEED_STALL_ENTER_RPM) &&
        (rpm < CONFIG_SPEED_STALL_ENTER_RPM);

    if (state->active) {
        /*
         * 外环接近目标时会主动降低速度目标。此时正误差已不足以判定堵转，
         * 必须立即撤销助推，否则600‰托底会覆盖角度环的减速命令。
         */
        if ((target_rpm < CONFIG_SPEED_STALL_MIN_ERROR_RPM) ||
            (error < CONFIG_SPEED_STALL_MIN_ERROR_RPM)) {
            reset_stall_recovery(state);
            return false;
        }
        if (rpm >= CONFIG_SPEED_STALL_EXIT_RPM) {
            reset_stall_recovery(state);
            return false;
        }
        accumulate_time(&state->active_time_ms, elapsed_ms,
                        CONFIG_SPEED_STALL_TIMEOUT_MS);
        return state->active_time_ms >= CONFIG_SPEED_STALL_TIMEOUT_MS;
    }

    if (near_zero && (error >= CONFIG_SPEED_STALL_MIN_ERROR_RPM)) {
        accumulate_time(&state->low_time_ms, elapsed_ms,
                        CONFIG_SPEED_STALL_DETECT_MS);
        if (state->low_time_ms >= CONFIG_SPEED_STALL_DETECT_MS) {
            state->active = true;
            state->active_time_ms = 0u;
        }
    } else {
        state->low_time_ms = 0u;
    }
    return false;
#else
    (void)target_rpm;
    (void)rpm;
    (void)elapsed_ms;
    reset_stall_recovery(state);
    return false;
#endif
}

/**
 * @brief 堵转活动时提供不进入积分器的输出托底。
 *
 * CONFIG_SPEED_STALL_BOOST_OUTPUT如果误设得高于总输出上限，也会在这里
 * 限制到CONFIG_SPEED_OUTPUT_MAX，保证最终命令仍遵守速度环安全边界。
 */
static float apply_stall_boost(float output, bool stall_active)
{
#if (CONFIG_SPEED_STALL_RECOVERY_ENABLE == 1u)
    float boost = CONFIG_SPEED_STALL_BOOST_OUTPUT;

    if (boost > CONFIG_SPEED_OUTPUT_MAX) {
        boost = CONFIG_SPEED_OUTPUT_MAX;
    }
    if (stall_active && (output < boost)) {
        return boost;
    }
#else
    (void)stall_active;
#endif
    return output;
}

/**
 * @brief 更新一路编码器的连续反向样本计数。
 * @param rpm 当前车轮RPM。
 * @param counter 对应电机的连续反向计数器。
 *
 * 计数器使用饱和递增，避免uint8_t溢出重新回到0。任意一次RPM回到允许
 * 区域就清零，因此必须“连续”多次反向才会触发保护。
 */
#if (CONFIG_SPEED_DIRECTION_FAULT_ENABLE == 1u)
static void update_reverse_counter(float rpm, uint8_t *counter)
{
    if (rpm < -CONFIG_SPEED_DIRECTION_FAULT_RPM) {
        if (*counter < CONFIG_SPEED_DIRECTION_FAULT_SAMPLES) {
            ++(*counter);
        }
    } else {
        *counter = 0u;
    }
}
#endif

/**
 * @brief 初始化双轮速度控制器，并把两个H桥置于安全停止状态。
 * @param now_ms 当前1 ms系统时间，用作调度时间戳起点。
 *
 * 初始化顺序很重要：
 *   - 先构造左右轮独立PID配置；
 *   - 再清除快照、保护计数器和调度时间；
 *   - 最后明确下发00，使DRV8871输出高阻。
 */
void SpeedControl_Init(uint32_t now_ms)
{
    static const PidConfig m1_config = {
        .kp = CONFIG_SPEED_M1_KP,
        .ki = CONFIG_SPEED_M1_KI,
        .kd = CONFIG_SPEED_M1_KD,
        .output_min = CONFIG_SPEED_OUTPUT_MIN,
        .output_max = CONFIG_SPEED_OUTPUT_MAX,
        .integral_min = CONFIG_SPEED_INTEGRAL_MIN,
        .integral_max = CONFIG_SPEED_INTEGRAL_MAX,
        .integral_separation_threshold =
            CONFIG_SPEED_INTEGRAL_SEPARATION_RPM,
        .derivative_filter_tau_s = CONFIG_SPEED_D_FILTER_TAU_S,
        .derivative_on_measurement = true
    };
    static const PidConfig m2_config = {
        .kp = CONFIG_SPEED_M2_KP,
        .ki = CONFIG_SPEED_M2_KI,
        .kd = CONFIG_SPEED_M2_KD,
        .output_min = CONFIG_SPEED_OUTPUT_MIN,
        .output_max = CONFIG_SPEED_OUTPUT_MAX,
        .integral_min = CONFIG_SPEED_INTEGRAL_MIN,
        .integral_max = CONFIG_SPEED_INTEGRAL_MAX,
        .integral_separation_threshold =
            CONFIG_SPEED_INTEGRAL_SEPARATION_RPM,
        .derivative_filter_tau_s = CONFIG_SPEED_D_FILTER_TAU_S,
        .derivative_on_measurement = true
    };

    /*
     * 两个反馈控制器参数可以相同，但积分和历史状态必须位于不同对象。
     * PidConfig的output范围约束“前馈+反馈”总输出；integral范围只约束
     * PI修正中的积分分量。
     */
    (void)Pid_Init(&g_speed.m1_pid, &m1_config);
    (void)Pid_Init(&g_speed.m2_pid, &m2_config);
    g_speed.snapshot.target_rpm = 0.0f;
    g_speed.snapshot.m1_target_rpm = 0.0f;
    g_speed.snapshot.m2_target_rpm = 0.0f;
    g_speed.snapshot.m1_rpm = 0.0f;
    g_speed.snapshot.m2_rpm = 0.0f;
    g_speed.snapshot.m1_output_permille = 0.0f;
    g_speed.snapshot.m2_output_permille = 0.0f;
    g_speed.snapshot.running = false;
    g_speed.snapshot.direction_fault = false;
    g_speed.snapshot.direction_fault_mask = 0u;
    g_speed.snapshot.m1_stall_active = false;
    g_speed.snapshot.m2_stall_active = false;
    g_speed.snapshot.stall_fault = false;
    g_speed.snapshot.stall_fault_mask = 0u;
    (void)Pid_GetTerms(&g_speed.m1_pid, &g_speed.snapshot.m1_terms);
    (void)Pid_GetTerms(&g_speed.m2_pid, &g_speed.snapshot.m2_terms);
    g_speed.last_update_ms = now_ms;
    g_speed.run_start_ms = now_ms;
    g_speed.m1_reverse_samples = 0u;
    g_speed.m2_reverse_samples = 0u;
    reset_stall_recovery(&g_speed.m1_stall);
    reset_stall_recovery(&g_speed.m2_stall);

    /*
     * DRV8871真值表：IN1=IN2=0时H桥输出High-Z；持续约1 ms后器件进入
     * 低功耗睡眠。上电完成后再次显式写00，避免依赖定时器复位默认值。
     */
    Drv8871_Stop(DRV8871_MOTOR_M1, DRV8871_STOP_COAST);
    Drv8871_Stop(DRV8871_MOTOR_M2, DRV8871_STOP_COAST);
}

/**
 * @brief 立即停止双轮并复位速度环动态状态。
 *
 * 故障标志故意保持锁存，外环可以在停止后读取停机原因。新动作开始前由
 * SpeedControl_ClearFaults()显式确认并清除，防止外环连续下发非零目标
 * 自动绕过保护。
 */
void SpeedControl_Stop(void)
{
    g_speed.snapshot.target_rpm = 0.0f;
    g_speed.snapshot.m1_target_rpm = 0.0f;
    g_speed.snapshot.m2_target_rpm = 0.0f;
    g_speed.snapshot.running = false;
    g_speed.snapshot.m1_output_permille = 0.0f;
    g_speed.snapshot.m2_output_permille = 0.0f;
    g_speed.snapshot.m1_stall_active = false;
    g_speed.snapshot.m2_stall_active = false;
    g_speed.m1_reverse_samples = 0u;
    g_speed.m2_reverse_samples = 0u;
    reset_stall_recovery(&g_speed.m1_stall);
    reset_stall_recovery(&g_speed.m2_stall);
    Pid_Reset(&g_speed.m1_pid, 0.0f);
    Pid_Reset(&g_speed.m2_pid, 0.0f);
    (void)Pid_GetTerms(&g_speed.m1_pid, &g_speed.snapshot.m1_terms);
    (void)Pid_GetTerms(&g_speed.m2_pid, &g_speed.snapshot.m2_terms);
    Drv8871_Stop(DRV8871_MOTOR_M1, DRV8871_STOP_COAST);
    Drv8871_Stop(DRV8871_MOTOR_M2, DRV8871_STOP_COAST);
}

void SpeedControl_ClearFaults(void)
{
    if (g_speed.snapshot.running) {
        return;
    }
    g_speed.snapshot.direction_fault = false;
    g_speed.snapshot.direction_fault_mask = 0u;
    g_speed.snapshot.stall_fault = false;
    g_speed.snapshot.stall_fault_mask = 0u;
}

void SpeedControl_SetWheelTargets(float m1_target_rpm, float m2_target_rpm)
{
    float new_m1 = clamp_target(m1_target_rpm);
    float new_m2 = clamp_target(m2_target_rpm);
    int32_t old_m1_direction =
        target_direction(g_speed.snapshot.m1_target_rpm);
    int32_t old_m2_direction =
        target_direction(g_speed.snapshot.m2_target_rpm);
    int32_t new_m1_direction = target_direction(new_m1);
    int32_t new_m2_direction = target_direction(new_m2);
    bool was_running = g_speed.snapshot.running;
    bool m1_direction_changed =
        (old_m1_direction != new_m1_direction);
    bool m2_direction_changed =
        (old_m2_direction != new_m2_direction);
    float m1_magnitude;
    float m2_magnitude;

    if ((new_m1_direction == 0) && (new_m2_direction == 0)) {
        SpeedControl_Stop();
        return;
    }

    /* 锁存故障只能由新动作入口显式清除，周期目标更新不能绕过保护。 */
    if (g_speed.snapshot.direction_fault || g_speed.snapshot.stall_fault) {
        return;
    }

    if (!was_running || m1_direction_changed) {
        Pid_Reset(&g_speed.m1_pid, g_speed.snapshot.m1_rpm);
        g_speed.m1_reverse_samples = 0u;
        reset_stall_recovery(&g_speed.m1_stall);
    }
    if (!was_running || m2_direction_changed) {
        Pid_Reset(&g_speed.m2_pid, g_speed.snapshot.m2_rpm);
        g_speed.m2_reverse_samples = 0u;
        reset_stall_recovery(&g_speed.m2_stall);
    }
    if (!was_running || m1_direction_changed || m2_direction_changed) {
        g_speed.run_start_ms = g_speed.last_update_ms;
    }

    g_speed.snapshot.m1_target_rpm = new_m1;
    g_speed.snapshot.m2_target_rpm = new_m2;
    m1_magnitude = absolute_float(new_m1);
    m2_magnitude = absolute_float(new_m2);
    g_speed.snapshot.target_rpm =
        (m1_magnitude >= m2_magnitude) ? m1_magnitude : m2_magnitude;
    g_speed.snapshot.m1_stall_active = false;
    g_speed.snapshot.m2_stall_active = false;
    g_speed.snapshot.running = true;
}

void SpeedControl_SetTargetRPM(float target_rpm)
{
    SpeedControl_SetWheelTargets(target_rpm, target_rpm);
}

/**
 * @brief 响应一次PB21“按下事件”，循环切换速度档位。
 *
 * 本函数只处理已经消抖完成的一次事件，不读取GPIO，也不处理长按重复。
 * 超过最大目标后回到0，确保用户总能通过继续按键停机。
 */
void SpeedControl_CycleTarget(void)
{
    float next = g_speed.snapshot.target_rpm +
                 CONFIG_SPEED_BUTTON_STEP_RPM;

    if (next > CONFIG_SPEED_TARGET_MAX_RPM) {
        next = 0.0f;
    }
    SpeedControl_Stop();
    SpeedControl_ClearFaults();
    SpeedControl_SetTargetRPM(next);
}

/**
 * @brief 非阻塞执行一次速度控制任务。
 * @param now_ms 当前1 ms系统时间。
 *
 * 调用者可以在主循环中高频调用。函数先检查经过时间，不到控制周期立即
 * 返回；达到周期后使用真实elapsed_ms计算dt，因此偶发的主循环延迟不会
 * 把固定周期错误代入积分计算。前馈每次由当前目标直接计算，PI只修正
 * 模型误差和负载扰动。
 */
void SpeedControl_Update(uint32_t now_ms)
{
    EncoderSnapshot m1;
    EncoderSnapshot m2;
    uint32_t elapsed_ms =
        (uint32_t)(now_ms - g_speed.last_update_ms);
    float dt_s;
    float m1_feedforward;
    float m2_feedforward;
    float m1_output;
    float m2_output;
    float m1_target_magnitude;
    float m2_target_magnitude;
    float m1_aligned_rpm;
    float m2_aligned_rpm;
    int32_t m1_target_direction;
    int32_t m2_target_direction;
    uint8_t stall_timeout_mask = 0u;

    /* 周期尚未到：不读取快照、不计算PID、不改PWM，保持上次命令。 */
    if (elapsed_ms < CONFIG_TICKS_FROM_HZ(CONFIG_PID_SPEED_HZ)) {
        return;
    }
    g_speed.last_update_ms = now_ms;
    dt_s = (float)elapsed_ms * 0.001f;

    /*
     * 编码器中断只累计计数，Encoder_Update()已在主循环将历史快照换算为
     * RPM。这里读取的是最近一次稳定结果，不在速度控制层重复做64位除法。
     */
    (void)Encoder_GetSnapshot(ENCODER_M1, &m1);
    (void)Encoder_GetSnapshot(ENCODER_M2, &m2);
    g_speed.snapshot.m1_rpm = (float)m1.rpm;
    g_speed.snapshot.m2_rpm = (float)m2.rpm;

    /* 停止状态仍刷新RPM显示，但绝不运行PID，避免静止时积分悄悄累积。 */
    if (!g_speed.snapshot.running) {
        return;
    }

    /*
     * 速度PID始终控制正的“目标方向速度幅值”。例如M2目标=-60且实测=-55：
     * 对齐后目标=60、测量=55，误差仍为+5。最后下发电机前再乘目标方向，
     * 因而原有前馈和PI参数可以同时用于前进与后退。
     */
    m1_target_direction =
        target_direction(g_speed.snapshot.m1_target_rpm);
    m2_target_direction =
        target_direction(g_speed.snapshot.m2_target_rpm);
    m1_target_magnitude =
        absolute_float(g_speed.snapshot.m1_target_rpm);
    m2_target_magnitude =
        absolute_float(g_speed.snapshot.m2_target_rpm);
    m1_aligned_rpm =
        g_speed.snapshot.m1_rpm * (float)m1_target_direction;
    m2_aligned_rpm =
        g_speed.snapshot.m2_rpm * (float)m2_target_direction;

    /*
     * 检查的是“相对于本路目标方向”的RPM，而不是固定检查原始负RPM。
     * 这样角度环允许一路正转、一路反转，同时仍能发现目标/反馈极性错误。
     */
#if (CONFIG_SPEED_DIRECTION_FAULT_ENABLE == 1u)
    if ((uint32_t)(now_ms - g_speed.run_start_ms) >=
        CONFIG_SPEED_DIRECTION_GUARD_MS) {
        if (m1_target_direction != 0) {
            update_reverse_counter(m1_aligned_rpm,
                                   &g_speed.m1_reverse_samples);
        } else {
            g_speed.m1_reverse_samples = 0u;
        }
        if (m2_target_direction != 0) {
            update_reverse_counter(m2_aligned_rpm,
                                   &g_speed.m2_reverse_samples);
        } else {
            g_speed.m2_reverse_samples = 0u;
        }
    }

    /*
     * 每周期根据两个独立计数器重新构造位掩码：
     *   bit0=1 -> M1方向错误；bit1=1 -> M2方向错误。
     */
    g_speed.snapshot.direction_fault_mask = 0u;
    if (g_speed.m1_reverse_samples >=
        CONFIG_SPEED_DIRECTION_FAULT_SAMPLES) {
        g_speed.snapshot.direction_fault_mask |= 0x01u;
    }
    if (g_speed.m2_reverse_samples >=
        CONFIG_SPEED_DIRECTION_FAULT_SAMPLES) {
        g_speed.snapshot.direction_fault_mask |= 0x02u;
    }

    if (g_speed.snapshot.direction_fault_mask != 0u) {
        /*
         * 任一路方向错误都停止双轮，避免小车单轮继续驱动而突然转向。
         * Stop()不会清除fault mask，所以VOFA仍能看到停机原因；下一次
         * B21新动作会先显式ClearFaults()。
         */
        g_speed.snapshot.direction_fault = true;
        SpeedControl_Stop();
        return;
    }
#else
    /*
     * 电机和编码器极性已经完成架空标定。抗扰测试中外力会反拖车轮，
     * 其RPM可能短时与控制目标相反，不能把物理扰动误判成接线错误。
     */
    g_speed.m1_reverse_samples = 0u;
    g_speed.m2_reverse_samples = 0u;
    g_speed.snapshot.direction_fault_mask = 0u;
    g_speed.snapshot.direction_fault = false;
#endif

    /*
     * 正常启动的最初300 ms内，车轮可能尚未克服静摩擦，不能误判为堵转。
     * 宽限期结束后，两路状态机分别检测“接近0 RPM且正误差很大”。连续
     * 50 ms满足条件才进入助推，避免单个测速毛刺触发600‰命令。
     */
    if ((uint32_t)(now_ms - g_speed.run_start_ms) <
        CONFIG_SPEED_STALL_GUARD_MS) {
        reset_stall_recovery(&g_speed.m1_stall);
        reset_stall_recovery(&g_speed.m2_stall);
    } else {
        if ((m1_target_direction != 0) &&
            update_stall_recovery(m1_target_magnitude,
                                  m1_aligned_rpm, elapsed_ms,
                                  &g_speed.m1_stall)) {
            stall_timeout_mask |= 0x01u;
        } else if (m1_target_direction == 0) {
            reset_stall_recovery(&g_speed.m1_stall);
        }
        if ((m2_target_direction != 0) &&
            update_stall_recovery(m2_target_magnitude,
                                  m2_aligned_rpm, elapsed_ms,
                                  &g_speed.m2_stall)) {
            stall_timeout_mask |= 0x02u;
        } else if (m2_target_direction == 0) {
            reset_stall_recovery(&g_speed.m2_stall);
        }
    }
    g_speed.snapshot.m1_stall_active = g_speed.m1_stall.active;
    g_speed.snapshot.m2_stall_active = g_speed.m2_stall.active;

    if (stall_timeout_mask != 0u) {
        /*
         * 长时间堵转会让DRV8871、电机和电池持续承受大电流。任一路超过
         * 配置时限都停止双轮，并锁存具体故障位；再次按PB21启动才清除。
         */
        g_speed.snapshot.stall_fault = true;
        g_speed.snapshot.stall_fault_mask = stall_timeout_mask;
        SpeedControl_Stop();
        return;
    }

    /*
     * 左右轮目标相同，但实测维持同一RPM所需命令不同，所以分别计算前馈。
     * 前馈系数集中在project_config.h，换电机/电池/负载后只需重标定宏。
     */
    m1_feedforward = calculate_feedforward(
        m1_target_magnitude,
        CONFIG_SPEED_M1_FF_STATIC,
        CONFIG_SPEED_M1_FF_KV);
    m2_feedforward = calculate_feedforward(
        m2_target_magnitude,
        CONFIG_SPEED_M2_FF_STATIC,
        CONFIG_SPEED_M2_FF_KV);

    /*
     * 必须把前馈传入PID内部，而不是在Pid_Update()返回后简单相加。
     * 这样条件积分抗饱和判断的是“前馈+P+I+D”真实总输出：总输出已到
     * 900且误差仍为正时，不会继续积累正积分；反向误差仍可解除饱和。
     */
    if (m1_target_direction != 0) {
        m1_output = Pid_UpdateWithFeedforward(
            &g_speed.m1_pid, m1_target_magnitude, m1_aligned_rpm,
            m1_feedforward, dt_s);
    } else {
        m1_output = 0.0f;
    }
    if (m2_target_direction != 0) {
        m2_output = Pid_UpdateWithFeedforward(
            &g_speed.m2_pid, m2_target_magnitude, m2_aligned_rpm,
            m2_feedforward, dt_s);
    } else {
        m2_output = 0.0f;
    }

    /*
     * 启动补偿位于总控制器之后、方向映射之前。快照保存实际下发幅值，
     * 因此VOFA的M1_OUT/M2_OUT能看见250‰托底；PidTerms仍分别保存
     * FF/P/I/D和托底前的限幅总输出，便于区分模型、反馈和静摩擦补偿。
     */
    m1_output = apply_startup_minimum(
        m1_output, m1_aligned_rpm, m1_target_magnitude);
    m2_output = apply_startup_minimum(
        m2_output, m2_aligned_rpm, m2_target_magnitude);
    m1_output = apply_stall_boost(m1_output, g_speed.m1_stall.active);
    m2_output = apply_stall_boost(m2_output, g_speed.m2_stall.active);

    g_speed.snapshot.m1_output_permille = m1_output;
    g_speed.snapshot.m2_output_permille = m2_output;
    (void)Pid_GetTerms(&g_speed.m1_pid, &g_speed.snapshot.m1_terms);
    (void)Pid_GetTerms(&g_speed.m2_pid, &g_speed.snapshot.m2_terms);

    /*
     * Drv8871_SetPermille()对“正命令”令IN1保持高、IN2做互补PWM：
     * 有效区间IN1/IN2=10为正向驱动；其余区间=11为制动慢衰减。
     * COMMAND_SIGN把软件“车体前进”映射到镜像安装的电气方向；目标方向
     * 再决定前进或后退。角度环可下发M1=+RPM、M2=-RPM完成原地旋转。
     */
    Drv8871_SetPermille(
        DRV8871_MOTOR_M1,
        output_to_command(
            m1_output,
            CONFIG_SPEED_M1_COMMAND_SIGN * m1_target_direction));
    Drv8871_SetPermille(
        DRV8871_MOTOR_M2,
        output_to_command(
            m2_output,
            CONFIG_SPEED_M2_COMMAND_SIGN * m2_target_direction));
}

/**
 * @brief 复制最近一次速度控制快照。
 * @param snapshot 调用者提供的目标结构；NULL时返回false。
 * @return 成功复制返回true。
 */
bool SpeedControl_GetSnapshot(SpeedControlSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    *snapshot = g_speed.snapshot;
    return true;
}
