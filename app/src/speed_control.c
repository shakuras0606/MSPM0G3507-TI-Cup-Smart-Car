/**
 * @file    speed_control.c
 * @brief   双轮10 ms定速PID实现
 *
 * 每个车轮拥有独立积分和历史状态，不能共享同一个PID实例。
 * 初始参数使用PI（Kd=0），这是因为13PPR编码器与100 ms测速窗口会使
 * 微分项对量化台阶十分敏感。
 *
 * 一次有效控制更新的完整数据流：
 *
 *   编码器累计计数
 *       -> 100 ms滑动窗口RPM
 *       -> 目标RPM - 实际RPM
 *       -> 左右轮独立PI
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

typedef struct
{
    PidController m1_pid;       /**< M1独立PID状态：积分、前次测量、D滤波。 */
    PidController m2_pid;       /**< M2独立PID状态，绝不能与M1共享积分。 */

    /**
     * 给屏幕和VOFA读取的最近一次完整控制快照。
     * 所有字段在一次Update中成组更新，外部模块无需访问内部PID对象。
     */
    SpeedControlSnapshot snapshot;

    /** 最近一次真正执行PID的时间戳，单位ms；用于周期门控和计算真实dt。 */
    uint32_t last_update_ms;

    /** 本次从停止切换到运行的时间戳，供方向保护启动宽限使用。 */
    uint32_t run_start_ms;

    /** M1连续反向样本数，达到配置阈值后置fault mask bit0。 */
    uint8_t m1_reverse_samples;

    /** M2连续反向样本数，达到配置阈值后置fault mask bit1。 */
    uint8_t m2_reverse_samples;
} SpeedControlState;

/** 模块唯一实例；裸机单线程使用，ISR不直接修改该对象。 */
static SpeedControlState g_speed;

/**
 * @brief 将外部目标约束在本工程支持的单向速度范围。
 * @param target 请求的车轮目标RPM。
 * @return [0, CONFIG_SPEED_TARGET_MAX_RPM]内的安全目标。
 *
 * 当前速度环只实现“正目标+方向映射”。传入负值不会倒车，而是被限制为0。
 * 以后若实现倒车，需要同时重做PID输出范围、方向保护和按键状态机。
 */
static float clamp_target(float target)
{
    if (target < 0.0f) {
        return 0.0f;
    }
    if (target > CONFIG_SPEED_TARGET_MAX_RPM) {
        return CONFIG_SPEED_TARGET_MAX_RPM;
    }
    return target;
}

/**
 * @brief 将PID浮点输出转换为DRV8871整数千分比命令。
 * @param output PID/启动补偿产生的非负千分比输出。
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
 * PID 的 P/I/D 状态仍按原输出更新；这里只提高最终下发到 H 桥的命令，
 * 因此不会把启动补偿累加到积分器中。
 *
 * 只在以下条件同时成立时补偿：
 *   1. 车轮仍处于正负启动阈值之间；
 *   2. PID确实请求正向驱动；
 *   3. PID输出低于可克服静摩擦的最小值。
 *
 * 一旦车轮速度越过阈值，立即恢复使用原始PID输出。
 */
static float apply_startup_minimum(float output, float rpm)
{
    if ((rpm > -CONFIG_SPEED_STARTUP_RPM_THRESHOLD) &&
        (rpm < CONFIG_SPEED_STARTUP_RPM_THRESHOLD) &&
        (output > 0.0f) &&
        (output < CONFIG_SPEED_STARTUP_MIN_OUTPUT)) {
        return CONFIG_SPEED_STARTUP_MIN_OUTPUT;
    }
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
        .derivative_filter_tau_s = CONFIG_SPEED_D_FILTER_TAU_S,
        .derivative_on_measurement = true
    };

    /* 两个控制器参数可相同，但积分和历史状态必须位于不同对象。 */
    (void)Pid_Init(&g_speed.m1_pid, &m1_config);
    (void)Pid_Init(&g_speed.m2_pid, &m2_config);
    g_speed.snapshot.target_rpm = 0.0f;
    g_speed.snapshot.m1_rpm = 0.0f;
    g_speed.snapshot.m2_rpm = 0.0f;
    g_speed.snapshot.m1_output_permille = 0.0f;
    g_speed.snapshot.m2_output_permille = 0.0f;
    g_speed.snapshot.running = false;
    g_speed.snapshot.direction_fault = false;
    g_speed.snapshot.direction_fault_mask = 0u;
    (void)Pid_GetTerms(&g_speed.m1_pid, &g_speed.snapshot.m1_terms);
    (void)Pid_GetTerms(&g_speed.m2_pid, &g_speed.snapshot.m2_terms);
    g_speed.last_update_ms = now_ms;
    g_speed.run_start_ms = now_ms;
    g_speed.m1_reverse_samples = 0u;
    g_speed.m2_reverse_samples = 0u;

    /*
     * DRV8871真值表：IN1=IN2=0时H桥输出High-Z；持续约1 ms后器件进入
     * 低功耗睡眠。上电完成后再次显式写00，避免依赖定时器复位默认值。
     */
    Drv8871_Stop(DRV8871_MOTOR_M1, DRV8871_STOP_COAST);
    Drv8871_Stop(DRV8871_MOTOR_M2, DRV8871_STOP_COAST);
}

/**
 * @brief 设置两个车轮共同目标RPM。
 * @param target_rpm 请求目标；负数被限制为0，过大值被限制到配置上限。
 *
 * 目标为0时执行完整停止过程：清运行标志、清输出、清方向计数、复位PID
 * 积分并让H桥滑行。非零目标从停止状态启动时也先清积分，避免复用上一次
 * 运行留下的积分输出。
 */
void SpeedControl_SetTargetRPM(float target_rpm)
{
    float new_target = clamp_target(target_rpm);

    if (new_target == 0.0f) {
        /* 先更新软件状态，再停硬件；外部读取快照时不会看到“运行但零目标”。 */
        g_speed.snapshot.target_rpm = 0.0f;
        g_speed.snapshot.running = false;
        g_speed.snapshot.m1_output_permille = 0.0f;
        g_speed.snapshot.m2_output_permille = 0.0f;
        g_speed.m1_reverse_samples = 0u;
        g_speed.m2_reverse_samples = 0u;
        /*
         * 复位时把当前RPM作为初始测量，若以后启用D项，可避免下一次启动
         * 因测量历史不连续产生微分冲击。
         */
        Pid_Reset(&g_speed.m1_pid, g_speed.snapshot.m1_rpm);
        Pid_Reset(&g_speed.m2_pid, g_speed.snapshot.m2_rpm);
        (void)Pid_GetTerms(&g_speed.m1_pid, &g_speed.snapshot.m1_terms);
        (void)Pid_GetTerms(&g_speed.m2_pid, &g_speed.snapshot.m2_terms);
        Drv8871_Stop(DRV8871_MOTOR_M1, DRV8871_STOP_COAST);
        Drv8871_Stop(DRV8871_MOTOR_M2, DRV8871_STOP_COAST);
        return;
    }

    if (!g_speed.snapshot.running) {
        /*
         * 从停止状态启动时清除旧积分，避免上次运行残留导致占空比跳变。
         * DRV8871退出睡眠的典型启动时间约50 us，远小于10 ms控制周期。
         */
        Pid_Reset(&g_speed.m1_pid, g_speed.snapshot.m1_rpm);
        Pid_Reset(&g_speed.m2_pid, g_speed.snapshot.m2_rpm);
    }
    /* 用户再次给出非零目标，视为确认已检查故障，解除上一次锁存显示。 */
    g_speed.snapshot.direction_fault = false;
    g_speed.snapshot.direction_fault_mask = 0u;
    g_speed.snapshot.target_rpm = new_target;
    g_speed.snapshot.running = true;
    g_speed.run_start_ms = g_speed.last_update_ms;
    g_speed.m1_reverse_samples = 0u;
    g_speed.m2_reverse_samples = 0u;
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
    SpeedControl_SetTargetRPM(next);
}

/**
 * @brief 非阻塞执行一次速度控制任务。
 * @param now_ms 当前1 ms系统时间。
 *
 * 调用者可以在主循环中高频调用。函数先检查经过时间，不到控制周期立即
 * 返回；达到周期后使用真实elapsed_ms计算dt，因此偶发的主循环延迟不会
 * 把固定10 ms错误代入积分计算。
 */
void SpeedControl_Update(uint32_t now_ms)
{
    EncoderSnapshot m1;
    EncoderSnapshot m2;
    uint32_t elapsed_ms =
        (uint32_t)(now_ms - g_speed.last_update_ms);
    float dt_s;
    float m1_output;
    float m2_output;

    /* 周期尚未到：不读取快照、不计算PID、不改PWM，保持上次命令。 */
    if (elapsed_ms < CONFIG_SPEED_CONTROL_PERIOD_MS) {
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
     * 目标只有正转档。如果任一车轮稳定测得明显负RPM，通常是电机接线、
     * 命令符号或编码器方向不一致。先经过启动宽限，再要求连续多个反向
     * 样本，防止100 ms测速窗口刚填充或单个毛刺造成误停。
     */
    if ((uint32_t)(now_ms - g_speed.run_start_ms) >=
        CONFIG_SPEED_DIRECTION_GUARD_MS) {
        update_reverse_counter(g_speed.snapshot.m1_rpm,
                               &g_speed.m1_reverse_samples);
        update_reverse_counter(g_speed.snapshot.m2_rpm,
                               &g_speed.m2_reverse_samples);
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
         * SetTargetRPM(0)不会清除fault mask，所以VOFA仍能看到停机原因；
         * 下一次用户按下PB21设置非零目标时才解除锁存。
         */
        g_speed.snapshot.direction_fault = true;
        SpeedControl_SetTargetRPM(0.0f);
        return;
    }

    /* 左右轮使用相同目标，但分别使用自己的测量、积分和输出。 */
    m1_output = Pid_Update(&g_speed.m1_pid,
                           g_speed.snapshot.target_rpm,
                           g_speed.snapshot.m1_rpm, dt_s);
    m2_output = Pid_Update(&g_speed.m2_pid,
                           g_speed.snapshot.target_rpm,
                           g_speed.snapshot.m2_rpm, dt_s);

    /*
     * 启动补偿位于PID之后、方向映射之前。快照保存“实际下发幅值”，因此
     * VOFA ch3/ch4能直接看见250‰启动托底，而PID分项仍保持真实P/I/D。
     */
    m1_output = apply_startup_minimum(m1_output,
                                      g_speed.snapshot.m1_rpm);
    m2_output = apply_startup_minimum(m2_output,
                                      g_speed.snapshot.m2_rpm);

    g_speed.snapshot.m1_output_permille = m1_output;
    g_speed.snapshot.m2_output_permille = m2_output;
    (void)Pid_GetTerms(&g_speed.m1_pid, &g_speed.snapshot.m1_terms);
    (void)Pid_GetTerms(&g_speed.m2_pid, &g_speed.snapshot.m2_terms);

    /*
     * Drv8871_SetPermille()对“正命令”令IN1保持高、IN2做互补PWM：
     * 有效区间IN1/IN2=10为正向驱动；其余区间=11为制动慢衰减。
     * 当前实物COMMAND_SIGN=-1，因此PID正输出会映射为负命令，实际采用
     * 01/11组合。符号只负责适配安装方向，不改变PID误差的正负定义。
     */
    Drv8871_SetPermille(
        DRV8871_MOTOR_M1,
        output_to_command(m1_output, CONFIG_SPEED_M1_COMMAND_SIGN));
    Drv8871_SetPermille(
        DRV8871_MOTOR_M2,
        output_to_command(m2_output, CONFIG_SPEED_M2_COMMAND_SIGN));
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
