/**
 * @file    drv8871.c
 * @brief   DRV8871 H 桥双电机驱动实现 (v0.3.0)
 *
 * M1: TIMG7 PWM @ 20 kHz on PA26(CCP0)/PA27(CCP1)
 * M2: TIMG8 PWM @ 20 kHz on PB15(CCP0)/PB16(CCP1)
 * 两路均由 SysConfig 生成外设、电源、IOMUX 和比较通道配置。
 *
 * DRV8871数据手册允许0~200 kHz输入PWM，且推荐在驱动态和制动态之间
 * 切换以获得慢衰减。本驱动正转时在10(驱动)与11(制动)之间切换，
 * 反转时在01(驱动)与11(制动)之间切换。
 *
 * 注意：软件只能限制占空比，电机峰值电流必须由ILIM电阻硬件限定：
 *   ITRIP(A) ≈ 64 / RILIM(kΩ)，RILIM不得小于15 kΩ。
 * 当前扩展板原理图RILIM=30 kΩ，对应典型限流约64/30=2.13 A。
 *
 * 数据手册输入真值表：
 *   IN1 IN2 | OUT1/OUT2状态
 *    0   0  | High-Z滑行，持续约1 ms后进入睡眠
 *    0   1  | 反向驱动
 *    1   0  | 正向驱动
 *    1   1  | 低侧制动
 *
 * 本驱动接口使用“千分比”而不是百分数：
 *   command=+250 表示一个方向25%等效驱动；
 *   command=-250 表示反方向25%等效驱动；
 *   command=0 表示00高阻停止。
 */

#include "drv8871.h"

#include "project_config.h"
#include "ti_msp_dl_config.h"

/**
 * 保存上一次下发给每个电机的有符号千分比指令。
 * 该数组只用于状态查询，不是PWM硬件寄存器的镜像；Stop()会将其清零。
 */
static int16_t g_command[DRV8871_MOTOR_COUNT];

/**
 * @brief 判断电机枚举值是否有效。
 *
 * 公共接口可能接收来自通信协议的参数；先检查范围可以避免数组越界，
 * 也防止无效枚举被错误地当成 M2 操作。
 */
static bool is_valid_motor(Drv8871Motor motor)
{
    return ((uint32_t)motor < (uint32_t)DRV8871_MOTOR_COUNT);
}

/* =========================== 占空比计算 ============================ */

/**
 * @brief 将0~1000千分比占空比换算为TIMG比较寄存器值。
 * @param duty_permille 无符号占空比，0=0%，1000=100%。
 * @return 适配当前边沿对齐、低初值PWM极性的比较值。
 *
 * SysConfig生成的0%初值为compare=period；因此换算需要取反：
 *
 *   active_count = period * duty / 1000
 *   compare      = period - active_count
 *
 * 结果示例（period=1600）：
 *   duty=0    -> compare=1600 -> 输出始终低
 *   duty=250  -> compare=1200 -> 输出高25%
 *   duty=1000 -> compare=0    -> 输出始终高
 */
static uint32_t duty_to_compare(uint16_t duty_permille)
{
    uint32_t active;

    if (duty_permille > (uint16_t)CONFIG_MOTOR_COMMAND_MAX) {
        duty_permille = (uint16_t)CONFIG_MOTOR_COMMAND_MAX;
    }

    active = ((uint32_t)CONFIG_MOTOR_PWM_PERIOD_COUNTS * duty_permille) /
             (uint32_t)CONFIG_MOTOR_COMMAND_MAX;
    return (uint32_t)CONFIG_MOTOR_PWM_PERIOD_COUNTS - active;
}

/**
 * @brief 原子式更新M1两个PWM比较通道的目标占空比。
 *
 * “原子式”表示两个比较通道在同一个短函数内连续更新；TIMG使用立即更新
 * 方法，最坏情况下两个写操作之间只相差数个CPU周期。
 */
static void set_inputs_m1(uint16_t in1, uint16_t in2)
{
    DL_TimerG_setCaptureCompareValue(MOTOR_M1_PWM_INST,
        duty_to_compare(in1), GPIO_MOTOR_M1_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(MOTOR_M1_PWM_INST,
        duty_to_compare(in2), GPIO_MOTOR_M1_PWM_C1_IDX);
}

/** @brief 更新M2的PB15/PB16两个TIMG8比较通道。 */
static void set_inputs_m2(uint16_t in1, uint16_t in2)
{
    DL_TimerG_setCaptureCompareValue(MOTOR_M2_PWM_INST,
        duty_to_compare(in1), GPIO_MOTOR_M2_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(MOTOR_M2_PWM_INST,
        duty_to_compare(in2), GPIO_MOTOR_M2_PWM_C1_IDX);
}

/**
 * @brief 按电机编号把两个逻辑输入占空比路由到对应定时器。
 * @param motor M1或M2。
 * @param in1 DRV8871 IN1占空比，单位千分比。
 * @param in2 DRV8871 IN2占空比，单位千分比。
 */
static void set_inputs(Drv8871Motor motor, uint16_t in1, uint16_t in2)
{
    if (motor == DRV8871_MOTOR_M1) {
        set_inputs_m1(in1, in2);
    } else if (motor == DRV8871_MOTOR_M2) {
        set_inputs_m2(in1, in2);
    }
}

/* ============================= 公共接口 ============================= */

/**
 * @brief 初始化指定电机的软件状态并启动对应PWM计数器。
 * @param motor DRV8871_MOTOR_M1或DRV8871_MOTOR_M2。
 *
 * 电源、IOMUX、PWM模式和20 kHz周期已经由SYSCFG_DL_init()配置。本函数
 * 先写00确保无驱动，再启动计数器；启动计数器本身不会让电机转动。
 */
void Drv8871_Init(Drv8871Motor motor)
{
    if (!is_valid_motor(motor)) {
        return;
    }

    g_command[motor] = 0;

    /* IN1=0、IN2=0：高阻滑行/睡眠，是最安全的初始化输出。 */
    set_inputs(motor, 0u, 0u);

    if (motor == DRV8871_MOTOR_M1) {
        DL_TimerG_startCounter(MOTOR_M1_PWM_INST);
    } else {
        DL_TimerG_startCounter(MOTOR_M2_PWM_INST);
    }
}

/**
 * @brief 下发有符号电机命令。
 * @param motor 目标电机。
 * @param command [-1000,+1000]千分比；超范围会在本层再次限幅。
 *
 * Brake-decay模式下，命令幅值mag代表每周期“驱动态”所占比例：
 *
 *   command > 0:
 *     IN1=1000，IN2=1000-mag
 *     IN2低时为10驱动，IN2高时为11制动。
 *
 *   command < 0:
 *     IN1=1000-mag，IN2=1000
 *     IN1低时为01驱动，IN1高时为11制动。
 *
 * command=0专门走Stop(COAST)，而不是输出11制动，避免零目标时电机持续
 * 通电发热，并允许DRV8871进入低功耗睡眠。
 */
void Drv8871_SetPermille(Drv8871Motor motor, int16_t command)
{
    uint16_t mag;

    if (!is_valid_motor(motor)) {
        return;
    }

    /* 公共驱动层必须自行防御，不能假设所有调用者都已正确限幅。 */
    if (command > CONFIG_MOTOR_COMMAND_MAX) {
        command = CONFIG_MOTOR_COMMAND_MAX;
    } else if (command < -CONFIG_MOTOR_COMMAND_MAX) {
        command = -CONFIG_MOTOR_COMMAND_MAX;
    }

    if (command == 0) {
        Drv8871_Stop(motor, DRV8871_STOP_COAST);
        return;
    }

    /* 保存有符号命令；绝对值mag仅用于占空比组合。 */
    g_command[motor] = command;
    mag = (uint16_t)((command < 0) ? -command : command);

#if CONFIG_MOTOR_PWM_BRAKE_DECAY
    /* 推荐的Drive/Brake慢衰减组合。 */
    if (command > 0) {
        set_inputs(motor, (uint16_t)CONFIG_MOTOR_COMMAND_MAX,
                   (uint16_t)(CONFIG_MOTOR_COMMAND_MAX - mag));
    } else {
        set_inputs(motor, (uint16_t)(CONFIG_MOTOR_COMMAND_MAX - mag),
                   (uint16_t)CONFIG_MOTOR_COMMAND_MAX);
    }
#else
    /* 可选的Drive/Coast组合：非驱动阶段为00高阻。 */
    if (command > 0) {
        set_inputs(motor, mag, 0u);
    } else {
        set_inputs(motor, 0u, mag);
    }
#endif
}

/**
 * @brief 以指定方式停止一个电机。
 * @param motor 目标电机。
 * @param mode COAST=00高阻滑行，BRAKE=11低侧制动。
 *
 * COAST适合目标速度归零和故障后断开驱动；BRAKE适合需要快速制动的上层
 * 动作，但长时间保持制动会改变机械手感和电流路径。
 */
void Drv8871_Stop(Drv8871Motor motor, Drv8871StopMode mode)
{
    if (!is_valid_motor(motor)) {
        return;
    }

    g_command[motor] = 0;
    if (mode == DRV8871_STOP_BRAKE) {
        set_inputs(motor, (uint16_t)CONFIG_MOTOR_COMMAND_MAX,
                   (uint16_t)CONFIG_MOTOR_COMMAND_MAX);
    } else {
        set_inputs(motor, 0u, 0u);
    }
}

/**
 * @brief 获取最近一次保存的有符号千分比命令。
 * @return 无效电机返回0，否则返回[-1000,+1000]。
 *
 * 该值适合调试/遥测，不应当用它反推电机真实电压、电流或转速。
 */
int16_t Drv8871_GetCommand(Drv8871Motor motor)
{
    if (!is_valid_motor(motor)) {
        return 0;
    }

    return g_command[motor];
}
