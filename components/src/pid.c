/**
 * @file    pid.c
 * @brief   带抗积分饱和和微分滤波的离散 PID 实现。
 *
 * 本文件不包含任何电机、编码器或巡线业务逻辑，因此同一个算法可以分别
 * 创建多个独立实例用于速度环、方向环和巡线环。实例之间不得共享状态。
 */

#include "pid.h"

#include <stddef.h>

/**
 * @brief 将value限制在闭区间[minimum, maximum]。
 * 上层初始化已保证minimum<=maximum，因此热路径中不重复检查区间合法性。
 */
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

/**
 * @brief 检查会导致算法无定义的配置。
 *
 * Kp/Ki/Kd允许为0或负数，因为某些控制对象可能需要反向增益；业务层应
 * 自己决定符号。本函数只拒绝空指针、反向限幅区间和负滤波时间常数。
 */
static bool is_valid_config(const PidConfig *config)
{
    if (config == NULL) {
        return false;
    }
    if (config->output_min > config->output_max) {
        return false;
    }
    if (config->integral_min > config->integral_max) {
        return false;
    }
    if (config->derivative_filter_tau_s < 0.0f) {
        return false;
    }
    return true;
}

/** 复制配置并把控制器复位到零输出、无历史样本状态。 */
bool Pid_Init(PidController *controller, const PidConfig *config)
{
    if ((controller == NULL) || !is_valid_config(config)) {
        return false;
    }

    controller->config = *config;
    Pid_Reset(controller, 0.0f);
    return true;
}

/**
 * 在线替换配置但保留动态状态，适合以后通过上位机实时调参。
 * 修改增益不会自动清积分；若需要比较干净的阶跃响应，应主动Reset。
 */
bool Pid_SetConfig(PidController *controller, const PidConfig *config)
{
    if ((controller == NULL) || !is_valid_config(config)) {
        return false;
    }

    controller->config = *config;

    /*
     * 在线缩小积分限幅时，立即约束已有积分，防止下一控制周期突然输出
     * 超范围值。P、D 项会在下一次 Update 中按新参数计算。
     */
    controller->integral_state = clamp_float(
        controller->integral_state,
        controller->config.integral_min,
        controller->config.integral_max);
    controller->terms.integral = controller->integral_state;
    controller->terms.output = clamp_float(
        controller->terms.output,
        controller->config.output_min,
        controller->config.output_max);
    return true;
}

/**
 * 清空所有P/I/D分项和历史。
 * initial_measurement被保存为微分基准，但has_previous_sample=false保证
 * 第一次Update仍不会计算D项。
 */
void Pid_Reset(PidController *controller, float initial_measurement)
{
    if (controller == NULL) {
        return;
    }

    controller->terms.error = 0.0f;
    controller->terms.proportional = 0.0f;
    controller->terms.integral = 0.0f;
    controller->terms.derivative = 0.0f;
    controller->terms.output = clamp_float(
        0.0f,
        controller->config.output_min,
        controller->config.output_max);
    controller->terms.saturated = false;

    controller->previous_error = 0.0f;
    controller->previous_measurement = initial_measurement;
    controller->integral_state = 0.0f;
    controller->derivative_state = 0.0f;
    controller->has_previous_sample = false;
}

/**
 * 执行一次完整离散PID。所有运算使用调用者提供的真实dt_s，任务延迟时
 * 积分量和变化率仍按真实经过时间计算。
 */
float Pid_Update(PidController *controller, float setpoint,
                 float measurement, float dt_s)
{
    float error;
    float raw_derivative;
    float derivative_alpha;
    float candidate_integral;
    float unclamped_output;
    float output;
    bool pushes_further_into_saturation;

    if ((controller == NULL) || (dt_s <= 0.0f)) {
        return (controller != NULL) ? controller->terms.output : 0.0f;
    }

    /* 统一采用“目标-测量”；正误差意味着输出应向正方向增加。 */
    error = setpoint - measurement;

    /*
     * 第一次采样没有前一时刻的数据，D 项置零，避免初始化瞬间产生冲击。
     * 后续默认对 measurement 微分；目标值阶跃时不会产生 derivative kick。
     */
    if (!controller->has_previous_sample) {
        raw_derivative = 0.0f;
        controller->has_previous_sample = true;
    } else if (controller->config.derivative_on_measurement) {
        raw_derivative =
            -(measurement - controller->previous_measurement) / dt_s;
    } else {
        raw_derivative =
            (error - controller->previous_error) / dt_s;
    }

    /*
     * 一阶低通：
     *   filtered += alpha * (raw - filtered)
     *   alpha = dt / (tau + dt)
     */
    if (controller->config.derivative_filter_tau_s > 0.0f) {
        derivative_alpha =
            dt_s / (controller->config.derivative_filter_tau_s + dt_s);
        controller->derivative_state += derivative_alpha *
            (raw_derivative - controller->derivative_state);
    } else {
        controller->derivative_state = raw_derivative;
    }

    /* P项只取决于本周期误差，不包含任何历史。 */
    controller->terms.error = error;
    controller->terms.proportional = controller->config.kp * error;
    controller->terms.derivative =
        controller->config.kd * controller->derivative_state;

    /*
     * integral_state 直接保存“积分输出”，而不是单纯的误差积分。
     * 这样调整积分上下限时，限值与最终控制输出使用相同量纲。
     */
    /*
     * 先计算候选积分而不是立即提交。后面若发现它会把饱和推得更深，
     * 可以丢弃本次增量，实现条件积分抗饱和。
     */
    candidate_integral = controller->integral_state +
        controller->config.ki * error * dt_s;
    candidate_integral = clamp_float(
        candidate_integral,
        controller->config.integral_min,
        controller->config.integral_max);

    unclamped_output =
        controller->terms.proportional +
        candidate_integral +
        controller->terms.derivative;

    /*
     * 条件积分抗饱和：
     * 若输出已越过上限且正误差还会把输出继续向上推，或输出越过下限且
     * 负误差还会继续向下推，则拒绝本周期新增积分。反方向误差仍允许积分，
     * 使控制器可以快速退出饱和。
     */
    pushes_further_into_saturation =
        ((unclamped_output > controller->config.output_max) &&
         (error > 0.0f)) ||
        ((unclamped_output < controller->config.output_min) &&
         (error < 0.0f));

    if (!pushes_further_into_saturation) {
        controller->integral_state = candidate_integral;
    }

    /* 使用最终获准的积分状态重新计算并限幅输出。 */
    controller->terms.integral = controller->integral_state;
    unclamped_output =
        controller->terms.proportional +
        controller->terms.integral +
        controller->terms.derivative;
    output = clamp_float(
        unclamped_output,
        controller->config.output_min,
        controller->config.output_max);

    controller->terms.output = output;
    controller->terms.saturated = (output != unclamped_output);
    /* 最后更新历史，供下一周期微分；顺序不能提前。 */
    controller->previous_error = error;
    controller->previous_measurement = measurement;
    return output;
}

/** 复制调试分项，不暴露可写的内部状态指针。 */
bool Pid_GetTerms(const PidController *controller, PidTerms *terms)
{
    if ((controller == NULL) || (terms == NULL)) {
        return false;
    }

    *terms = controller->terms;
    return true;
}
