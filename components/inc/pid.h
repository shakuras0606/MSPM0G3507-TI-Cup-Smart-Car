/**
 * @file    pid.h
 * @brief   通用离散 PID 控制器。
 *
 * 适用场景：
 *   - 巡线位置外环；
 *   - 陀螺仪 Z 轴角速度内环；
 *   - 左右车轮速度环。
 *
 * 控制器包含输出限幅、积分限幅、条件积分抗饱和、可选的测量值微分和
 * 一阶微分低通滤波。调用者必须提供真实控制周期 dt_s，不能假设任务
 * 每次都严格准时。
 *
 * 连续形式：
 *   u(t) = Kp*e(t) + Ki*integral(e dt) + Kd*de/dt
 *
 * 本实现离散形式：
 *   error = setpoint - measurement
 *   I[k]  = clamp(I[k-1] + Ki*error*dt)
 *   D[k]  = Kd * LPF(d(error)/dt 或 -d(measurement)/dt)
 *   output = clamp(P + I + D)
 */

#ifndef PID_H_
#define PID_H_

#include <stdbool.h>

/**
 * @brief PID 参数。
 *
 * 所有量纲由调用者保持一致。例如车轮速度环可以使用：
 *   setpoint/measurement = RPM，output = DRV8871 千分比命令。
 */
typedef struct
{
    float kp;                       /**< P输出/误差；决定当前误差的即时响应。 */
    float ki;                       /**< P输出/(误差*s)；决定误差随时间累积速度。 */
    float kd;                       /**< P输出*s/误差；抑制快速变化但放大测量噪声。 */

    float output_min;               /**< 控制器最小输出。 */
    float output_max;               /**< 控制器最大输出。 */
    float integral_min;             /**< 积分项最小值，量纲与输出相同。 */
    float integral_max;             /**< 积分项最大值，量纲与输出相同。 */

    /**
     * 微分低通滤波时间常数，单位 s。
     * 设为 0 表示不滤波；数值越大，微分越平滑但相位延迟越大。
     */
    float derivative_filter_tau_s;

    /**
     * true：对测量值微分，避免目标值阶跃引起微分冲击，控制环推荐；
     * false：对误差微分。
     */
    bool derivative_on_measurement;
} PidConfig;

/** @brief 最近一次 PID 计算的分项，便于 VOFA 调参与故障定位。 */
typedef struct
{
    float error;                    /**< setpoint - measurement。 */
    float proportional;             /**< P 项。 */
    float integral;                 /**< I 项。 */
    float derivative;               /**< D 项。 */
    float output;                   /**< 限幅后的最终输出。 */
    bool saturated;                 /**< 最终输出是否触及上下限。 */
} PidTerms;

/**
 * @brief PID 控制器实例。
 *
 * 每个控制环必须使用独立实例，不能让左右轮或内外环共享状态。
 * 应用层通常只读取 terms，其他状态由 pid.c 管理。
 */
typedef struct
{
    PidConfig config;
    PidTerms terms;

    float previous_error;           /**< 上周期误差，误差微分模式使用。 */
    float previous_measurement;     /**< 上周期测量值，测量微分模式使用。 */
    float integral_state;           /**< 已乘Ki的积分输出，量纲与最终输出相同。 */
    float derivative_state;         /**< 尚未乘Kd的滤波后变化率。 */
    bool has_previous_sample;       /**< false时首拍D项强制为0，避免微分冲击。 */
} PidController;

/**
 * @brief 初始化 PID 实例。
 * @param controller 需要初始化的实例。
 * @param config     参数配置。
 * @return 参数有效时返回 true。
 */
bool Pid_Init(PidController *controller, const PidConfig *config);

/**
 * @brief 在线更新 PID 参数，并保留当前积分状态。
 *
 * 调参过程中使用此接口不会突然清空积分项；积分状态会自动限制到新的范围。
 * 如需完全重新开始，应随后调用 Pid_Reset()。
 */
bool Pid_SetConfig(PidController *controller, const PidConfig *config);

/**
 * @brief 清除积分、微分和历史状态。
 * @param initial_measurement 当前测量值，用于建立无冲击的微分初值。
 */
void Pid_Reset(PidController *controller, float initial_measurement);

/**
 * @brief 执行一次 PID 计算。
 * @param setpoint    目标值。
 * @param measurement 当前测量值。
 * @param dt_s        本次控制间隔，单位 s，必须大于 0。
 * @return 限幅后的控制输出；参数无效时返回上一次输出。
 */
float Pid_Update(PidController *controller, float setpoint,
                 float measurement, float dt_s);

/**
 * @brief 读取最近一次 PID 分项。
 * @return 成功返回 true，空指针返回 false。
 */
bool Pid_GetTerms(const PidController *controller, PidTerms *terms);

#endif /* PID_H_ */
