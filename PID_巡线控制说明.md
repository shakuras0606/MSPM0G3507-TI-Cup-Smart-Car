# 差速车巡线 PID 控制方案

当前车辆具有：

- 16 路巡线传感器；
- 左右车轮编码器；
- 左右 DRV8871 电机；
- 陀螺仪 Z 轴角速度；
- 差速转向结构。

推荐使用“巡线位置外环 + 陀螺仪角速度内环 + 双车轮速度环”，不要把
巡线误差、陀螺仪角速度和电机 PWM 全部塞进同一个 PID。

## 1. 信号定义

所有方向符号必须先统一：

- `line_error`：巡线模块输出，建议归一化到 `-1.0..+1.0`；
- `gyro_z_dps`：车体绕 Z 轴的角速度，单位 degree/s；
- `wheel_rpm_left/right`：编码器计算的有符号车轮 RPM；
- `base_rpm`：直线行驶基础车轮转速；
- `target_yaw_rate`：位置外环产生的目标角速度；
- `steering_rpm`：角速度内环产生的左右轮差速量。

必须通过实车低速测试确认：线偏右时的误差符号、左转时陀螺仪符号和
左右轮正转 RPM 符号与公式一致。

## 2. 推荐控制结构

### 2.1 巡线位置外环

输入是目标线位置 0 和实际 `line_error`，输出目标车体角速度：

```text
target_yaw_rate = line_pid(0, line_error)
```

该环优先使用 PD：

- P 决定偏线后的转向强度；
- D 抑制蛇形振荡；
- I 初始设为 0，只有确认存在长期固定偏差时才加入很小的 I。

建议频率 100 Hz，与巡线数据更新同步。输出应限制为车辆允许的最大目标
角速度，例如 `-max_yaw_rate..+max_yaw_rate`。

### 2.2 陀螺仪角速度内环

输入目标角速度和陀螺仪 Z 轴实际角速度，输出左右轮差速转速：

```text
steering_rpm = yaw_rate_pid(target_yaw_rate, gyro_z_dps)
```

推荐使用 PI 或带小 D 的 PID：

- P 快速跟踪期望转向速度；
- I 补偿左右电机、轮胎和重心差异；
- 陀螺仪本身噪声较大，D 必须带低通，初期可设为 0。

建议频率 200～500 Hz。若陀螺仪当前只以 100 Hz 提供新数据，就先将该环
也运行在 100 Hz，不要对重复数据假装进行 500 Hz 控制。

### 2.3 左右车轮速度前馈 + PI反馈环

基础车速和差速量合成为左右轮目标：

```text
left_target_rpm  = base_rpm - steering_rpm
right_target_rpm = base_rpm + steering_rpm
```

再由两个完全独立的“目标RPM前馈 + PI反馈”控制器输出电机千分比命令：

```text
left_pwm  = left_feedforward(left_target_rpm)
            + left_speed_pi(left_target_rpm, left_wheel_rpm)
right_pwm = right_feedforward(right_target_rpm)
            + right_speed_pi(right_target_rpm, right_wheel_rpm)
```

车轮 RPM 每 10 ms 更新，因此速度环运行 100 Hz。速度环通常先使用 PI：

- 前馈根据目标RPM立即给出大部分基础驱动力，改善档位切换响应；
- P 提供速度误差响应；
- I 只修正前馈模型误差和负载静差；
- D 对量化后的编码器速度很敏感，初期设为 0。

左右轮必须使用两个 `PidController` 实例，不能共享积分和历史状态。
前馈必须在PID内部参与总输出限幅和抗积分饱和，不能在限幅后的PID输出上
再次盲目相加。

## 3. 弯道降速

固定高速通过急弯通常会造成传感器越线。可根据目标角速度或巡线误差降低
基础转速：

```text
base_rpm = straight_rpm
           - yaw_slowdown_gain * abs(target_yaw_rate)
           - error_slowdown_gain * abs(line_error)
```

最终限制在 `minimum_rpm..straight_rpm`。先完成低速稳定巡线，再加入
弯道降速，不要同时调所有功能。

## 4. 丢线和安全状态

巡线模块报告丢线后，不应继续让位置 PID 对无效误差积分：

1. 暂停或复位巡线位置 PID；
2. 根据最后一次有效误差方向，以受限低速短时间搜线；
3. 超过设定时间仍未找回，左右电机停止；
4. 重新找到线后，对 PID 执行 `Pid_Reset()`，再平滑恢复速度。

上电、模式切换、急停和传感器故障时都应复位相关 PID，防止遗留积分导致
电机突然输出。

## 5. 调参顺序

建议把车轮架空或使用低速限幅，并按以下顺序逐环调节：

1. 关闭巡线环和陀螺仪环，只调左右车轮速度 PI；
2. 固定较低基础 RPM，调陀螺仪角速度 PI；
3. 最后调巡线位置 PD；
4. 稳定后再增加少量积分和弯道降速；速度前馈应先用定速数据标定；
5. 通过 VOFA 同时观察目标值、测量值、PID 输出及饱和标志。

外环带宽必须低于内环。经验上，位置环响应速度应明显慢于角速度环，
角速度环也不应要求速度环完成其做不到的快速转速变化。

## 6. PID API 示例

```c
PidController left_speed_pid;

const PidConfig left_speed_config = {
    .kp = 0.0f,                      /* 必须通过实车调参 */
    .ki = 0.0f,
    .kd = 0.0f,
    .output_min = -600.0f,           /* 初次测试限制到 60% */
    .output_max = 600.0f,
    .integral_min = -300.0f,
    .integral_max = 300.0f,
    .derivative_filter_tau_s = 0.02f,
    .derivative_on_measurement = true
};

Pid_Init(&left_speed_pid, &left_speed_config);

/* 每 10 ms 调用，dt 使用真实经过时间。 */
float command = Pid_Update(
    &left_speed_pid,
    left_target_rpm,
    left_measured_rpm,
    0.010f);
```

示例中的增益故意为 0，因为 PID 增益取决于电机电压、轮径、整车质量、
轮距、巡线传感器安装位置和陀螺仪响应，不能脱离实物安全地猜测。
