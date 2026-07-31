# 三串级 PID 巡线调试说明

## 1. 控制结构

本工程按照从慢到快的三层串级运行：

```text
16路巡线位置
    -> 位置外环 PID（50 Hz）
    -> 目标 Yaw 修正角
    -> WT61 Yaw 中环 PI + Z轴角速度阻尼（100 Hz）
    -> 左右轮目标 RPM
    -> M1/M2 独立速度 PI + 前馈（200 Hz）
    -> DRV8871 PWM
```

通道与位置定义：

- Y0在最左边、Y15在最右边，每一路位置权重可在`project_config.h`单独调整。
- 默认边缘增强：Y0/Y15为±1400，Y1/Y14为±1150，Y2/Y13为±950。
- 阵列中心目标为 0。
- M1 是左轮，M2 是右轮。

边缘权重调试原则：

- 直线和缓弯稳定时，优先成对增大Y2/Y13，再调整Y1/Y14、Y0/Y15。
- 每次左右同时增加10%～15%，例如Y2/Y13从±950改为±1050。
- Y0或Y15单独检测时，位置环可能已经达到Yaw限幅；继续放大最外侧权重
  不会再增强转向，这时应调整Y1～Y3、Y12～Y14或降低基础速度。
- 左右权重不要只改一边，除非已经确认传感器安装存在固定机械偏差。

位置环采用：

```text
line_error = 0 - line_position
yaw_offset = CONFIG_LINE_OUTPUT_SIGN * PID(line_error)
yaw_target = current_yaw + yaw_offset
```

Yaw 环输出差速 RPM，最终车轮目标为：

```text
M1_target = base_rpm + turn_rpm * CONFIG_YAW_M1_WHEEL_SIGN
M2_target = base_rpm + turn_rpm * CONFIG_YAW_M2_WHEEL_SIGN
```

本车实测负RPM才是物理前进，因此巡线基础速度默认为-30 RPM；相应的
`CONFIG_LINE_OUTPUT_SIGN`设为+1，以保持Y15方向仍向右修正。

## 2. B21 与丢线保护

- `CONFIG_B21_LINE_FOLLOW_MODE=1` 时，B21第一次按下开始巡线，再按一次停止。
- 开始时没有检测到黑线，不会启动电机。
- 短时丢线使用最后一次目标航向，基础速度由 `CONFIG_LINE_LOST_BASE_RPM` 决定。
- 连续丢线超过 `CONFIG_LINE_LOST_TIMEOUT_MS` 后停车并锁存丢线状态，需要重新按B21。
- 将 `CONFIG_B21_LINE_FOLLOW_MODE` 改为0，可恢复原来的Yaw增加90度测试。

## 3. VOFA 十个通道

```text
1  M1_TARGET_RPM
2  M1_ACTUAL_RPM
3  M2_TARGET_RPM
4  M2_ACTUAL_RPM
5  LINE_TARGET
6  LINE_POSITION
7  LINE_ERROR
8  YAW_TARGET
9  YAW_ACTUAL
10 YAW_TURN_RPM
```

## 4. 调试顺序

1. 先把车轮架空，确认两个负RPM目标对应物理前进。线在右侧时
   `LINE_POSITION>0`，M1目标应更负、M2目标应更接近0；线在左侧时相反。
   若转向错误，只翻转 `CONFIG_LINE_OUTPUT_SIGN`。
2. 保持已经调好的速度环和Yaw环参数不动，将基础速度先设为20~30 RPM。
3. 位置环先只调P，保持 `KI=0、KD=0`。逐步增加Kp，直到过弯足够快但直线尚未连续摆动。
4. 若直线左右摆动，先把Kp降低约20%；仍需要阻尼时再把Kd从0逐步增加到0.0001~0.0005。
5. 只有长期固定偏向一侧时才加Ki，可从0.0005开始；积分输出限幅保持在±3度。
6. 低速稳定后，每次增加5~10 RPM基础速度，并重新检查位置环响应。速度越高，通常需要更快的外环响应或更大的前视距离。

不要同时修改三个环。调巡线外环时，速度环和Yaw环必须先保持已验证参数不变。
