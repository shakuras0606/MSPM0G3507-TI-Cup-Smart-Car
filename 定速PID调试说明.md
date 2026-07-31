# 双轮定速PID调试说明

## 1. 控制结构

两个车轮分别使用独立的线性前馈和PI反馈：

```text
共同目标RPM ─┬─ M1前馈 + M1 PI ─ DRV8871 M1 ─ M1车轮 ─ M1编码器 ─┐
             └─ M2前馈 + M2 PI ─ DRV8871 M2 ─ M2车轮 ─ M2编码器 ─┘
```

控制周期为10 ms，编码器每10 ms采集累计计数，使用最近100 ms滑动窗口
计算车轮RPM。微分项暂时为0。

每一路输出为：

```text
FF       = FF_STATIC + FF_KV × TARGET_RPM
ERROR    = TARGET_RPM - MEASURED_RPM
OUT      = clamp(FF + Kp×ERROR + I + D, 0, 900)
I[k]     = clamp(I[k-1] + Ki×ERROR×dt, -200, 120)
```

前馈负责给出维持目标速度所需的大部分基础输出，PI只修正模型误差、电池
电压和负载变化。前馈在PID内部参与总输出限幅和抗积分饱和，不是在PID
返回后简单相加。

## 2. DRV8871工作方式

TI数据手册真值表：

| IN1 | IN2 | 状态 |
|---:|---:|---|
| 0 | 0 | High-Z滑行，约1 ms后睡眠 |
| 0 | 1 | 反转 |
| 1 | 0 | 正转 |
| 1 | 1 | 低侧制动、慢衰减 |

20 kHz正转PWM在`10`驱动和`11`制动之间切换，符合TI推荐方式。

软件限制不能替代硬件限流。必须核对每块DRV8871的ILIM电阻：

```text
ITRIP(A) ≈ 64 / RILIM(kΩ)
```

例如32 kΩ约为2 A；数据手册要求RILIM不得低于15 kΩ。VM旁需要0.1 µF
陶瓷旁路和足够的储能电容，功率地、逻辑地与MCU必须可靠共地。

## 3. 板载PB21操作

核心板板载PB21低有效，使用内部上拉和30 ms软件消抖。当前固件中PB21
用于触发相对90度Yaw动作，不再循环切换共同速度档位；双轮目标RPM由
Yaw角度外环实时给出。

## 4. VOFA通道

选择JustFloat、921600 baud、10个通道：

| 通道 | 名称 |
|---:|---|
| 0 | M1_TARGET_RPM：有符号目标 |
| 1 | M1_ACTUAL_RPM：有符号实际转速 |
| 2 | M2_TARGET_RPM：有符号目标 |
| 3 | M2_ACTUAL_RPM：有符号实际转速 |
| 4 | YAW_TARGET |
| 5 | YAW_ACTUAL |
| 6 | YAW_ERROR：已处理±180度跳变 |
| 7 | YAW_TURN_RPM：角度外环输出 |
| 8 | GYRO_Z_DPS |
| 9 | YAW_STATE：0=空闲，1=执行，2=完成，负数=故障码 |

10通道每帧44字节，100 Hz约4.4 kB/s。

## 5. 首次上电顺序

1. 把车轮架空，电机电源先限流。
2. 下载固件，确认目标为0且电机不转。
3. 按一次PB21后，M1/M2目标应一正一负，实际RPM分别跟随相同符号。
4. 再按一次PB21可立即停止。
5. 方向正确后再落地测试90度Yaw动作。

## 6. 前馈标定

参数位于`inc/project_config.h`：

```c
CONFIG_SPEED_M1_FF_STATIC
CONFIG_SPEED_M1_FF_KV
CONFIG_SPEED_M2_FF_STATIC
CONFIG_SPEED_M2_FF_KV
```

当前架空实测拟合初值：

```text
M1_FF = 11.5 + 2.81 × TARGET_RPM
M2_FF = 14.7 + 2.63 × TARGET_RPM
```

更换电池、电机、减速箱、轮胎或整车落地后应重新标定：

1. 先用当前系数低速运行，确认方向和保护正常。
2. 分别设置50、100、150、200、250 RPM，每档至少保持4～5秒。
3. 记录稳定段的`OUT`，最好临时降低Ki或读取`FF+I`所需总基础输出。
4. 用两点快速计算时，可使用：

   ```text
   FF_KV     = (OUT_high - OUT_low) / (RPM_high - RPM_low)
   FF_STATIC = OUT_low - FF_KV × RPM_low
   ```

5. 所有档位都偏慢/积分长期为正：增大`FF_STATIC`。
6. 低速正常但高速偏慢：增大`FF_KV`。
7. 所有档位都偏快/积分长期为负：减小`FF_STATIC`。
8. 低速正常但高速偏快：减小`FF_KV`。

每次建议`FF_STATIC`调整5～10，`FF_KV`调整0.05～0.10。稳定后I修正越
接近0，说明前馈模型越准确。

设置`CONFIG_SPEED_FEEDFORWARD_ENABLE=0`可以关闭前馈做A/B比较，但关闭
后必须重新调纯PI参数。

## 7. PI反馈调参顺序

参数位于`inc/project_config.h`：

```c
CONFIG_SPEED_M1_KP / KI / KD
CONFIG_SPEED_M2_KP / KI / KD
```

前馈标定基本正确后再调反馈：

1. 保持`Kd=0`，把`Ki`暂时设为0。
2. 从`Kp=2.0`开始，每次增加0.5。
3. 让实际RPM能快速接近目标，但不要出现持续往复振荡。
4. 将Kp保留在振荡临界值的约50%～70%。
5. 从`Ki=4.0`开始，按4、5、6逐步增加，消除负载静差。
6. 若慢周期摆动或超调明显，先降低Ki。
7. 两个电机分别调整，不强求参数相同。
8. 当前13PPR编码器和100 ms测速窗口下保持`Kd=0`。

常见现象：

| 波形 | 调整 |
|---|---|
| 上升很慢且FF明显偏低 | 先重标定FF，再适当增大Kp |
| 稳态I长期为较大正值 | 前馈偏小，增大FF_STATIC或FF_KV |
| 稳态I长期为较大负值 | 前馈偏大，减小FF_STATIC或FF_KV |
| 前馈正确但仍有小静差 | 小幅增大Ki |
| 快速来回振荡 | 减小Kp或Ki |
| 大幅超调后慢慢回来 | 减小Ki、Kp或前馈 |
| 输出长期等于900 | 目标过高、电压不足、机械卡滞或负载过大 |
| RPM呈小台阶但总体稳定 | 编码器量化，属正常现象 |

## 8. 当前初始参数

当前为前馈启用后的保守反馈初值：

```text
M1 FF = 11.5 + 2.81 × TARGET_RPM
M2 FF = 14.7 + 2.63 × TARGET_RPM
M1/M2 Kp = 3.0 permille/RPM
M1/M2 Ki = 4.0 permille/(RPM·s)
M1/M2 Kd = 0
输出范围 = 0..900 permille
积分修正范围 = -200..120 permille
```

负积分用于在前馈偏大时向下修正，总输出下限仍是0，因此不会主动倒车。
这些参数只是安全起点，最终值取决于电池电压、电机、轮径、车重和机械阻力。

## 9. 低速堵转恢复与安全停机

最新堵转数据中，旧参数会在堵转期间把积分累积到`+250‰`。松手后正积分
仍持续推动车轮，实测峰值约170 RPM，并需要数秒才能回到100 RPM。现在
采用“积分分离 + 独立助推 + 超时停机”，助推量不会写入积分器：

```text
|ERROR| >= 30 RPM：暂停同方向积分，允许反方向误差卸载已有积分
启动后前300 ms：堵转检测宽限
|RPM| < 10且正误差>=30，连续50 ms：进入堵转助推
助推输出下限：600 permille
RPM >= 20：退出助推，立即恢复普通前馈+PI
助推持续1500 ms：锁存故障并停止双轮
```

可调宏均位于`inc/project_config.h`：

```c
CONFIG_SPEED_INTEGRAL_SEPARATION_RPM
CONFIG_SPEED_STALL_GUARD_MS
CONFIG_SPEED_STALL_DETECT_MS
CONFIG_SPEED_STALL_ENTER_RPM
CONFIG_SPEED_STALL_EXIT_RPM
CONFIG_SPEED_STALL_BOOST_OUTPUT
CONFIG_SPEED_STALL_TIMEOUT_MS
```

调试时先架空车轮并限制电源电流。短暂用手增加阻力即可观察恢复，不要长时间
完全夹死电机；助推超时后必须先排除机械卡滞，再按PB21重新启动。

数据手册：<https://www.ti.com/lit/ds/symlink/drv8871.pdf>
