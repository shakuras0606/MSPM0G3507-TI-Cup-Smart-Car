# MSPM0G3507 电赛车工程

工程位置：`D:\CCS_Workspace\ti_3507\empty`

开发环境：CCS 21.0、TI Clang 5.1.1 LTS、MSPM0 SDK 2.11.00.07、
SysConfig 1.26.2。当前版本 v0.6.0。

## 1. 实时任务方案

本工程是裸机超级循环，不是只能在主循环放屏幕代码。任务分工如下：

| 上下文 | 周期/触发 | 工作 |
|---|---:|---|
| 编码器 GPIO ISR | A/B 任一边沿 | x4 查表并累计计数 |
| SysTick ISR | 1 ms；每 10 ms 采样 | 保存两路累计计数和准确时间，不计算 RPM |
| UART1 ISR | 每 11 字节 DMA 完成 | WT61 DMA 数据复制入环形缓冲区并重启 DMA |
| UART0 ISR | 每帧 TX DMA 完成 | 释放 VOFA DMA 发送缓冲区 |
| 主循环 | 尽快 | WT61 解包、RPM 64 位除法、控制算法 |
| 主循环定时任务 | 10 ms | 两路独立车轮速度PID |
| 主循环定时任务 | 10 ms | VOFA JustFloat 发送 |
| 主循环低优先级任务 | 200 ms | ST7735 局部刷新 |

因此屏幕软件 SPI 即使占用了一段主循环时间，编码器 10 ms 快照仍由
SysTick 准时完成，WT61 仍由 DMA 接收。中断中不做浮点、除法、协议解析
或屏幕刷新。

## 2. 目录结构

```text
empty
├─ app
│  ├─ inc/app.h
│  ├─ inc/screen_task.h
│  ├─ inc/speed_control.h
│  ├─ src/app.c              协作式任务调度、VOFA 数据组织
│  ├─ src/screen_task.c      ST7735 低频局部刷新
│  └─ src/speed_control.c    双轮独立定速PID
├─ board                     板级初始化和人工可读引脚表
├─ bsp
│  ├─ bsp_time.*             1 ms SysTick
│  └─ bsp_uart.*             UART0 上位机 TX DMA
├─ components
│  ├─ byte_ring.*            ISR/主循环环形缓冲
│  ├─ comm_protocol.*        保留的 AA55 协议组件
│  └─ pid.*                  通用 PID
├─ drivers
│  ├─ encoder.*              双编码器 x4 解码和车轮 RPM
│  ├─ wt61.*                 UART1 RX DMA 和 WIT 11 字节协议
│  ├─ st7735.*               128×160 软件 SPI 显示
│  ├─ vofa.*                 JustFloat DMA 输出
│  ├─ drv8871.*              双电机驱动
│  ├─ line_sensor16.*        16 路巡线数据层
│  └─ bmi270_port.*          暂停使用的 BMI270 端口层
├─ inc/project_config.h      采样、减速比、刷新周期等集中配置
├─ empty.syscfg              外设、DMA 和引脚配置的唯一入口
└─ Debug/empty.out           已验证的 Debug 固件
```

## 3. 当前引脚

| 模块 | 信号 | MSPM0G3507 | 参数 |
|---|---|---:|---|
| ST7735 | MOSI/SCLK | PB8/PB9 | 软件 SPI |
| ST7735 | RST/DC/CS/BL | PB10/PB11/PB14/PB26 | GPIO |
| DRV8871 M1 | IN1/IN2 | PA26/PA27 | TIMG7，20 kHz |
| DRV8871 M2 | BIN1/BIN2 | PB15/PB16 | TIMG8，20 kHz |
| 编码器 M1 | A/B | PB0/PB1 | GPIO 双边沿中断 |
| 编码器 M2 | A/B | PB2/PB3 | GPIO 双边沿中断 |
| CH343/VOFA | UART0 TX/RX | PA10/PA11 | 921600，TX DMA_CH0 |
| WT61TTL | UART1 TX/RX | PA8/PA9 | 115200，RX DMA_CH1 |
| 板载速度按键 | GPIO | PB21 | 低有效、内部上拉、30 ms消抖 |
| 蜂鸣器 | GPIO | PA7 | 预留 |
| SWD | SWDIO/SWCLK | PA19/PA20 | 调试固定 |
| 16 路巡线 | transport | 未分配 | 等模块协议确定 |

WT61 接线方向：

```text
WT61 TX  -> PA9  (MCU UART1_RX)
WT61 RX  -> PA8  (MCU UART1_TX，当前仅为以后配置保留)
WT61 GND -> MCU GND
WT61 VCC -> 按模块丝印/手册供电
```

PA8/PA9 和 PB6/PB7 都是同一个 UART1 外设的不同复用位置，不能当成两路
UART 同时使用。当前 UART1 给 WT61，原 PB6/PB7 蓝牙通信已停用。以后若要
WT61 与蓝牙同时在线，蓝牙必须改接 UART2/UART3 的可用引脚并增加另一条
DMA 通道。

## 4. 编码器和车轮 RPM

当前实物参数：

```c
#define CONFIG_ENCODER_AB_CYCLES_PER_MOTOR_REV       (13u)
#define CONFIG_ENCODER_DECODE_MULTIPLIER              (4u)
#define CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_NUM   (28u)
#define CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_NUM   (28u)
#define CONFIG_ENCODER_UPDATE_PERIOD_MS              (10u)
#define CONFIG_ENCODER_MEASUREMENT_WINDOW_MS        (100u)
```

车轮一圈理论计数为 `13 × 4 × 28 = 1456`。SysTick 每 10 ms 保存一个
准确快照，主循环用最近 100 ms 两个端点计算：

```text
wheel_RPM =
    delta_count × 60000 × ratio_den
    ---------------------------------------------
    motor_counts_per_rev × ratio_num × elapsed_ms
```

`Encoder_Update()` 可以放进定时中断，但不建议：其中有 64 位除法，会
增加中断延迟。当前方案只把“采样”放进 SysTick，把“运算”留在主循环，
同时获得准确采样周期和较短 ISR。

## 5. 双轮定速PID

核心板板载PB21每次确认按下后按以下顺序切换共同目标：

```text
0 → 50 → 100 → 150 → 200 → 250 → 300 → 0 RPM
```

两轮使用独立PID实例，控制周期10 ms。初始参数为：

```c
Kp = 2.0f;
Ki = 8.0f;
Kd = 0.0f;
```

PID输出限制为0～900千分比。目标为0时，DRV8871两个输入均为低，
H桥High-Z滑行并进入低功耗睡眠。运行PWM采用手册推荐的驱动/制动慢衰减。
详细调试步骤见`定速PID调试说明.md`。

## 6. WT61 和 VOFA

WT61 标准帧固定为 11 字节：

```text
0x55 TYPE DATA0...DATA7 CHECKSUM
```

- `TYPE=0x52`：X/Y/Z 角速度，换算系数 `2000/32768 deg/s`；
- `TYPE=0x53`：Roll/Pitch/Yaw，换算系数 `180/32768 deg`；
- 校验为前 10 字节累加和的低 8 位。

VOFA+ 设置：

- 串口：板载 CH343 对应的 COM 口；
- 波特率：`921600`；
- 数据引擎：`JustFloat`；
- 发送周期：10 ms；
- 通道数：11。

| VOFA 通道 | 数据 |
|---:|---|
| ch0 | 共同目标RPM |
| ch1 | M1车轮RPM |
| ch2 | M2车轮RPM |
| ch3 | M1 PWM千分比输出 |
| ch4 | M2 PWM千分比输出 |
| ch5 | M1 RPM误差 |
| ch6 | M2 RPM误差 |
| ch7 | 方向故障掩码：0正常，1=M1，2=M2，3=两路 |
| ch8 | Yaw，degree |
| ch9 | Pitch，degree |
| ch10 | Roll，degree |

如果后三路始终为 0：

1. 确认交叉接线和共地；
2. 确认 WT61 当前波特率与工程一致，当前均为115200；
3. 确认模块已开启 `0x53` 角度帧输出；
4. 在调试器观察 `WT61Snapshot.angle_frames` 和 `checksum_errors`。

## 7. CH343 波特率

CH343 芯片支持远高于 921600 的波特率；工程先使用 921600，兼顾 Windows
驱动、VOFA 和 MCU 分频误差。10个float、100 Hz的负载仍远低于链路能力，
115200 也够用，提升到 921600 的目的是给后续调试和更多通道留余量。

如果 Windows 不能稳定打开 921600，请安装 WCH 官方 CH343 VCP 驱动，
然后确认设备管理器中的芯片型号确实是 CH343，而不是 CH340。

## 8. 构建

1. CCS 中只导入 `D:\CCS_Workspace\ti_3507\empty`。
2. 修改 `empty.syscfg` 后保存。
3. 执行 **Project → Clean Project**。
4. 执行 **Build Project**。
5. 下载 `Debug/empty.out`。

命令行构建已验证为 0 error、0 warning。若新增的 `screen_task.c` 或
`wt61.c` 在 Project Explorer 未立即显示，右键工程选择 **Refresh**，
再 Clean/Build。

## 9. 参考资料

- TI MSPM0G3507 数据手册：<https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf>
- TI DRV8871 数据手册：<https://www.ti.com/lit/ds/symlink/drv8871.pdf>
- WIT 标准通信协议：<https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol>
- ST7735S 控制器手册：<https://www.displayfuture.com/Display/datasheet/controller/ST7735S.pdf>
- WCH CH343 驱动：<https://www.wch.cn/downloads/CH343SER_ZIP.html>
