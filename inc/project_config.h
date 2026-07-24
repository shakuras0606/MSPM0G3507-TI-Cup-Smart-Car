/**
 * @file    project_config.h
 * @brief   TI3507-CAR 项目全局配置（编译时常量），v0.6.1 双轮定速调试版
 *
 * 本文件集中管理所有应用层可调参数。物理引脚不在此定义，
 * 引脚分配请使用 empty.syscfg 修改，并在 board/inc/board_pins.h 中保持同步。
 *
 * 调参规则：
 *   1. 所有带时间含义的宏均在名称中写明单位，例如 _MS、_S；
 *   2. PWM 命令统一使用千分比，1000 表示 100%；
 *   3. 转速统一表示车轮机械 RPM，而不是电机转子 RPM；
 *   4. 一次只修改一类参数，并保存 VOFA 调参曲线，避免无法判断效果来源；
 *   5. 电机方向、编码器方向与 PID 增益是三件不同的事，不要用 PID 参数
 *      修正接线或方向错误；
 *   6. 修改宏后必须重新 Build 并重新下载 empty.out，运行中的 MCU 不会
 *      自动读取头文件新值。
 *
 * @author  TI3507 Team
 * @version 0.6.1
 */

#ifndef PROJECT_CONFIG_H_
#define PROJECT_CONFIG_H_

/* ========================================================================
 *  项目标识
 * ======================================================================== */

/** 工程名称，仅用于版本信息和调试标识，不参与控制计算。 */
#define PROJECT_NAME                         "TI3507-CAR"

/** 固件版本；修改控制逻辑或默认参数后应同步递增，便于区分烧录版本。 */
#define PROJECT_VERSION                      "0.6.1"

/* ========================================================================
 *  通信缓冲区
 * ======================================================================== */

/**
 * UART0 TX DMA 临时缓冲区，单位：字节。
 * 必须能容纳一帧最大的 VOFA 或通信数据。减小可节省 SRAM，但过小会导致
 * 发送接口拒绝长帧；增大只增加内存占用，不会提高串口波特率。
 */
#define CONFIG_HOST_UART_TX_DMA_BUFFER_SIZE  (128u)

/**
 * WT61 UART1 接收环形缓冲区，单位：字节。
 * 115200 baud 下约每毫秒收到 11.5 字节；256 字节可缓存约 22 ms 数据。
 * 如果主循环被长时间阻塞并出现丢帧，可增大；正常情况下不靠无限增大
 * 缓冲区掩盖阻塞任务。
 */
#define CONFIG_WT61_RX_RING_SIZE             (256u)

/** 自定义 AA55 通信协议允许的最大有效载荷，单位：字节。 */
#define CONFIG_COMM_MAX_PAYLOAD              (64u)

/**
 * AA55 发送帧工作缓冲区，单位：字节。
 * 当前 72 = 64 字节载荷 + 帧头、长度、命令和校验余量；必须不小于协议
 * 完整帧的最大长度。
 */
#define CONFIG_COMM_TX_BUFFER_SIZE           (72u)

/* ========================================================================
 *  电机驱动 (DRV8871 × 2, v0.2.0 双电机)
 * ======================================================================== */

/**
 * PWM 周期计数值，单位：定时器时钟计数。
 *
 * 公式：
 *   PWM频率 = 定时器时钟 / CONFIG_MOTOR_PWM_PERIOD_COUNTS
 * 当前 TIMG7/TIMG8 时钟均为 32 MHz，因此 32 MHz / 1600 = 20 kHz。
 *
 * 该值必须与 empty.syscfg 中 MOTOR_M1_PWM、MOTOR_M2_PWM 的 timerCount
 * 同时修改。只改本宏会导致占空比换算与硬件周期不一致。
 *
 * 调试影响：
 *   - 计数增大：PWM 频率降低、占空比分辨率提高，可能进入可听频段；
 *   - 计数减小：PWM 频率升高、开关损耗增加、占空比分辨率降低；
 *   - DRV8871 推荐工作范围内 20 kHz 可避开大部分人耳听觉范围。
 */
#define CONFIG_MOTOR_PWM_PERIOD_COUNTS       (1600u)

/**
 * 电机指令满量程，单位：千分比。
 * 公共驱动接口范围为 [-1000,+1000]：
 *   +1000 = 一个方向 100%驱动，-1000 = 反方向100%驱动，0 = 停止。
 * 此宏同时作为换算分母，不建议改成其他值；若只想限制最大输出，应调整
 * CONFIG_SPEED_OUTPUT_MAX。
 */
#define CONFIG_MOTOR_COMMAND_MAX             (1000)

/**
 * DRV8871 PWM 衰减模式选择，布尔量：0 或 1。
 *
 *   1 = Drive/Brake 慢衰减：
 *       正转在 IN1/IN2=10（驱动）和11（制动）之间切换；
 *       低速转矩和速度保持通常更好，但制动更强、电流纹波方式不同。
 *
 *   0 = Drive/Coast 快衰减：
 *       正转在10（驱动）和00（高阻滑行）之间切换；
 *       滑行感更强，低占空比下可能更容易停转。
 *
 * 当前定速环使用 1。改变该宏后同一组 PID 参数的响应也会改变，需要重新
 * 调整 Kp/Ki，不能直接沿用原曲线结论。
 */
#define CONFIG_MOTOR_PWM_BRAKE_DECAY         (1u)

/* ========================================================================
 *  双轮定速 PID
 * ======================================================================== */

/**
 * 双轮速度环执行周期，单位：ms。
 * 当前 10 ms = 100 Hz。SpeedControl_Update() 每次主循环都会被调用，但
 * 只有累计时间达到该值才真正执行一次 PID。
 *
 * 调小：响应更快、CPU 占用略增，Ki/Kd 的离散效果和噪声敏感度会上升。
 * 调大：响应变慢，容易出现明显阶梯控制。
 * 当前编码器每10 ms产生新快照，因此建议保持10 ms；若修改，必须重新
 * 检查 CONFIG_ENCODER_UPDATE_PERIOD_MS 和 PID 参数。
 */
#define CONFIG_SPEED_CONTROL_PERIOD_MS       (10u)

/**
 * PB21 每次有效按下增加的目标车轮转速，单位：RPM。
 * 当前按键序列为 0→50→100→...→300→0。
 * 减小步长便于低速细调；增大步长会使目标阶跃和电流冲击更明显。
 */
#define CONFIG_SPEED_BUTTON_STEP_RPM         (50.0f)

/**
 * 按键允许设置的最大车轮目标转速，单位：RPM。
 * 超过该值时下一次按键回到 0。它不是机械安全的唯一保障；实际安全上限
 * 还取决于电机电压、轮径、底盘和场地。首次架空测试建议临时设为100。
 */
#define CONFIG_SPEED_TARGET_MAX_RPM          (300.0f)

/**
 * PB21 软件消抖时间，单位：ms。
 * 太小会把一次机械按压识别为多次；太大会增加按键响应延迟。
 * 常见机械按键可在 20~50 ms 调整，当前 30 ms。
 */
#define CONFIG_SPEED_BUTTON_DEBOUNCE_MS      (30u)

/**
 * 两个车轮使用独立PID状态，参数初值相同但可以分别调整。
 *
 * 由于编码器使用100 ms滑动窗口，微分项容易放大量化噪声，初调采用PI：
 *   output = Kp * RPM误差 + Ki * RPM误差积分。
 * 输出单位为DRV8871千分比命令。
 *
 * 调参顺序：
 *   1. 先令 Ki=0、Kd=0，只增加 Kp；
 *   2. 找到响应较快但不持续振荡的 Kp；
 *   3. 再逐渐增加 Ki 消除稳态误差；
 *   4. 13PPR 编码器量化明显，通常保持 Kd=0。
 */
/** M1比例增益：增大可加快响应，过大会造成振荡、噪声和电流冲击。 */
#define CONFIG_SPEED_M1_KP                   (2.0f)

/** M1积分增益，单位约为 输出千分比/(RPM*s)；过大易超调和低频摆动。 */
#define CONFIG_SPEED_M1_KI                   (8.0f)

/** M1微分增益；当前为0，低分辨率编码器不建议在初调阶段开启。 */
#define CONFIG_SPEED_M1_KD                   (0.0f)

/** M2比例增益；应根据M2实际摩擦、负载分别调整，不要求与M1相同。 */
#define CONFIG_SPEED_M2_KP                   (2.0f)

/** M2积分增益；若M2长期低于目标可小幅增加，若慢速摆动则减小。 */
#define CONFIG_SPEED_M2_KI                   (8.0f)

/** M2微分增益；当前为0，若以后启用必须同时观察测速锯齿和滤波效果。 */
#define CONFIG_SPEED_M2_KD                   (0.0f)

/**
 * PID 输出下限，单位：千分比。
 * 当前只允许正目标速度，设为0可防止速度超过目标时控制器主动反转电机。
 * 不要用它设置启动占空比，启动补偿由 CONFIG_SPEED_STARTUP_MIN_OUTPUT 管理。
 */
#define CONFIG_SPEED_OUTPUT_MIN              (0.0f)

/**
 * PID 输出上限，单位：千分比。
 * 900 表示最多90%命令，保留10%余量。降低可限制加速度和最大驱动力；
 * 过低会导致满输出仍达不到目标。它不能代替 DRV8871 的 RILIM 硬件限流。
 */
#define CONFIG_SPEED_OUTPUT_MAX              (900.0f)

/**
 * 积分项下限，单位：输出千分比。
 * 正向单向速度环设为0，避免积分累积成负驱动。若以后实现双向速度控制，
 * 需要重新设计输出/积分范围，而不是只把该值改成负数。
 */
#define CONFIG_SPEED_INTEGRAL_MIN            (0.0f)

/**
 * 积分项上限，单位：输出千分比。
 * 降低可减小长时间堵转后的超调；过低会无法消除大负载下的稳态误差。
 * 当前800，小于总输出上限900，为比例项保留一定调节余量。
 */
#define CONFIG_SPEED_INTEGRAL_MAX            (800.0f)

/**
 * 微分一阶低通滤波时间常数，单位：s。
 * 数值越大，滤波越强、微分响应越慢；数值越小，响应越快但噪声越大。
 * 当前 Kd=0 时不影响最终输出，但保留0.03 s供以后调试。
 */
#define CONFIG_SPEED_D_FILTER_TAU_S          (0.03f)

/**
 * 50 RPM 时仅靠 Kp 的首拍输出为 100‰，通常不足以克服 N20 减速电机
 * 的静摩擦。车轮尚未达到阈值时，将实际命令至少提升到 250‰。
 * 这只是低速启动补偿，不改变 PID 积分状态。
 *
 * 调试方法：
 *   - 电机只响/抖但不起转：每次增加 20~50，直到可靠启动；
 *   - 启动冲击过大：逐步减小，但必须保证两个轮子都能在负载下启动；
 *   - 该值单位是千分比，250表示25%等效驱动；
 *   - 不得超过 CONFIG_SPEED_OUTPUT_MAX。
 */
#define CONFIG_SPEED_STARTUP_MIN_OUTPUT       (250.0f)

/**
 * 启动补偿生效的速度区间阈值，单位：车轮 RPM。
 * 当测量速度在 (-10,+10) RPM 且 PID 输出大于0时，实际命令才会被抬高
 * 到启动最小值。增大可能导致低速目标长期使用较大占空比；减小可能在刚
 * 起转时过早撤掉补偿而再次停转。
 */
#define CONFIG_SPEED_STARTUP_RPM_THRESHOLD    (10.0f)

/**
 * 方向保护采用“启动宽限 + 连续样本确认”：
 *   - 启动后前 300 ms 不判断，等待 100 ms 测速窗口填满；
 *   - 此后连续 5 个控制周期测得反向才停机。
 * 避免启动抖动或单个编码器毛刺把两个电机立即关闭。
 */
/**
 * 判定“明显反向”的负转速阈值，单位：RPM。
 * 只有 rpm < -5 才累计反向样本。设得太小会被零速量化噪声误触发；
 * 设得太大则真正反向时需要更高转速才被发现。
 */
#define CONFIG_SPEED_DIRECTION_FAULT_RPM     (5.0f)

/**
 * 启动后的方向检测宽限时间，单位：ms。
 * 必须覆盖 DRV8871 唤醒、电机起转和编码器100 ms窗口填充时间。
 * 当前300 ms。太短易在启动瞬间误判，太长会让错误方向运行更久。
 */
#define CONFIG_SPEED_DIRECTION_GUARD_MS      (300u)

/**
 * 方向错误必须连续出现的控制周期数，单位：次。
 * 当前5次，在10 ms速度环下相当于约50 ms连续确认。
 * 增大可抗毛刺但停机更慢；减小保护更快但更容易误触发。
 */
#define CONFIG_SPEED_DIRECTION_FAULT_SAMPLES (5u)

/**
 * 车体“目标RPM为正”时的电机电气命令符号，只允许 +1 或 -1。
 *
 * 修改判断：
 *   - 车轮物理旋转方向错误：修改 COMMAND_SIGN；
 *   - 车轮物理方向正确但VOFA RPM为负：修改 ENCODER_INVERT_DIRECTION；
 *   - 不要同时修改两者，否则可能看似RPM变正但车仍向错误方向运动。
 *
 * 当前实物两路均需要 -1，表示 PID 的正输出经过方向映射后下发负的
 * DRV8871 命令。
 */
#define CONFIG_SPEED_M1_COMMAND_SIGN         (-1)

/** M2电机命令方向，解释和调试规则与M1相同；当前实物为-1。 */
#define CONFIG_SPEED_M2_COMMAND_SIGN         (-1)

/* ========================================================================
 *  编码器 (N20 霍尔编码器 × 2)
 * ======================================================================== */

/**
 * 编码器 A/B 每个完整正交周期数，常见带霍尔 N20 标称为
 * 电机轴每圈 13 个 A/B 周期。
 *
 * 当前驱动同时统计 A/B 的上升沿和下降沿，是 x4 解码，所以：
 *   电机轴每圈计数 = 13 * 4 = 52
 *
 * 本工程输出的是“车轮 RPM”。编码器位于电机转子轴，因此还必须填写
 * 电机转子到车轮的实际总减速比：
 *
 *   减速比 = 电机转子转数 / 车轮转数
 *
 * 例如 30:1 应将 RATIO_NUM 设为 30、RATIO_DEN 设为 1。
 * 如果实际减速比为 29.86:1，可填写 NUM=2986、DEN=100，驱动在 RPM
 * 公式中直接使用这个分数，不会先取整成每圈计数。
 *
 * 当前两路均按实物参数设置为 28:1，因此每个车轮机械圈对应：
 *   13 * 4 * 28 = 1456 个 x4 计数。
 */
/**
 * 编码器每个电机轴机械圈的 A/B 完整正交周期数。
 * 用户给出的编码器精度为13 PP，因此填13。这里不是x4后的计数值；
 * 若误填52，最终RPM会变成真实值的1/4。
 */
#define CONFIG_ENCODER_AB_CYCLES_PER_MOTOR_REV (13u)

/**
 * 正交解码倍频系数。
 * 当前A/B两相上升沿和下降沿全部计数，因此为4。该值必须与encoder.c的
 * 实际中断解码方式一致，不是可随意用于“校准RPM”的比例系数。
 */
#define CONFIG_ENCODER_DECODE_MULTIPLIER        (4u)

/** 电机轴每圈软件计数值：13×4=52，由上面两个物理/算法参数自动计算。 */
#define CONFIG_ENCODER_COUNTS_PER_MOTOR_REV \
    (CONFIG_ENCODER_AB_CYCLES_PER_MOTOR_REV * \
     CONFIG_ENCODER_DECODE_MULTIPLIER)

/**
 * M1减速比的分子：电机转子转数/车轮转数。
 * 28:1减速箱填写NUM=28、DEN=1；增大NUM会让计算RPM减小。
 */
#define CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_NUM (28u)

/** M1减速比分母；整数减速比通常为1，小数比可用分数精确表示。 */
#define CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_DEN (1u)

/** M2减速比分子，当前28；解释与M1相同，但允许左右轮使用不同减速箱。 */
#define CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_NUM (28u)

/** M2减速比分母，当前1。分子和分母都必须大于0。 */
#define CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_DEN (1u)

/**
 * 编码器反馈符号修正，布尔量：0=保持解码符号，1=整体取反。
 *
 * 只有“车轮物理前进方向正确，但显示RPM为负”时才修改该项。
 * 若车轮物理方向本身错误，应改 CONFIG_SPEED_Mx_COMMAND_SIGN。
 * 不要交换通用正交状态表；也不要把减速比写成负数。
 */
/** M1编码器方向取反开关；当前实物保持原始方向。 */
#define CONFIG_ENCODER_M1_INVERT_DIRECTION     (0u)

/** M2编码器方向取反开关；当前实物保持原始方向。 */
#define CONFIG_ENCODER_M2_INVERT_DIRECTION     (0u)

/**
 * 车轮 RPM 每 10 ms 更新一次，即 100 Hz。
 *
 * 13PPR 编码器四倍频后只有 52 count/电机轴圈。如果直接使用单个
 * 10 ms 区间计算，量化跳动会非常明显。因此驱动每 10 ms 保存一个
 * 计数快照，但使用最近 100 ms 的滑动窗口计算 RPM：
 *
 *   - UPDATE_PERIOD_MS 决定输出刷新速度；
 *   - MEASUREMENT_WINDOW_MS 决定计数分辨率和平均时间。
 *
 * 每次计算都使用窗口的真实经过时间，主循环偶尔延迟不会造成比例误差。
 */
/**
 * 编码器累计计数快照周期，单位：ms。
 * 该快照由1 ms系统节拍触发，10 ms对应100 Hz。调小会增加历史数组和
 * 中断写入频率；调大则RPM输出更新更慢。必须能整除测量窗口。
 */
#define CONFIG_ENCODER_UPDATE_PERIOD_MS       (10u)

/**
 * RPM滑动测量窗口，单位：ms。
 * 计算公式：
 *   wheel_rpm = Δcount × 60000 × ratio_den
 *               / (52 × ratio_num × elapsed_ms)
 *
 * 增大窗口：曲线更平滑、低速分辨率更高，但真实变化延迟更大；
 * 减小窗口：响应更快，但13PPR编码器的锯齿更严重。
 * 当前100 ms是低速稳定性与响应速度的折中。
 */
#define CONFIG_ENCODER_MEASUREMENT_WINDOW_MS (100u)

/* ========================================================================
 *  BMI270 六轴 IMU
 * ======================================================================== */

/*
 * PA8/PA9 当前分配给 WT61TTL 的 UART1，因此原 BMI270 软件 SPI 暂停使用。
 * 后续重新分配 BMI270 引脚后可置 1，并同步修改 empty.syscfg。
 */
/**
 * BMI270功能总开关：0=不初始化，1=启用。
 * 当前必须保持0。PB21正作为板载按键使用，而扩展板又把PB21标为
 * IMU_CS_A；没有完成引脚重分配前直接置1会产生资源冲突。
 */
#define CONFIG_BMI270_ENABLED                (0u)

/**
 * BMI270软件SPI每个半周期的CPU延时计数。
 * 数值越大SPI越慢、时序裕量越大；数值越小越快但可能违反建立/保持时间。
 * 仅在BMI270启用且仍使用软件SPI时生效，不能直接理解为微秒。
 */
#define CONFIG_BMI270_SPI_HALF_CYCLES        (4u)

/* ========================================================================
 *  WT61TTL 姿态传感器
 * ======================================================================== */

/**
 * WT61标准二进制协议固定帧长，单位：字节。
 * 协议格式为 0x55 + TYPE + DATA[8] + CHECKSUM，共11字节。
 * 这是协议常量，不应为了修复丢帧而修改。
 */
#define CONFIG_WT61_FRAME_SIZE               (11u)

/**
 * WT61姿态数据离线超时，单位：ms。
 * 距离最后一帧有效角度数据超过500 ms后，屏幕将传感器标为离线。
 * 减小可更快发现断线，过小会在低回传率时误判；应明显大于模块回传周期。
 */
#define CONFIG_WT61_STALE_TIMEOUT_MS         (500u)

/* ========================================================================
 *  16 通道线传感器
 * ======================================================================== */

/**
 * 16路巡线数字量默认有效电平：1=低有效，0=高有效。
 * 最终必须与巡线模块输出极性一致。极性错误会让“黑线/白底”全部反转，
 * 但不会改变硬件引脚电平。
 */
#define CONFIG_LINE_SENSOR_DEFAULT_ACTIVE_LOW (1u)

/* ========================================================================
 *  遥测
 * ======================================================================== */

/**
 * ST7735界面刷新周期，单位：ms。
 * 200 ms = 5 Hz，足够查看数值且不会让软件SPI长期占用主循环。
 * 减小可让显示更流畅，但会增加CPU占用并推迟协议解析；PID本身不依赖屏幕。
 */
#define CONFIG_SCREEN_PERIOD_MS              (200u)

/* ========================================================================
 *  VOFA+ JustFloat 数据可视化 (v0.2.0)
 * ======================================================================== */

/**
 * VOFA发送周期，单位：ms。
 * 10 ms = 100 Hz，与速度环更新一致，便于一个控制周期对应一个波形点。
 * 调得过小不会创造新的编码器信息，只会增加串口和DMA负担；调大则波形
 * 数据更稀疏。
 */
#define CONFIG_VOFA_PERIOD_MS                (10u)

/**
 * VOFA输出端口枚举。
 * 当前使用板载Type-C/CH343对应UART0，发送由DMA完成。若改到其他串口，
 * 必须确认该串口已初始化、波特率正确且不会与WT61/蓝牙混用。
 */
#define CONFIG_VOFA_PORT                     BSP_UART_HOST

/**
 * 定速PID调试通道：
 * 0=目标RPM，1=M1 RPM，2=M2 RPM，3=M1输出，4=M2输出，
 * 5=M1误差，6=M2误差，7=方向故障掩码，8=Yaw，9=Pitch，10=Roll。
 *
 * 通道数必须与 app.c 中 Vofa_PutFloat() 的调用次数完全一致。少于实际
 * 数量会导致帧缓冲或解析错误，多于实际数量会让上位机曲线错位。
 */
#define CONFIG_VOFA_CHANNELS                 (11u)

/**
 * 编码器测试模式下，UART0 只允许输出 VOFA JustFloat。
 *
 * 置 1 时：
 *   - UART0 不发送 ASCII 启动文字；
 *   - UART0 不发送 AA55 遥测；
 *   - UART0 不解析命令；
 *   - 蓝牙 UART1 仍保留 AA55 命令和遥测请求。
 *
 * 这样可以防止三种数据格式混入同一串口导致 VOFA 曲线错帧。
 * 调试JustFloat时保持1；以后改为自定义协议控制时可置0，但必须同步调整
 * 上位机解析方式。
 */
#define CONFIG_HOST_UART_EXCLUSIVE_VOFA      (1u)

/*
 * 配置合法性检查：
 * 这些错误若拖到运行期才暴露，通常只会表现为转速恒为 0、除零或采样异常，
 * 因此在编译阶段直接给出清晰提示。
 */
#if (CONFIG_ENCODER_UPDATE_PERIOD_MS == 0u)
#error "CONFIG_ENCODER_UPDATE_PERIOD_MS must be greater than zero."
#endif

#if (CONFIG_ENCODER_MEASUREMENT_WINDOW_MS < CONFIG_ENCODER_UPDATE_PERIOD_MS)
#error "Encoder measurement window must not be shorter than update period."
#endif

#if ((CONFIG_ENCODER_MEASUREMENT_WINDOW_MS % \
      CONFIG_ENCODER_UPDATE_PERIOD_MS) != 0u)
#error "Encoder measurement window must be an integer multiple of update period."
#endif

#if (CONFIG_ENCODER_COUNTS_PER_MOTOR_REV == 0u)
#error "Encoder motor-shaft counts per revolution must be greater than zero."
#endif

#if (CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_NUM == 0u) || \
    (CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_DEN == 0u) || \
    (CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_NUM == 0u) || \
    (CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_DEN == 0u)
#error "Encoder motor-to-wheel ratio numerator and denominator must be non-zero."
#endif

#endif /* PROJECT_CONFIG_H_ */
