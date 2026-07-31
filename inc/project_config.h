/**
 * @file    project_config.h
 * @brief   TI3507-CAR 全局可调参数
 *
 * 物理引脚在 empty.syscfg 中配置。本文件只保存应用参数、方向极性和
 * 编译期合法性检查。PWM 使用千分比，速度统一使用车轮 RPM。
 *
 * 调参时一次只改一组参数并保存同工况VOFA数据。方向SIGN是实物极性，
 * 不是PID增益；闭环方向已正确后不要再用SIGN修正振荡或响应速度。
 */

#ifndef PROJECT_CONFIG_H_
#define PROJECT_CONFIG_H_

/* 项目标识 */
#define PROJECT_NAME                              "TI3507-CAR" /* 工程名称 */
#define PROJECT_VERSION                           "0.10.0"     /* 加入WT61加速度与经典CAN前馈遥测 */

/*
 * 系统任务与三级 PID 执行频率
 *
 * 所有周期执行项统一在这里以 Hz 配置，便于直接比较各环带宽。通常应
 * 保持“速度内环 >= Yaw中环 >= 巡线外环”，改频率后需重新检查PID参数。
 * 消抖、超时、测量窗口等“持续时间”仍在对应模块中使用 ms。
 */
#define CONFIG_PID_SPEED_HZ                       (200u)  /* 速度内环更新率/Hz；使用最新RPM快照 */
#define CONFIG_PID_YAW_HZ                         (100u)  /* Yaw中环更新率/Hz；建议不高于WT61有效帧率 */
#define CONFIG_PID_LINE_HZ                        (100u)  /* 巡线外环更新率/Hz；过高会放大二值位置跳变 */
#define CONFIG_TASK_ENCODER_CAPTURE_HZ            (CONFIG_PID_SPEED_HZ) /* 编码器快照率/Hz；与速度环同步 */
#define CONFIG_TASK_LINE_SENSOR_SCAN_HZ           (200u)  /* 8路原始模拟帧请求/处理率/Hz */
#define CONFIG_TASK_CAN_TX_HZ                     (100u)  /* WT61加速度CAN发送率/Hz */
#define CONFIG_TASK_VOFA_HZ                       (100u)  /* JustFloat遥测帧率/Hz */
#define CONFIG_TASK_SCREEN_HZ                     (5u)    /* ST7735刷新率/Hz；低优先级任务 */

/* 1 ms固定时基；频率必须能整除1000 Hz。此换算宏不作为调参项。 */
#define CONFIG_SCHEDULER_TICK_HZ                  (1000u)
#define CONFIG_TICKS_FROM_HZ(frequency_hz)        \
    (CONFIG_SCHEDULER_TICK_HZ / (frequency_hz))

/* 通信缓冲区 */
#define CONFIG_WT61_RX_RING_SIZE                  (256u)  /* WT61 RX 环形缓冲区/字节 */
#define CONFIG_COMM_MAX_PAYLOAD                   (64u)   /* AA55 最大载荷/字节 */
#define CONFIG_COMM_TX_BUFFER_SIZE                (72u)   /* AA55 发送缓冲区/字节 */

/*
 * 车体板 -> 钢珠控制板：经典CAN、11位标准ID、8字节。
 * empty.syscfg固定为500 kbps；两端波特率必须一致。
 * 0x180当前为三轴加速度，0x181~0x187保留车体扩展，0x188~0x18F保留C板回传。
 */
#define CONFIG_CAN_NOMINAL_BIT_RATE               (500000u) /* 文档镜像值；实际位时序由SysConfig生成 */
#define CONFIG_CAN_ID_ACCEL                       (0x180u)   /* 三轴加速度快速帧 */
#define CONFIG_CAN_PROTOCOL_VERSION               (1u)       /* 状态字节bit7..5，取值0..7 */
#define CONFIG_CAN_ACCEL_STALE_TIMEOUT_MS          (50u)      /* 超时后清ACCEL_FRESH，C板应关闭前馈 */

/* DRV8871：32 MHz / 1600 = 20 kHz */
#define CONFIG_MOTOR_PWM_PERIOD_COUNTS            (1600u) /* PWM 周期计数，须与 SysConfig 一致 */
#define CONFIG_MOTOR_COMMAND_MAX                  (1000)  /* 电机命令满量程：±1000 */
#define CONFIG_MOTOR_PWM_BRAKE_DECAY              (1u)    /* 1=Drive/Brake，0=Drive/Coast */

/* 速度按键 */
#define CONFIG_SPEED_BUTTON_STEP_RPM              (50.0f) /* PB21 每次增加的目标 RPM */
#define CONFIG_SPEED_TARGET_MAX_RPM               (300.0f)/* PB21 可设最大车轮 RPM */
#define CONFIG_SPEED_BUTTON_DEBOUNCE_MS           (30u)   /* PB21 消抖时间/ms */

/* 前馈：FF = STATIC + KV × target_rpm */
#define CONFIG_SPEED_FEEDFORWARD_ENABLE           (1u)    /* 1=启用前馈，0=纯 PID */
#define CONFIG_SPEED_M1_FF_STATIC                 (11.5f) /* M1 静态前馈/千分比 */
#define CONFIG_SPEED_M1_FF_KV                     (2.81f) /* M1 前馈斜率/千分比每 RPM */
#define CONFIG_SPEED_M2_FF_STATIC                 (14.7f) /* M2 静态前馈/千分比 */
#define CONFIG_SPEED_M2_FF_KV                     (2.63f) /* M2 前馈斜率/千分比每 RPM */

/* 双轮独立 PID */
#define CONFIG_SPEED_M1_KP                        (3.6f)   /* M1 比例增益：提高动态跟随 */
#define CONFIG_SPEED_M1_KI                        (4.0f)   /* M1 积分增益 */
#define CONFIG_SPEED_M1_KD                        (0.0f)   /* M1 微分增益，当前关闭 */
#define CONFIG_SPEED_M2_KP                        (3.6f)   /* M2 比例增益：提高动态跟随 */
#define CONFIG_SPEED_M2_KI                        (4.0f)   /* M2 积分增益 */
#define CONFIG_SPEED_M2_KD                        (0.0f)   /* M2 微分增益，当前关闭 */

/* PID 限幅、积分分离与启动托底 */
#define CONFIG_SPEED_OUTPUT_MIN                   (0.0f)   /* 总输出下限/千分比 */
#define CONFIG_SPEED_OUTPUT_MAX                   (900.0f) /* 总输出上限/千分比 */
#define CONFIG_SPEED_INTEGRAL_MIN                 (-200.0f)/* 积分下限/千分比 */
#define CONFIG_SPEED_INTEGRAL_MAX                 (120.0f) /* 积分上限/千分比 */
#define CONFIG_SPEED_INTEGRAL_SEPARATION_RPM      (30.0f)  /* 大误差冻结同方向积分/RPM */
#define CONFIG_SPEED_D_FILTER_TAU_S               (0.03f)  /* D 项低通时间常数/s */
#define CONFIG_SPEED_STARTUP_MIN_OUTPUT           (160.0f) /* 静止启动输出托底/千分比 */
#define CONFIG_SPEED_STARTUP_RPM_THRESHOLD        (8.0f)   /* 托底释放转速上限/RPM */
#define CONFIG_SPEED_STARTUP_MIN_TARGET_RPM       (3.0f)   /* 目标至少达到此值才启用托底 */
#define CONFIG_SPEED_STARTUP_RELEASE_RATIO        (0.60f)  /* 实速达到目标比例后退出托底 */

/* 堵转恢复与超时停机 */
#define CONFIG_SPEED_STALL_RECOVERY_ENABLE        (1u)    /* 1=启用堵转恢复 */
#define CONFIG_SPEED_STALL_ENTER_RPM              (10.0f) /* 进入堵转的低速阈值/RPM */
#define CONFIG_SPEED_STALL_EXIT_RPM               (20.0f) /* 退出助推的转速阈值/RPM */
#define CONFIG_SPEED_STALL_MIN_ERROR_RPM           (30.0f) /* 允许助推的最小正误差/RPM */
#define CONFIG_SPEED_STALL_GUARD_MS               (300u)  /* 启动后检测宽限/ms */
#define CONFIG_SPEED_STALL_DETECT_MS              (50u)   /* 连续低速确认时间/ms */
#define CONFIG_SPEED_STALL_BOOST_OUTPUT           (600.0f)/* 堵转助推输出下限/千分比 */
#define CONFIG_SPEED_STALL_TIMEOUT_MS             (1500u) /* 助推超时停机时间/ms */

/* 方向保护 */
#define CONFIG_SPEED_DIRECTION_FAULT_ENABLE       (0u)    /* 极性已标定；0避免外力反拖误停 */
#define CONFIG_SPEED_DIRECTION_FAULT_RPM          (5.0f)  /* 负 RPM 故障阈值 */
#define CONFIG_SPEED_DIRECTION_GUARD_MS           (300u)  /* 启动方向检测宽限/ms */
#define CONFIG_SPEED_DIRECTION_FAULT_SAMPLES      (5u)    /* 连续反向确认次数 */

/* 镜像安装：编码器 INVERT 由 COMMAND_SIGN 自动同步，不再单独调极性 */
#define CONFIG_SPEED_M1_COMMAND_SIGN              (-1)    /* M1 当前车体前进命令极性 */
#define CONFIG_SPEED_M2_COMMAND_SIGN              (+1)    /* M2 当前车体前进命令极性 */

/* 编码器：13 个 AB 周期 × x4 × 28:1 = 1456 count/车轮圈 */
#define CONFIG_ENCODER_AB_CYCLES_PER_MOTOR_REV    (13u)   /* 电机轴每圈 AB 完整周期 */
#define CONFIG_ENCODER_DECODE_MULTIPLIER           (4u)    /* 正交 x4 解码 */
#define CONFIG_ENCODER_COUNTS_PER_MOTOR_REV        (CONFIG_ENCODER_AB_CYCLES_PER_MOTOR_REV * CONFIG_ENCODER_DECODE_MULTIPLIER) /* 电机轴每圈计数 */
#define CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_NUM (28u)  /* M1 减速比分子 */
#define CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_DEN (1u)   /* M1 减速比分母 */
#define CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_NUM (28u)  /* M2 减速比分子 */
#define CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_DEN (1u)   /* M2 减速比分母 */
#define CONFIG_ENCODER_M1_INVERT_DIRECTION         ((CONFIG_SPEED_M1_COMMAND_SIGN > 0) ? 1u : 0u) /* 随 M1 命令自动同步 */
#define CONFIG_ENCODER_M2_INVERT_DIRECTION         ((CONFIG_SPEED_M2_COMMAND_SIGN > 0) ? 1u : 0u) /* 随 M2 命令自动同步 */
#define CONFIG_ENCODER_MEASUREMENT_WINDOW_MS       (50u)   /* RPM窗口/ms：降低角度串级环延迟 */

/* Yaw中环：Yaw误差 -> 左右轮差速RPM；先保证此环独立稳定再调巡线外环 */
#define CONFIG_YAW_BUTTON_STEP_DEG                  (90.0f) /* PB21单次相对旋转角度/deg */
#define CONFIG_YAW_KP                               (1.60f) /* 比例：增大则转向更快，过大会摆振 */
#define CONFIG_YAW_KI                               (0.02f) /* 积分/RPM/(deg*s)：消除静差，过大会慢摆 */
#define CONFIG_YAW_KD                               (0.14f) /* 角速度阻尼/RPM/(deg/s)：过大会抖动迟钝 */
#define CONFIG_YAW_OUTPUT_MIN_RPM                   (-75.0f)/* 差速下限/RPM；与上限保持对称 */
#define CONFIG_YAW_OUTPUT_MAX_RPM                   (75.0f) /* 差速上限/RPM；限制最大转向强度 */
#define CONFIG_YAW_GYRO_SIGN                        (-1)    /* 使SIGN*gyro_z与Yaw数值变化方向一致 */
#define CONFIG_YAW_INTEGRAL_MIN_RPM                 (-6.0f) /* I项输出下限/RPM，限制长期偏置 */
#define CONFIG_YAW_INTEGRAL_MAX_RPM                 (6.0f)  /* I项输出上限/RPM，限制长期偏置 */
#define CONFIG_YAW_INTEGRAL_SEPARATION_DEG          (5.0f)  /* 超出此误差时冻结同向积分/deg */
#define CONFIG_YAW_ZERO_COMMAND_RPM                 (0.20f) /* 小于此差速归零/RPM；不是角度死区 */
#define CONFIG_YAW_MIN_TURN_RPM                     (3.0f)  /* 触发静摩擦补偿后的最小差速/RPM */
#define CONFIG_YAW_STATIC_COMP_ERROR_DEG            (1.50f) /* 角误差达到此值可触发静摩擦补偿/deg */
#define CONFIG_YAW_STATIC_COMP_GYRO_DPS             (8.0f)  /* 角速度达到此值可触发静摩擦补偿/(deg/s) */
#define CONFIG_YAW_LOCK_ERROR_DEG                   (0.80f) /* 仅用于锁定状态判定的角误差范围/deg */
#define CONFIG_YAW_GYRO_TOLERANCE_DPS               (3.0f)  /* 仅用于锁定状态判定的角速度范围/(deg/s) */
#define CONFIG_YAW_OUTPUT_SIGN                      (-1)    /* 已标定闭环极性；方向发散才检查此项 */
#define CONFIG_YAW_M1_WHEEL_SIGN                    (+1)    /* 正turn_rpm叠加到M1左轮目标 */
#define CONFIG_YAW_M2_WHEEL_SIGN                    (-1)    /* 正turn_rpm反向叠加到M2右轮目标 */

/* 传感器 */
#define CONFIG_BMI270_ENABLED                      (0u)    /* 0=暂停；当前引脚与 WT61/PB21 冲突 */
#define CONFIG_BMI270_SPI_HALF_CYCLES              (4u)    /* 软件 SPI 半周期延时计数 */
#define CONFIG_WT61_FRAME_SIZE                     (11u)   /* WT61 标准帧长度/字节 */
#define CONFIG_WT61_STALE_TIMEOUT_MS               (500u)  /* WT61 离线超时/ms */
#define CONFIG_LINE_SENSOR_ADC_THRESHOLD           (1000u)  /* 仅用于启停线/二值位图；不参与模拟位置计算 */
#define CONFIG_LINE_SENSOR_ANALOG_BLACK_HIGH       (1u)    /* 1=模拟值>阈值为黑线；0=模拟值<阈值为黑线 */
#define CONFIG_LINE_SENSOR_REVERSE_ORDER           (0u)    /* 0=S1左/S8右；实物安装相反时改为1 */
#define CONFIG_LINE_SENSOR_POSITION_DIGITAL        (0u)    /* 原算法：超过阈值的通道等权平均 */
#define CONFIG_LINE_SENSOR_POSITION_ANALOG_WEIGHTED (1u)   /* 新算法：8路原始ADC直接加权平均 */
#define CONFIG_LINE_SENSOR_POSITION_MODE           CONFIG_LINE_SENSOR_POSITION_ANALOG_WEIGHTED /* 在这里选择巡线位置算法 */
#define CONFIG_LINE_SENSOR_UART_RESPONSE_TIMEOUT_MS (20u)  /* 命令响应超时/ms */
#define CONFIG_LINE_SENSOR_UART_STALE_TIMEOUT_MS   (100u)  /* 无状态数据后判串口离线/ms */
#define CONFIG_LINE_SENSOR_UART_MODE_RETRY_MS      (250u)  /* 模块晚供电时重发手动模式/ms */

/*
 * 8路巡线有符号位置权重：S1最左，S8最右，绝对值越大转向请求越强。
 * 满量程仍保持约±1400，使已调好的位置环具有接近旧16路模块的输入尺度。
 * 新模块通道更少、位置跳变更大，先验证方向和权重，之后再单独重调位置环。
 * 左右权重应优先保持近似对称，固定机械偏差用TARGET_POSITION补偿。
 */
#define CONFIG_LINE_SENSOR_WEIGHT_S1                 (-1400) /* 最左边缘 */
#define CONFIG_LINE_SENSOR_WEIGHT_S2                 (-1000) /* 左侧急弯 */
#define CONFIG_LINE_SENSOR_WEIGHT_S3                 (-650)  /* 左侧修正 */
#define CONFIG_LINE_SENSOR_WEIGHT_S4                 (-200)  /* 中心左 */
#define CONFIG_LINE_SENSOR_WEIGHT_S5                 (+200)  /* 中心右 */
#define CONFIG_LINE_SENSOR_WEIGHT_S6                 (+650)  /* 右侧修正 */
#define CONFIG_LINE_SENSOR_WEIGHT_S7                 (+1000) /* 右侧急弯 */
#define CONFIG_LINE_SENSOR_WEIGHT_S8                 (+1400) /* 最右边缘 */

/*
 * 巡线外环：位置误差 -> Yaw目标偏移，随后由Yaw中环和速度内环执行。
 * 推荐顺序：阈值/通道方向/权重 -> 低速只调KP -> 需要时加KD -> 最后少量KI；
 * 调位置PID时先保持Yaw环、速度环参数不变，一次只改一个巡线参数。
 */
#define CONFIG_B21_LINE_FOLLOW_MODE                  (1u)    /* 1=B21启停巡线；0=保留Yaw+90deg测试 */
#define CONFIG_RACE_INFINITE_MODE_DEFAULT             (0u)    /* 0=上电单圈停车；1=上电无限巡线 */
#define CONFIG_RACE_INFINITE_SPEED_PERCENT           (65u)    /* 无限模式速度占普通模式百分比，建议60~70 */
#define CONFIG_LINE_TARGET_POSITION                  (0.0f)  /* 期望质心/权重单位；非0可补偿固定偏差 */
#define CONFIG_LINE_BASE_RPM                         (-105.0f)/* 直道巡航RPM；当前负值代表车体前进 */
#define CONFIG_LINE_CURVE_MIN_RPM                    (-78.0f) /* 大误差最低巡航RPM；越接近0弯中越慢 */
#define CONFIG_LINE_CURVE_SLOW_START_POSITION        (180.0f)/* |位置|超过此值开始由BASE降速 */
#define CONFIG_LINE_CURVE_SLOW_FULL_POSITION         (600.0f)/* |位置|达到此值降至CURVE_MIN，须大于START */
#define CONFIG_LINE_ACCEL_RPM_PER_S                  (65.0f) /* 起步/加速斜率；减小更平缓但加速更慢 */
#define CONFIG_LINE_DECEL_RPM_PER_S                  (150.0f)/* 入弯减速斜率；增大则更快降至弯道速度 */
#define CONFIG_LINE_STOP_RPM_PER_S                   (220.0f)/* 终点制动斜率；减小更柔和但停车距离变长 */
#define CONFIG_LINE_LOST_BASE_RPM                    (0.0f)  /* 短时丢线保留的共同速度/RPM；0为原地纠偏 */
#define CONFIG_LINE_LOST_TIMEOUT_MS                  (600u)  /* 连续丢线故障延时/ms；超时后停止控制 */
#define CONFIG_LINE_POSITION_FILTER_TAU_S            (0.030f)/* 位置低通/s；增大更稳但转弯响应更慢 */
#define CONFIG_LINE_KP                               (0.0675f)/* P单位deg/权重；增大转向更强，过大会蛇形 */
#define CONFIG_LINE_KI                               (0.000f)  /* I单位deg/(权重*s)；消静差，过大会慢摆 */
#define CONFIG_LINE_KD                               (0.0015f)  /* D单位deg*s/权重；抑制摆动但放大跳变噪声 */
#define CONFIG_LINE_OUTPUT_SIGN                      (-1)    /* 已标定转向极性；线路偏右时应向右修正 */
#define CONFIG_LINE_YAW_OFFSET_MIN_DEG               (-60.0f)/* 位置环输出下限/deg；限制左向Yaw请求 */
#define CONFIG_LINE_YAW_OFFSET_MAX_DEG               (60.0f) /* 位置环输出上限/deg；限制右向Yaw请求 */
#define CONFIG_LINE_INTEGRAL_MIN_DEG                 (-3.0f) /* I项输出下限/deg，防止长期积分饱和 */
#define CONFIG_LINE_INTEGRAL_MAX_DEG                 (3.0f)  /* I项输出上限/deg，防止长期积分饱和 */
#define CONFIG_LINE_INTEGRAL_SEPARATION              (180.0f)/* |误差|超限时冻结同向积分/权重单位 */
#define CONFIG_LINE_DERIVATIVE_FILTER_TAU_S          (0.04f) /* D项低通/s；增大更稳但阻尼响应更慢 */

/* B21直接启动；到达开放时间后，在滑动窗内累计被斜终线扫过的通道。 */
#define CONFIG_RACE_MARKER_SWEEP_WINDOW_MS          (200u)   /* 通道判黑时间戳的保留窗口/ms */
#define CONFIG_RACE_MARKER_SWEEP_MIN_CHANNELS         (6u)   /* 窗内至少扫过5个不同通道 */
#define CONFIG_RACE_MARKER_SWEEP_LEFT_MASK          (0x07u)  /* 左外侧S1~S3至少扫到1路 */
#define CONFIG_RACE_MARKER_SWEEP_RIGHT_MASK         (0xE0u)  /* 右外侧S6~S8至少扫到1路 */
#define CONFIG_RACE_MARKER_CONFIRM_MS                (20u)   /* 扫线覆盖条件连续确认/ms */
#define CONFIG_RACE_FINISH_MIN_TIME_MS              (16000u) /* 当前16s后开放；需18s时改为18000u */
#define CONFIG_RACE_FINISH_BRAKE_DELAY_MS            (0u)    /* 识别终点后等待制动/ms；增大会越线更远 */
#define CONFIG_RACE_AUTO_STOP_TIME_MS               (17500u) /* 定时自动停车时间/ms；禁用启停线时到期锁存并制动 */
#define CONFIG_RACE_AUTO_STOP_BRAKE_DELAY_MS         (0u)    /* 定时停车后的制动延迟/ms；0为立即制动 */

/*
 * VOFA串口：UART0，PA10=TX、PA11=RX、DMA_CH0发送。
 * 引脚/实例/DMA仍由empty.syscfg分配；以下参数由BSP_Uart_Init()真正应用。
 * 校验定义：0=None、1=Even、2=Odd。
 */
#define CONFIG_VOFA_PORT                           BSP_UART_HOST /* UART0/PA10发送 */
#define CONFIG_VOFA_UART_BAUD_RATE                 (115200u) /* 无线模块与VOFA必须一致 */
#define CONFIG_VOFA_UART_DATA_BITS                 (8u)    /* JustFloat必须使用8位数据 */
#define CONFIG_VOFA_UART_STOP_BITS                 (1u)    /* 1或2 */
#define CONFIG_VOFA_UART_PARITY                    (0u)    /* 0=None，1=Even，2=Odd */
#define CONFIG_VOFA_UART_TX_DMA_BUFFER_SIZE        (128u)  /* DMA持久发送缓冲区/字节 */
#define CONFIG_VOFA_CHANNELS                       (12u)   /* 巡线位置外环专用12通道 */
#define CONFIG_VOFA_UART_EXCLUSIVE                 (1u)    /* UART0仅发送JustFloat */

/* 线路占用检查使用：1起始位 + 数据位 + 可选校验位 + 停止位。 */
#define CONFIG_VOFA_UART_PARITY_BITS               \
    ((CONFIG_VOFA_UART_PARITY == 0u) ? 0u : 1u)
#define CONFIG_VOFA_UART_BITS_PER_BYTE             \
    (1u + CONFIG_VOFA_UART_DATA_BITS + \
     CONFIG_VOFA_UART_PARITY_BITS + CONFIG_VOFA_UART_STOP_BITS)
#define CONFIG_VOFA_FRAME_BYTES                    \
    (CONFIG_VOFA_CHANNELS * 4u + 4u)

/* 布尔量与方向极性检查 */
#if ((CONFIG_MOTOR_PWM_BRAKE_DECAY != 0u) && (CONFIG_MOTOR_PWM_BRAKE_DECAY != 1u))
#error "CONFIG_MOTOR_PWM_BRAKE_DECAY must be 0 or 1"
#endif

#if ((CONFIG_SPEED_FEEDFORWARD_ENABLE != 0u) && (CONFIG_SPEED_FEEDFORWARD_ENABLE != 1u))
#error "CONFIG_SPEED_FEEDFORWARD_ENABLE must be 0 or 1"
#endif

#if ((CONFIG_SPEED_STALL_RECOVERY_ENABLE != 0u) && (CONFIG_SPEED_STALL_RECOVERY_ENABLE != 1u))
#error "CONFIG_SPEED_STALL_RECOVERY_ENABLE must be 0 or 1"
#endif

#if ((CONFIG_SPEED_DIRECTION_FAULT_ENABLE != 0u) && (CONFIG_SPEED_DIRECTION_FAULT_ENABLE != 1u))
#error "CONFIG_SPEED_DIRECTION_FAULT_ENABLE must be 0 or 1"
#endif

#if ((CONFIG_SPEED_M1_COMMAND_SIGN != 1) && (CONFIG_SPEED_M1_COMMAND_SIGN != -1)) || \
    ((CONFIG_SPEED_M2_COMMAND_SIGN != 1) && (CONFIG_SPEED_M2_COMMAND_SIGN != -1))
#error "Motor command signs must be +1 or -1"
#endif

#if ((CONFIG_YAW_OUTPUT_SIGN != 1) && (CONFIG_YAW_OUTPUT_SIGN != -1)) || \
    ((CONFIG_YAW_GYRO_SIGN != 1) && (CONFIG_YAW_GYRO_SIGN != -1)) || \
    ((CONFIG_YAW_M1_WHEEL_SIGN != 1) && (CONFIG_YAW_M1_WHEEL_SIGN != -1)) || \
    ((CONFIG_YAW_M2_WHEEL_SIGN != 1) && (CONFIG_YAW_M2_WHEEL_SIGN != -1))
#error "Yaw direction signs must be +1 or -1"
#endif

#if (CONFIG_YAW_M1_WHEEL_SIGN == CONFIG_YAW_M2_WHEEL_SIGN)
#error "Yaw in-place rotation requires opposite M1/M2 wheel signs"
#endif

/* 周期任务频率必须有效，并能由1 ms时基无误差地产生。 */
#if (CONFIG_PID_SPEED_HZ == 0u) || \
    (CONFIG_PID_YAW_HZ == 0u) || \
    (CONFIG_PID_LINE_HZ == 0u) || \
    (CONFIG_TASK_ENCODER_CAPTURE_HZ == 0u) || \
    (CONFIG_TASK_LINE_SENSOR_SCAN_HZ == 0u) || \
    (CONFIG_TASK_CAN_TX_HZ == 0u) || \
    (CONFIG_TASK_VOFA_HZ == 0u) || \
    (CONFIG_TASK_SCREEN_HZ == 0u)
#error "Task and PID frequencies must be greater than zero"
#else
#if ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_PID_SPEED_HZ) != 0u) || \
    ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_PID_YAW_HZ) != 0u) || \
    ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_PID_LINE_HZ) != 0u) || \
    ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_TASK_ENCODER_CAPTURE_HZ) != 0u) || \
    ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_TASK_LINE_SENSOR_SCAN_HZ) != 0u) || \
    ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_TASK_CAN_TX_HZ) != 0u) || \
    ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_TASK_VOFA_HZ) != 0u) || \
    ((CONFIG_SCHEDULER_TICK_HZ % CONFIG_TASK_SCREEN_HZ) != 0u)
#error "Task and PID frequencies must divide the 1000 Hz scheduler tick exactly"
#endif
#endif

#if (CONFIG_CAN_NOMINAL_BIT_RATE != 500000u)
#error "CONFIG_CAN_NOMINAL_BIT_RATE must match empty.syscfg 500 kbps"
#endif

#if (CONFIG_CAN_ID_ACCEL > 0x7FFu)
#error "CONFIG_CAN_ID_ACCEL must be an 11-bit standard CAN identifier"
#endif

#if (CONFIG_CAN_PROTOCOL_VERSION > 7u)
#error "CONFIG_CAN_PROTOCOL_VERSION must fit in status bits 7..5"
#endif

/*
 * 速度PID不得快于编码器RPM更新。当前快照频率直接跟随速度环，此检查用于
 * 防止以后拆开两个宏时产生“一个RPM样本被重复计算多次”的隐蔽配置错误。
 */
#if (CONFIG_TASK_ENCODER_CAPTURE_HZ < CONFIG_PID_SPEED_HZ)
#error "Encoder capture frequency must not be lower than speed PID frequency"
#endif

#if (CONFIG_VOFA_UART_BAUD_RATE == 0u)
#error "CONFIG_VOFA_UART_BAUD_RATE must be greater than zero"
#endif

#if (CONFIG_VOFA_UART_DATA_BITS != 8u)
#error "VOFA JustFloat requires CONFIG_VOFA_UART_DATA_BITS = 8"
#endif

#if ((CONFIG_VOFA_UART_STOP_BITS != 1u) && \
     (CONFIG_VOFA_UART_STOP_BITS != 2u))
#error "CONFIG_VOFA_UART_STOP_BITS must be 1 or 2"
#endif

#if (CONFIG_VOFA_UART_PARITY > 2u)
#error "CONFIG_VOFA_UART_PARITY must be 0(None), 1(Even), or 2(Odd)"
#endif

#if (CONFIG_VOFA_FRAME_BYTES > CONFIG_VOFA_UART_TX_DMA_BUFFER_SIZE)
#error "VOFA frame does not fit the UART TX DMA buffer"
#endif

#if (CONFIG_VOFA_UART_BAUD_RATE < \
     (CONFIG_VOFA_FRAME_BYTES * CONFIG_TASK_VOFA_HZ * \
      CONFIG_VOFA_UART_BITS_PER_BYTE))
#error "VOFA UART baud rate is too low for the configured channels and send rate"
#endif

#if (CONFIG_ENCODER_M1_INVERT_DIRECTION > 1u) || \
    (CONFIG_ENCODER_M2_INVERT_DIRECTION > 1u)
#error "Encoder invert settings must be 0 or 1"
#endif

#if (CONFIG_BMI270_ENABLED > 1u) || \
    (CONFIG_LINE_SENSOR_ANALOG_BLACK_HIGH > 1u) || \
    (CONFIG_LINE_SENSOR_REVERSE_ORDER > 1u) || \
    (CONFIG_VOFA_UART_EXCLUSIVE > 1u)
#error "Feature switches must be 0 or 1"
#endif

#if (CONFIG_LINE_SENSOR_ADC_THRESHOLD > 65535u)
#error "Line sensor ADC threshold must be in the uint16 range 0..65535"
#endif

#if (CONFIG_LINE_SENSOR_POSITION_MODE != \
     CONFIG_LINE_SENSOR_POSITION_DIGITAL) && \
    (CONFIG_LINE_SENSOR_POSITION_MODE != \
     CONFIG_LINE_SENSOR_POSITION_ANALOG_WEIGHTED)
#error "Line sensor position mode must be DIGITAL or ANALOG_WEIGHTED"
#endif

#if (CONFIG_LINE_SENSOR_UART_RESPONSE_TIMEOUT_MS == 0u) || \
    (CONFIG_LINE_SENSOR_UART_STALE_TIMEOUT_MS <= \
     CONFIG_LINE_SENSOR_UART_RESPONSE_TIMEOUT_MS) || \
    (CONFIG_LINE_SENSOR_UART_MODE_RETRY_MS <= \
     CONFIG_LINE_SENSOR_UART_STALE_TIMEOUT_MS)
#error "Line sensor UART timeouts must satisfy 0 < response < stale < retry"
#endif

/* 位置权重必须从S1到S8严格递增，并在S4/S5之间跨过0。 */
#if (CONFIG_LINE_SENSOR_WEIGHT_S1 >= CONFIG_LINE_SENSOR_WEIGHT_S2) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S2 >= CONFIG_LINE_SENSOR_WEIGHT_S3) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S3 >= CONFIG_LINE_SENSOR_WEIGHT_S4) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S4 >= CONFIG_LINE_SENSOR_WEIGHT_S5) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S5 >= CONFIG_LINE_SENSOR_WEIGHT_S6) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S6 >= CONFIG_LINE_SENSOR_WEIGHT_S7) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S7 >= CONFIG_LINE_SENSOR_WEIGHT_S8)
#error "Line sensor weights must increase strictly from S1 to S8"
#endif

#if (CONFIG_LINE_SENSOR_WEIGHT_S1 < -32768) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S8 > 32767) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S4 >= 0) || \
    (CONFIG_LINE_SENSOR_WEIGHT_S5 <= 0)
#error "Line sensor weights must fit int16 and cross zero between S4/S5"
#endif

#if ((CONFIG_B21_LINE_FOLLOW_MODE != 0u) && \
     (CONFIG_B21_LINE_FOLLOW_MODE != 1u))
#error "CONFIG_B21_LINE_FOLLOW_MODE must be 0 or 1"
#endif

#if (CONFIG_RACE_INFINITE_MODE_DEFAULT > 1u)
#error "CONFIG_RACE_INFINITE_MODE_DEFAULT must be 0 or 1"
#endif

#if (CONFIG_RACE_INFINITE_SPEED_PERCENT == 0u) || \
    (CONFIG_RACE_INFINITE_SPEED_PERCENT > 100u)
#error "CONFIG_RACE_INFINITE_SPEED_PERCENT must be in the range 1..100"
#endif

#if ((CONFIG_LINE_OUTPUT_SIGN != 1) && (CONFIG_LINE_OUTPUT_SIGN != -1))
#error "CONFIG_LINE_OUTPUT_SIGN must be +1 or -1"
#endif

#if (CONFIG_RACE_MARKER_SWEEP_MIN_CHANNELS == 0u) || \
    (CONFIG_RACE_MARKER_SWEEP_MIN_CHANNELS > 8u)
#error "Race marker swept channels must be in the range 1..8"
#endif

#if (CONFIG_RACE_MARKER_SWEEP_WINDOW_MS == 0u) || \
    (CONFIG_RACE_MARKER_CONFIRM_MS == 0u)
#error "Race marker sweep window and confirmation time must be nonzero"
#endif

#if ((CONFIG_RACE_MARKER_SWEEP_LEFT_MASK & 0xFFu) == 0u) || \
    ((CONFIG_RACE_MARKER_SWEEP_RIGHT_MASK & 0xFFu) == 0u) || \
    ((CONFIG_RACE_MARKER_SWEEP_LEFT_MASK & ~0xFFu) != 0u) || \
    ((CONFIG_RACE_MARKER_SWEEP_RIGHT_MASK & ~0xFFu) != 0u)
#error "Race marker sweep side masks must be nonzero 8-bit masks"
#endif

/* 编码器比例与时间检查 */
#if ((CONFIG_ENCODER_MEASUREMENT_WINDOW_MS * \
      CONFIG_TASK_ENCODER_CAPTURE_HZ) < CONFIG_SCHEDULER_TICK_HZ)
#error "Encoder measurement window must not be shorter than update period"
#endif

#if (((CONFIG_ENCODER_MEASUREMENT_WINDOW_MS * \
       CONFIG_TASK_ENCODER_CAPTURE_HZ) % CONFIG_SCHEDULER_TICK_HZ) != 0u)
#error "Encoder measurement window must be an integer multiple of update period"
#endif

#if (CONFIG_ENCODER_COUNTS_PER_MOTOR_REV == 0u)
#error "Encoder motor-shaft counts per revolution must be greater than zero"
#endif

#if (CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_NUM == 0u) || \
    (CONFIG_ENCODER_M1_MOTOR_TO_WHEEL_RATIO_DEN == 0u) || \
    (CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_NUM == 0u) || \
    (CONFIG_ENCODER_M2_MOTOR_TO_WHEEL_RATIO_DEN == 0u)
#error "Encoder motor-to-wheel ratio numerator and denominator must be non-zero"
#endif

#endif /* PROJECT_CONFIG_H_ */
