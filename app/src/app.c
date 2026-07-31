/**
 * @file    app.c
 * @brief   TI3507-CAR 裸机协作式任务调度（串级Yaw角度控制）
 *
 * 主循环不是“只能放屏幕”。它持续运行短小的非阻塞任务：
 *   1. 解析 WT61 DMA 接收数据；
 *   2. 对中断采集的编码器快照计算车轮 RPM；
 *   3. 100 Hz 通过 UART0 TX DMA 输出 VOFA；
 *   4. 5 Hz 局部刷新 ST7735。
 *   5. 位置外环产生Yaw修正目标，Yaw中环产生双轮差速RPM；
 *   6. 两个车轮速度内环分别输出PWM，形成三串级巡线；
 *   7. 200 Hz扫描8路巡线模块并在屏幕显示ADC、二值位和质心位置。
 *   8. 100 Hz通过经典CAN向钢珠控制板发送WT61三轴加速度。
 *
 * 中断只做有硬实时要求的搬运/采样；协议、除法、显示均留在主循环。
 *
 * 调度方式不是阻塞delay，而是“当前时间-上次时间 >= 周期”的截止时间
 * 检查。SysTick中断维护1 ms时间基准；主循环任务即使被中断打断，返回后
 * 仍会继续执行，并用真实时间差判断是否到期。
 */

#include "app.h"

#include <stdbool.h>
#include <stdint.h>

#include "board_button.h"
#include "bsp_time.h"
#include "can_telemetry.h"
#include "drv8871.h"
#include "encoder.h"
#include "line_control.h"
#include "line_sensor16.h"
#include "project_config.h"
#include "screen_task.h"
#include "speed_control.h"
#include "vofa.h"
#include "wt61.h"
#include "yaw_control.h"

typedef struct
{
    uint32_t last_vofa_ms;      /**< 上次组装VOFA帧的时间，单位ms。 */
    uint32_t last_screen_ms;    /**< 上次刷新ST7735动态区域的时间，单位ms。 */
    uint32_t last_line_ms;      /**< 上次完成8通道扫描的时间，单位ms。 */
    uint32_t last_can_ms;       /**< 上次提交加速度CAN帧的调度时间，单位ms。 */
    uint32_t race_start_ms;     /**< 本圈按键有效并开始运动的时间。 */
    uint32_t race_finish_ms;    /**< 确认到达终点横线时锁存的本圈用时。 */
    uint32_t marker_change_ms;  /**< 起停线进入/离开确认的起始时间。 */
    uint32_t brake_request_ms;  /**< 再次识别起停线的时间。 */
    bool marker_timing;         /**< true=正在确认终点线。 */
    bool release_timing;        /**< true=正在确认已驶离起跑线。 */
    bool finish_armed;          /**< true=已驶离起跑线，可识别终点。 */
    ScreenRaceState race_state; /**< 屏幕和比赛起停状态。 */
} AppState;

/** 应用层协作式任务调度状态；不在ISR中写入。 */
static AppState g_app;

static bool race_marker_present(const LineSensor16Data *line)
{
    return !line->line_lost &&
           (line->active_count >=
            CONFIG_RACE_MARKER_MIN_ACTIVE_CHANNELS);
}

/** 返回从0开始的比赛用时；终点确认后锁存，制动期间不再变化。 */
static uint32_t race_elapsed_ms(uint32_t now_ms)
{
    if (g_app.race_state == SCREEN_RACE_RUN) {
        return (uint32_t)(now_ms - g_app.race_start_ms);
    } else if ((g_app.race_state == SCREEN_RACE_BRAKE) ||
               (g_app.race_state == SCREEN_RACE_DONE)) {
        return g_app.race_finish_ms;
    }
    return 0u;
}

static void race_start(uint32_t now_ms)
{
    SpeedControl_Stop();
    SpeedControl_ClearFaults();
    if (!LineControl_Start(now_ms)) {
        g_app.race_state = SCREEN_RACE_FAULT;
        return;
    }
    g_app.race_start_ms = now_ms;
    g_app.race_finish_ms = 0u;
    g_app.marker_change_ms = now_ms;
    g_app.brake_request_ms = 0u;
    g_app.marker_timing = false;
    g_app.release_timing = false;
    g_app.finish_armed = false;
    g_app.race_state = SCREEN_RACE_RUN;
}

/** H题A点起停线状态机；B21运行中仍作为人工急停。 */
static void race_update(uint32_t now_ms, bool button_event)
{
    const LineSensor16Data *line = LineSensor16_GetData();
    LineControlSnapshot control;
    bool marker = race_marker_present(line);

    (void)LineControl_GetSnapshot(&control);

    if (button_event) {
        if ((g_app.race_state == SCREEN_RACE_RUN) ||
            (g_app.race_state == SCREEN_RACE_BRAKE)) {
            LineControl_Stop(now_ms);
            g_app.race_finish_ms = 0u;
            g_app.race_state = SCREEN_RACE_WAIT;
            return;
        }
        /* 必须“按下B21”与“至少12路同时识别横线”在同一时刻成立。 */
        if (marker) {
            race_start(now_ms);
        }
        return;
    }

    if ((g_app.race_state != SCREEN_RACE_RUN) &&
        (g_app.race_state != SCREEN_RACE_BRAKE)) {
        return;
    }
    if (!control.enabled && (g_app.race_state == SCREEN_RACE_RUN)) {
        g_app.race_finish_ms =
            (uint32_t)(now_ms - g_app.race_start_ms);
        g_app.race_state = SCREEN_RACE_FAULT;
        return;
    }

    if (g_app.race_state == SCREEN_RACE_BRAKE) {
        if (g_app.brake_request_ms != 0u &&
            (uint32_t)(now_ms - g_app.brake_request_ms) >=
            CONFIG_RACE_FINISH_BRAKE_DELAY_MS &&
            !control.stopping) {
            LineControl_BeginStop(now_ms);
        }
        if (!control.enabled) {
            g_app.race_state = SCREEN_RACE_DONE;
        }
        return;
    }

    /* 起步后必须连续离开横线一段时间，才允许把下一次横线识别为终点。 */
    if (!g_app.finish_armed) {
        if (!marker) {
            if (!g_app.release_timing) {
                g_app.marker_change_ms = now_ms;
                g_app.release_timing = true;
            } else if ((uint32_t)(now_ms - g_app.marker_change_ms) >=
                       CONFIG_RACE_MARKER_RELEASE_MS) {
                g_app.finish_armed = true;
                g_app.release_timing = false;
            }
        } else {
            g_app.release_timing = false;
        }
        return;
    }

    /* 最短圈时与连续确认同时满足，避免赛道宽线或ADC毛刺误停。 */
    if ((uint32_t)(now_ms - g_app.race_start_ms) <
        CONFIG_RACE_FINISH_MIN_TIME_MS) {
        return;
    }
    if (marker) {
        if (!g_app.marker_timing) {
            g_app.marker_change_ms = now_ms;
            g_app.marker_timing = true;
        } else if ((uint32_t)(now_ms - g_app.marker_change_ms) >=
                   CONFIG_RACE_MARKER_CONFIRM_MS) {
            g_app.brake_request_ms = now_ms;
            /* 计时在确认到达终点线的这一刻锁存，而不是等柔和制动结束。 */
            g_app.race_finish_ms =
                (uint32_t)(now_ms - g_app.race_start_ms);
            g_app.marker_timing = false;
            g_app.race_state = SCREEN_RACE_BRAKE;
            if (CONFIG_RACE_FINISH_BRAKE_DELAY_MS == 0u) {
                LineControl_BeginStop(now_ms);
            }
        }
    } else {
        g_app.marker_timing = false;
    }
}

/** 获取当前所有显示/遥测量，避免分别读取时跨越多个状态更新点。 */
static void read_inputs(EncoderSnapshot *m1, EncoderSnapshot *m2,
                        WT61Snapshot *imu)
{
    (void)Encoder_GetSnapshot(ENCODER_M1, m1);
    (void)Encoder_GetSnapshot(ENCODER_M2, m2);
    (void)WT61_GetSnapshot(imu);
}

/**
 * @brief 组装并异步发送一帧JustFloat调试数据。
 *
 * Vofa_PutFloat()只依次写入软件帧缓冲；Vofa_Send()启动UART0 TX DMA。
 * CONFIG_VOFA_CHANNELS必须等于这里的PutFloat调用次数。DMA忙时由Vofa_Poll()
 * 管理后续状态，主循环不逐字节等待串口发送。
 */
static void send_vofa_frame(void)
{
    const LineSensor16Data *sensor;
    YawControlSnapshot yaw;
    LineControlSnapshot line;

    sensor = LineSensor16_GetData();
    (void)YawControl_GetSnapshot(&yaw);
    (void)LineControl_GetSnapshot(&line);

    /*
     * 巡线位置外环专用通道：
     *  0 LINE_TARGET       位置目标
     *  1 LINE_RAW          传感器本拍加权位置；ACTIVE_COUNT=0时为旧值
     *  2 LINE_FILTERED     进入位置PID的低通位置
     *  3 LINE_ERROR        LINE_TARGET-LINE_FILTERED
     *  4 LINE_P            位置环P项/deg
     *  5 LINE_I            位置环I项/deg
     *  6 LINE_D            位置环D项/deg
     *  7 LINE_YAW_OFFSET   经过OUTPUT_SIGN后的实际Yaw修正/deg
     *  8 YAW_TARGET        送入已调好Yaw中环的目标角/deg
     *  9 YAW_ACTUAL        WT61相对Yaw反馈/deg
     * 10 LINE_BASE_RPM     弯道降速与斜坡处理后的共同车轮RPM
     * 11 ACTIVE_COUNT      当前判定为黑线的通道数，0表示丢线
     *
     * P/I/D项使用PID内部符号，LINE_YAW_OFFSET才是实际送往Yaw中环的方向。
     * 这样既能独立调位置PID，也能确认Yaw中环是否忠实执行外环请求。
     */
    Vofa_PutFloat(line.target_position);
    Vofa_PutFloat((float)sensor->position);
    Vofa_PutFloat(line.measured_position);
    Vofa_PutFloat(line.position_error);
    Vofa_PutFloat(line.terms.proportional);
    Vofa_PutFloat(line.terms.integral);
    Vofa_PutFloat(line.terms.derivative);
    Vofa_PutFloat(line.yaw_offset_deg);
    Vofa_PutFloat(yaw.target_yaw_deg);
    Vofa_PutFloat(yaw.current_yaw_deg);
    Vofa_PutFloat(line.base_rpm);
    Vofa_PutFloat((float)sensor->active_count);
    Vofa_Send();
}

/**
 * @brief 读取最新快照并刷新ST7735动态数值。
 *
 * 屏幕是低优先级任务。软件SPI刷新期间编码器GPIO中断和UART DMA仍工作；
 * 刷新完成后主循环会再次处理WT61和编码器计算。
 */
static void refresh_screen(void)
{
    EncoderSnapshot m1;
    EncoderSnapshot m2;
    WT61Snapshot imu;
    YawControlSnapshot yaw;
    const LineSensor16Data *line;
    bool online;

    read_inputs(&m1, &m2, &imu);
    (void)YawControl_GetSnapshot(&yaw);
    line = LineSensor16_GetData();
    /* 必须曾收到有效角度帧，且帧龄没有超过离线阈值。 */
    online = imu.angle_valid &&
             ((uint32_t)(BSP_Time_Millis() - imu.last_angle_ms) <=
              CONFIG_WT61_STALE_TIMEOUT_MS);
    ScreenTask_Update(m1.rpm, m2.rpm,
                      yaw.current_yaw_deg, online, line->values,
                      line->raw_mask, line->position, line->line_lost,
                      race_elapsed_ms(BSP_Time_Millis()),
                      g_app.race_state);
}

/**
 * @brief 按依赖顺序初始化应用模块。
 *
 * Board_Init()已在main中完成SysConfig底层初始化。这里先初始化执行器并
 * 保证电机为00停止，再启动编码器、速度控制、按键和各个通信/显示任务。
 */
void App_Init(void)
{
    uint32_t now;

    /* 执行器优先进入已知安全状态，防止后续慢初始化期间误转。 */
    Drv8871_Init(DRV8871_MOTOR_M1);
    Drv8871_Init(DRV8871_MOTOR_M2);
    Encoder_Init();
    SpeedControl_Init(BSP_Time_Millis());
    BoardButton_Init(BSP_Time_Millis());
    LineSensor16_Init();

    /*
     * 先启动 WT61 DMA，再初始化屏幕。ST7735 初始化有数百毫秒延时，
     * 但这期间 UART1 仍由 DMA 接收，编码器仍由中断计数/采样。
     */
    /* UART DMA先于耗时屏幕初始化启动，避免屏幕延时期间丢掉姿态帧。 */
    WT61_Init();
    CanTelemetry_Init();
    YawControl_Init(BSP_Time_Millis());
    LineControl_Init(BSP_Time_Millis());
    Vofa_Init(CONFIG_VOFA_PORT, CONFIG_VOFA_CHANNELS);
    ScreenTask_Init();

    now = BSP_Time_Millis();
    g_app.last_vofa_ms = now;
    g_app.last_screen_ms = now;
    g_app.last_line_ms = now;
    g_app.last_can_ms = now;
    g_app.race_start_ms = now;
    g_app.race_finish_ms = 0u;
    g_app.marker_change_ms = now;
    g_app.brake_request_ms = 0u;
    g_app.marker_timing = false;
    g_app.release_timing = false;
    g_app.finish_armed = false;
    g_app.race_state = SCREEN_RACE_WAIT;
}

/**
 * @brief 执行一轮非阻塞协作式任务；由main中的无限循环持续调用。
 *
 * 优先级由调用顺序体现：
 *   1. 先消费DMA接收数据和编码器快照；
 *   2. 处理按键并运行三级 PID；
 *   3. 维护VOFA DMA；
 *   4. 到期时发送遥测；
 *   5. 最后执行较慢的屏幕刷新。
 */
void App_RunOnce(void)
{
    uint32_t now;

    /* 高频轻量任务：未收到新数据时会快速返回。 */
    WT61_Process();
    Encoder_Update();
    now = BSP_Time_Millis();
    if ((uint32_t)(now - g_app.last_can_ms) >=
        CONFIG_TICKS_FROM_HZ(CONFIG_TASK_CAN_TX_HZ)) {
        /*
         * 不补发过期帧：调度延迟后只发送当前最新WT61快照。CAN层若仍忙
         * 会丢弃本拍，C板通过sequence和ACCEL_FRESH判断连续性。
         */
        g_app.last_can_ms = now;
        (void)CanTelemetry_SendAcceleration(now);
    }
    if ((uint32_t)(now - g_app.last_line_ms) >=
        CONFIG_TICKS_FROM_HZ(CONFIG_TASK_LINE_SENSOR_SCAN_HZ)) {
        g_app.last_line_ms = now;
        LineSensor16_Scan();
    }
    /* 默认进入H题起停线状态机；宏为0时恢复Yaw +90deg阶跃测试。 */
#if (CONFIG_B21_LINE_FOLLOW_MODE != 0u)
    race_update(now, BoardButton_PressedEvent(now));
#else
    if (BoardButton_PressedEvent(now)) {
        YawControl_OnButtonPressed(now);
    }
#endif
    /* 位置外环先给Yaw目标，Yaw中环再给差速RPM，速度内环最后输出PWM。 */
    LineControl_Update(now);
    YawControl_Update(now);
    /* 每轮调用，函数内部按CONFIG_PID_SPEED_HZ自行门控。 */
    SpeedControl_Update(now);
    Vofa_Poll();

    now = BSP_Time_Millis();
    /* 无符号时间差写法允许32位毫秒计数器自然回绕。 */
    if ((uint32_t)(now - g_app.last_vofa_ms) >=
        CONFIG_TICKS_FROM_HZ(CONFIG_TASK_VOFA_HZ)) {
        /*
         * 采用last=now而不是last+=period：主循环偶发延迟时不连续补发多帧，
         * 防止串口突发堆积。遥测允许轻微抖动，速度环仍使用真实dt。
         */
        g_app.last_vofa_ms = now;
        send_vofa_frame();
    }

    if ((uint32_t)(now - g_app.last_screen_ms) >=
        CONFIG_TICKS_FROM_HZ(CONFIG_TASK_SCREEN_HZ)) {
        g_app.last_screen_ms = now;
        refresh_screen();

        /*
         * 软件 SPI 刷屏结束后立刻消化期间 DMA 累积的数据。编码器快照
         * 已由 SysTick 准时采集，因此这里的计算延迟不会改变测速窗口。
         */
        WT61_Process();
        Encoder_Update();
    }
}
