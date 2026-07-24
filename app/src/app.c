/**
 * @file    app.c
 * @brief   TI3507-CAR 裸机协作式任务调度（v0.6.0双轮定速）
 *
 * 主循环不是“只能放屏幕”。它持续运行短小的非阻塞任务：
 *   1. 解析 WT61 DMA 接收数据；
 *   2. 对中断采集的编码器快照计算车轮 RPM；
 *   3. 100 Hz 通过 UART0 TX DMA 输出 VOFA；
 *   4. 5 Hz 局部刷新 ST7735。
 *   5. 板载PB21按键切换双轮共同目标RPM，10 ms运行独立速度PID。
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
#include "drv8871.h"
#include "encoder.h"
#include "line_sensor16.h"
#include "project_config.h"
#include "screen_task.h"
#include "speed_control.h"
#include "vofa.h"
#include "wt61.h"

typedef struct
{
    uint32_t last_vofa_ms;      /**< 上次组装VOFA帧的时间，单位ms。 */
    uint32_t last_screen_ms;    /**< 上次刷新ST7735动态区域的时间，单位ms。 */
} AppState;

/** 应用层协作式任务调度状态；不在ISR中写入。 */
static AppState g_app;

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
    WT61Snapshot imu;
    SpeedControlSnapshot speed;

    (void)WT61_GetSnapshot(&imu);
    (void)SpeedControl_GetSnapshot(&speed);

    /*
     * PID调试通道：
     * TARGET, M1_RPM, M2_RPM, M1_OUT, M2_OUT, M1_ERR, M2_ERR,
     * DIRECTION_FAULT_MASK, YAW, PITCH, ROLL。
     */
    Vofa_PutFloat(speed.target_rpm);
    Vofa_PutFloat(speed.m1_rpm);
    Vofa_PutFloat(speed.m2_rpm);
    Vofa_PutFloat(speed.m1_output_permille);
    Vofa_PutFloat(speed.m2_output_permille);
    Vofa_PutFloat(speed.m1_terms.error);
    Vofa_PutFloat(speed.m2_terms.error);
    Vofa_PutFloat((float)speed.direction_fault_mask);
    Vofa_PutFloat(imu.yaw_deg);
    Vofa_PutFloat(imu.pitch_deg);
    Vofa_PutFloat(imu.roll_deg);
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
    SpeedControlSnapshot speed;
    bool online;

    read_inputs(&m1, &m2, &imu);
    (void)SpeedControl_GetSnapshot(&speed);
    /* 必须曾收到有效角度帧，且帧龄没有超过离线阈值。 */
    online = imu.angle_valid &&
             ((uint32_t)(BSP_Time_Millis() - imu.last_angle_ms) <=
              CONFIG_WT61_STALE_TIMEOUT_MS);
    ScreenTask_Update((int32_t)speed.target_rpm, m1.rpm, m2.rpm,
                      imu.yaw_deg, imu.pitch_deg, imu.roll_deg, online);
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
    Vofa_Init(CONFIG_VOFA_PORT, CONFIG_VOFA_CHANNELS);
    ScreenTask_Init();

    now = BSP_Time_Millis();
    g_app.last_vofa_ms = now;
    g_app.last_screen_ms = now;
}

/**
 * @brief 执行一轮非阻塞协作式任务；由main中的无限循环持续调用。
 *
 * 优先级由调用顺序体现：
 *   1. 先消费DMA接收数据和编码器快照；
 *   2. 处理按键并运行10 ms速度环；
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
    /* 一次消抖按下事件只切换一个档位，长按不会持续加速。 */
    if (BoardButton_PressedEvent(now)) {
        SpeedControl_CycleTarget();
    }
    /* 每轮调用，函数内部按CONFIG_SPEED_CONTROL_PERIOD_MS自行门控。 */
    SpeedControl_Update(now);
    Vofa_Poll();

    now = BSP_Time_Millis();
    /* 无符号时间差写法允许32位毫秒计数器自然回绕。 */
    if ((uint32_t)(now - g_app.last_vofa_ms) >= CONFIG_VOFA_PERIOD_MS) {
        /*
         * 采用last=now而不是last+=period：主循环偶发延迟时不连续补发多帧，
         * 防止串口突发堆积。遥测允许轻微抖动，速度环仍使用真实dt。
         */
        g_app.last_vofa_ms = now;
        send_vofa_frame();
    }

    if ((uint32_t)(now - g_app.last_screen_ms) >=
        CONFIG_SCREEN_PERIOD_MS) {
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
