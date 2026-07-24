/**
 * @file    board_button.c
 * @brief   板载PB21按键轮询和30 ms消抖实现
 *
 * 板载按键使用内部上拉：松开为高，按下接地为低。
 * 人机按键采用主循环轮询即可，不占用编码器所在的GPIO GROUP1中断。
 *
 * 消抖状态机维护两个电平：
 *   g_raw_pressed    = 最近一次直接读取的原始电平；
 *   g_stable_pressed = 已持续稳定达到消抖时间的确认电平。
 *
 * 只有“稳定状态从松开变为按下”时返回一次true。按住不重复触发，松开
 * 只更新内部状态但不产生事件，下一次重新按下才能再次触发。
 */

#include "board_button.h"

#include "project_config.h"
#include "ti_msp_dl_config.h"

/** 最近一次采样到的原始按压状态，可能仍含机械抖动。 */
static bool g_raw_pressed;

/** 已通过持续时间验证的稳定按压状态。 */
static bool g_stable_pressed;

/** 原始状态最近一次变化的系统时间，单位ms。 */
static uint32_t g_raw_change_ms;

/** 读取板载PB21原始电平并转换为“true=按下”。 */
static bool read_pressed(void)
{
    return (DL_GPIO_readPins(SPEED_KEY_PORT, SPEED_KEY_BUTTON_PIN) == 0u);
}

/**
 * @brief 用当前实物电平初始化消抖状态机。
 * @param now_ms 当前系统毫秒时间。
 *
 * 上电时若按键正被按住，稳定状态也初始化为按下，因此不会把“上电按住”
 * 误认为一次新按键事件；必须先松开并再次按下。
 */
void BoardButton_Init(uint32_t now_ms)
{
    g_raw_pressed = read_pressed();
    g_stable_pressed = g_raw_pressed;
    g_raw_change_ms = now_ms;
}

/**
 * @brief 轮询并返回一次消抖后的按下沿事件。
 * @param now_ms 当前系统毫秒时间。
 * @return 仅在确认“松开->按下”转换的那一次调用返回true。
 *
 * 算法：
 *   1. 原始电平变化时只重启计时，不立刻接受；
 *   2. 原始电平连续保持CONFIG_SPEED_BUTTON_DEBOUNCE_MS后才更新稳定状态；
 *   3. 新稳定状态为按下时产生事件，为松开时不产生事件。
 *
 * 使用无符号时间差可正确处理32位毫秒计数器回绕。
 */
bool BoardButton_PressedEvent(uint32_t now_ms)
{
    bool raw = read_pressed();

    /* 检测任何原始跳变，并从该时刻重新计算稳定持续时间。 */
    if (raw != g_raw_pressed) {
        g_raw_pressed = raw;
        g_raw_change_ms = now_ms;
    }

    /* 只有候选状态尚未被确认且已经稳定足够久，才提交状态转换。 */
    if ((g_stable_pressed != g_raw_pressed) &&
        ((uint32_t)(now_ms - g_raw_change_ms) >=
         CONFIG_SPEED_BUTTON_DEBOUNCE_MS)) {
        g_stable_pressed = g_raw_pressed;
        /* 按下返回true；松开提交状态但返回false。 */
        return g_stable_pressed;
    }
    return false;
}
