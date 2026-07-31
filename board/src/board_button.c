/**
 * @file    board_button.c
 * @brief   PB21巡线启停键和PB12模式键轮询消抖实现
 *
 * 两个按键均使用内部上拉：松开为高，按下接地为低。
 * 人机按键采用主循环轮询即可，不占用编码器所在的GPIO GROUP1中断。
 *
 * 每个按键都有完全独立的原始电平、稳定电平和变化时间，
 * 同时按下也不会互相覆盖消抖状态。
 *
 * 只有“稳定状态从松开变为按下”时返回一次true。按住不重复触发，松开
 * 只更新内部状态但不产生事件，下一次重新按下才能再次触发。
 */

#include "board_button.h"

#include "project_config.h"
#include "ti_msp_dl_config.h"

typedef struct
{
    bool raw_pressed;
    bool stable_pressed;
    uint32_t raw_change_ms;
} ButtonDebounce;

static ButtonDebounce g_start_button;
static ButtonDebounce g_mode_button;

/** 读取板载PB21原始电平并转换为“true=按下”。 */
static bool read_start_pressed(void)
{
    return (DL_GPIO_readPins(SPEED_KEY_PORT, SPEED_KEY_BUTTON_PIN) == 0u);
}

/** 读取扩展板KEY1/PB12并转换为“true=按下”。 */
static bool read_mode_pressed(void)
{
    return (DL_GPIO_readPins(MODE_KEY_PORT,
                             MODE_KEY_MODE_BUTTON_PIN) == 0u);
}

static void debounce_init(ButtonDebounce *button, bool pressed,
                          uint32_t now_ms)
{
    button->raw_pressed = pressed;
    button->stable_pressed = pressed;
    button->raw_change_ms = now_ms;
}

static bool debounce_pressed_event(ButtonDebounce *button, bool raw,
                                   uint32_t now_ms)
{
    if (raw != button->raw_pressed) {
        button->raw_pressed = raw;
        button->raw_change_ms = now_ms;
    }

    if ((button->stable_pressed != button->raw_pressed) &&
        ((uint32_t)(now_ms - button->raw_change_ms) >=
         CONFIG_SPEED_BUTTON_DEBOUNCE_MS)) {
        button->stable_pressed = button->raw_pressed;
        return button->stable_pressed;
    }
    return false;
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
    debounce_init(&g_start_button, read_start_pressed(), now_ms);
    debounce_init(&g_mode_button, read_mode_pressed(), now_ms);
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
    return debounce_pressed_event(
        &g_start_button, read_start_pressed(), now_ms);
}

bool BoardModeButton_PressedEvent(uint32_t now_ms)
{
    return debounce_pressed_event(
        &g_mode_button, read_mode_pressed(), now_ms);
}
