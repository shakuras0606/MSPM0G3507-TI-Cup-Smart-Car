/**
 * @file    board_button.h
 * @brief   PB21巡线启停键与PB12模式键（低有效、独立软件消抖）
 */

#ifndef BOARD_BUTTON_H_
#define BOARD_BUTTON_H_

#include <stdbool.h>
#include <stdint.h>

/** 初始化按键消抖状态；GPIO方向和内部上拉由SysConfig完成。 */
void BoardButton_Init(uint32_t now_ms);

/**
 * @brief 更新按键消抖并返回一次“确认按下”事件。
 *
 * 按住不连发；必须松开并再次按下才产生下一次事件。
 */
bool BoardButton_PressedEvent(uint32_t now_ms);

/**
 * @brief 更新PB12模式键消抖并返回一次确认按下事件。
 *
 * 每次按下用于在单圈停车模式和无限巡线模式之间切换。
 */
bool BoardModeButton_PressedEvent(uint32_t now_ms);

#endif /* BOARD_BUTTON_H_ */
