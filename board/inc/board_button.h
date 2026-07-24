/**
 * @file    board_button.h
 * @brief   核心板板载PB21速度按键（低有效、软件消抖）
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

#endif /* BOARD_BUTTON_H_ */
