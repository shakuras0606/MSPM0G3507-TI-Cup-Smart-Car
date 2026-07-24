/**
 * @file    vofa.h
 * @brief   VOFA+ JustFloat 协议输出模块（UART TX DMA 非阻塞）
 *
 * 通过 UART0 TX DMA 非阻塞发送 float32 二进制帧。
 *
 * JustFloat 帧格式 (小端):
 *   [float32_ch0]...[float32_chN][0x00 0x00 0x80 0x7F]
 *
 * Vofa_PutFloat() 压入通道, Vofa_Send() 非阻塞发送整帧,
 * Vofa_Poll() 在主循环中排空残留数据。
 */

#ifndef VOFA_H_
#define VOFA_H_

#include <stddef.h>
#include <stdint.h>
#include "bsp_uart.h"

#define VOFA_MAX_CHANNELS   (10u)

void Vofa_Init(BspUartPort port, uint8_t channel_count);
void Vofa_PutFloat(float value);
void Vofa_Send(void);

/**
 * @brief 排空未发送的残留数据 (每帧主循环调用)
 *
 * 应在 App_RunOnce() 中每次循环都调用,
 * 确保非阻塞发送的尾部数据最终排空。
 */
void Vofa_Poll(void);

#endif /* VOFA_H_ */
