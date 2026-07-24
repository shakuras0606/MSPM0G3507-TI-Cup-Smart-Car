/**
 * @file    vofa.c
 * @brief   VOFA+ JustFloat 协议输出实现（非阻塞 TX DMA）
 *
 * BSP_Uart_WriteFifo() 当前提交一整块 TX DMA；DMA 忙时保留最新帧，
 * 在后续 Vofa_Poll() 中重试，不会逐字节占用 CPU。
 *
 * VOFA+ 尾标记 00 00 80 7F 是官方定义的帧分隔符。
 */

#include "vofa.h"

#include <string.h>

static const uint8_t kTail[4] = { 0x00u, 0x00u, 0x80u, 0x7Fu };

static BspUartPort g_port;
static uint8_t     g_channel_count;
static uint8_t     g_index;
static uint8_t     g_buf[VOFA_MAX_CHANNELS * 4u + 4u];

/*
 * 非阻塞发送状态: 记录上次未发完的数据起始偏移和剩余长度。
 * 新帧会覆盖旧帧 (JustFloat 协议允许丢帧)。
 */
static const uint8_t *g_tx_data;
static size_t         g_tx_remain;

void Vofa_Init(BspUartPort port, uint8_t channel_count)
{
    g_port = port;
    g_channel_count = (channel_count > VOFA_MAX_CHANNELS) ?
                       VOFA_MAX_CHANNELS : channel_count;
    g_index    = 0u;
    g_tx_data   = (const uint8_t *)0;
    g_tx_remain = 0u;
}

void Vofa_PutFloat(float value)
{
    uint32_t raw;
    uint8_t i;

    if (g_index >= g_channel_count) return;

    memcpy(&raw, &value, sizeof(raw));
    for (i = 0u; i < 4u; ++i) {
        g_buf[g_index * 4u + i] = (uint8_t)(raw >> (i * 8u));
    }
    ++g_index;
}

void Vofa_Send(void)
{
    size_t total;
    size_t sent;

    /* 先尝试发送上次未发完的残留数据 */
    if (g_tx_remain != 0u) {
        sent = BSP_Uart_WriteFifo(g_port, g_tx_data, g_tx_remain);
        g_tx_data   += sent;
        g_tx_remain -= sent;
        if (g_tx_remain != 0u) {
            /* 仍有残留, 本帧数据丢弃 (JustFloat 允许丢帧) */
            g_index = 0u;
            return;
        }
    }

    /* 追加尾标记到缓冲区末尾 */
    total = (size_t)g_index * 4u;
    memcpy(&g_buf[total], kTail, sizeof(kTail));
    total += sizeof(kTail);

    /* 非阻塞写入: 如果一次写不完, 记录剩余数据下次发送 */
    sent = BSP_Uart_WriteFifo(g_port, g_buf, total);

    if (sent < total) {
        g_tx_data   = &g_buf[sent];
        g_tx_remain = total - sent;
    } else {
        g_tx_data   = (const uint8_t *)0;
        g_tx_remain = 0u;
    }

    g_index = 0u;
}

/**
 * @brief 排空非阻塞发送残留
 *
 * 每帧主循环调用, 确保上次未发完的数据最终发送完毕。
 * 若无残留则立即返回 (零开销)。
 */
void Vofa_Poll(void)
{
    size_t sent;

    if (g_tx_remain == 0u) {
        return;
    }
    sent = BSP_Uart_WriteFifo(g_port, g_tx_data, g_tx_remain);
    g_tx_data   += sent;
    g_tx_remain -= sent;
}
