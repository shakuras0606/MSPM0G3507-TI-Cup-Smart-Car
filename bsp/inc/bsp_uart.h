/**
 * @file    bsp_uart.h
 * @brief   UART0 VOFA发送抽象层（PA10无线串口/CH343 + DMA）
 *
 * 当前串口分工：
 *   - UART0 PA10/PA11：无线串口或板载CH343，VOFA使用TX DMA；
 *   - UART1 PA8/PA9：WT61TTL，由 wt61.c 独占并使用 RX DMA。
 *
 * UART1 和原 PB6/PB7 蓝牙脚属于同一个 UART1 外设，不能同时工作。
 * 如需同时接蓝牙，应改接 UART2/UART3 的另一组可复用引脚。
 */

#ifndef BSP_UART_H_
#define BSP_UART_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** 应用层可用的上位机串口。 */
typedef enum
{
    BSP_UART_HOST = 0,
    BSP_UART_COUNT
} BspUartPort;

/** 按project_config.h设置UART0格式，并初始化TX DMA状态和完成中断。 */
void BSP_Uart_Init(void);

/**
 * @brief 非阻塞提交一块 UART0 TX DMA 数据。
 *
 * 函数会先复制到驱动内部持久缓冲区，因此调用者返回后可立即修改原数据。
 * DMA 忙、参数错误或长度超过内部缓冲区时返回 0；成功时返回 length。
 */
size_t BSP_Uart_WriteFifo(BspUartPort port, const uint8_t *data, size_t length);

/**
 * @brief 使用 DMA 阻塞发送任意长度的数据。
 *
 * 仅用于低频配置/诊断；实时遥测应调用 BSP_Uart_WriteFifo()。
 */
void BSP_Uart_Write(BspUartPort port, const uint8_t *data, size_t length);

/** 使用 DMA 阻塞发送一个字节。 */
void BSP_Uart_WriteByte(BspUartPort port, uint8_t value);

/** 使用 DMA 阻塞发送以 '\0' 结尾的字符串。 */
void BSP_Uart_WriteString(BspUartPort port, const char *text);

/** 查询上位机 TX DMA 是否仍在传输。 */
bool BSP_Uart_TxBusy(BspUartPort port);

#endif /* BSP_UART_H_ */
