/**
 * @file    bsp_uart.c
 * @brief   UART0 上位机 TX DMA 实现
 *
 * UART 外设以 921600-8-N-1 工作。每次提交先复制到内部缓冲区，然后由
 * DMA_CH0 把字节搬运到 UART0 TXDATA；CPU 不再逐字节轮询 TX FIFO。
 */

#include "bsp_uart.h"

#include <string.h>

#include "project_config.h"
#include "ti_msp_dl_config.h"

/** DMA 传输期间必须保持不变的内部发送缓冲区。 */
static uint8_t g_host_tx_buffer[CONFIG_HOST_UART_TX_DMA_BUFFER_SIZE];

/** true 表示 DMA_CH0 正在使用 g_host_tx_buffer。 */
static volatile bool g_host_tx_busy;

void BSP_Uart_Init(void)
{
    g_host_tx_busy = false;
    NVIC_ClearPendingIRQ(HOST_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(HOST_UART_INST_INT_IRQN);
}

size_t BSP_Uart_WriteFifo(BspUartPort port, const uint8_t *data, size_t length)
{
    uint32_t primask;

    if ((port != BSP_UART_HOST) || (data == NULL) || (length == 0u) ||
        (length > sizeof(g_host_tx_buffer))) {
        return 0u;
    }

    /*
     * “检查 busy + 置 busy”必须原子完成，否则 DMA 完成中断恰好插入时，
     * 主循环可能错误覆盖仍在传输的内部缓冲区。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_host_tx_busy) {
        if (primask == 0u) {
            __enable_irq();
        }
        return 0u;
    }
    g_host_tx_busy = true;
    if (primask == 0u) {
        __enable_irq();
    }

    memcpy(g_host_tx_buffer, data, length);

    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
                      (uint32_t)&g_host_tx_buffer[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
                       (uint32_t)&HOST_UART_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, (uint16_t)length);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    return length;
}

void BSP_Uart_Write(BspUartPort port, const uint8_t *data, size_t length)
{
    while ((data != NULL) && (length != 0u)) {
        size_t chunk = length;

        if (chunk > sizeof(g_host_tx_buffer)) {
            chunk = sizeof(g_host_tx_buffer);
        }
        while (BSP_Uart_WriteFifo(port, data, chunk) == 0u) {
            __WFE();
        }
        while (BSP_Uart_TxBusy(port)) {
            __WFE();
        }
        data += chunk;
        length -= chunk;
    }
}

void BSP_Uart_WriteByte(BspUartPort port, uint8_t value)
{
    BSP_Uart_Write(port, &value, 1u);
}

void BSP_Uart_WriteString(BspUartPort port, const char *text)
{
    size_t length = 0u;

    if (text == NULL) {
        return;
    }
    while (text[length] != '\0') {
        ++length;
    }
    BSP_Uart_Write(port, (const uint8_t *)text, length);
}

bool BSP_Uart_TxBusy(BspUartPort port)
{
    return (port == BSP_UART_HOST) ? g_host_tx_busy : false;
}

/**
 * @brief UART0 中断：只处理 TX DMA 搬运完成。
 *
 * DMA_DONE_TX 表示源缓冲区已经搬完，可以安全接收下一帧；UART FIFO 中
 * 可能仍有少量字节在移出，但后续 DMA 可以无缝继续填充 FIFO。
 */
void HOST_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(HOST_UART_INST)) {
    case DL_UART_MAIN_IIDX_DMA_DONE_TX:
        g_host_tx_busy = false;
        break;
    default:
        break;
    }
}
