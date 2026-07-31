/**
 * @file    bsp_uart.c
 * @brief   UART0 上位机 TX DMA 实现
 *
 * UART0的波特率、校验和停止位由project_config.h统一配置。每次提交先
 * 复制到内部缓冲区，再由DMA_CH0搬运到UART0 TXDATA。
 */

#include "bsp_uart.h"

#include <string.h>

#include "project_config.h"
#include "ti_msp_dl_config.h"

#if (CONFIG_VOFA_UART_PARITY == 0u)
#define VOFA_UART_PARITY_MODE DL_UART_MAIN_PARITY_NONE
#elif (CONFIG_VOFA_UART_PARITY == 1u)
#define VOFA_UART_PARITY_MODE DL_UART_MAIN_PARITY_EVEN
#else
#define VOFA_UART_PARITY_MODE DL_UART_MAIN_PARITY_ODD
#endif

#if (CONFIG_VOFA_UART_STOP_BITS == 1u)
#define VOFA_UART_STOP_BITS_MODE DL_UART_MAIN_STOP_BITS_ONE
#else
#define VOFA_UART_STOP_BITS_MODE DL_UART_MAIN_STOP_BITS_TWO
#endif

/** DMA传输期间必须保持不变的内部发送缓冲区。 */
static uint8_t g_host_tx_buffer[CONFIG_VOFA_UART_TX_DMA_BUFFER_SIZE];

/** true 表示 DMA_CH0 正在使用 g_host_tx_buffer。 */
static volatile bool g_host_tx_busy;

void BSP_Uart_Init(void)
{
    g_host_tx_busy = false;

    /*
     * SysConfig负责UART0时钟、PA10/PA11复用、FIFO、中断和DMA事件。
     * 此处在第一次发送前重新应用应用层配置，使project_config.h成为
     * VOFA波特率和帧格式的实际生效入口，而不只是说明文字。
     */
    DL_UART_Main_disable(HOST_UART_INST);
    DL_UART_Main_configBaudRate(
        HOST_UART_INST,
        HOST_UART_INST_FREQUENCY,
        CONFIG_VOFA_UART_BAUD_RATE);
    DL_UART_Main_setWordLength(
        HOST_UART_INST, DL_UART_MAIN_WORD_LENGTH_8_BITS);
    DL_UART_Main_setParityMode(HOST_UART_INST, VOFA_UART_PARITY_MODE);
    DL_UART_Main_setStopBits(HOST_UART_INST, VOFA_UART_STOP_BITS_MODE);
    DL_UART_Main_enable(HOST_UART_INST);

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
