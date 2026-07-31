/**
 * @file    bsp_can.c
 * @brief   单发送缓冲区的经典CAN非阻塞实现
 */

#include "bsp_can.h"

#include <stddef.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#define BSP_CAN_TX_BUFFER_INDEX    (0u)
#define BSP_CAN_TX_BUFFER_MASK     (1u << BSP_CAN_TX_BUFFER_INDEX)
#define BSP_CAN_STANDARD_ID_MAX    (0x7FFu)
#define BSP_CAN_CLASSIC_MAX_DLC    (8u)

static BSP_CanStats g_can_stats;

void BSP_Can_Init(void)
{
    memset(&g_can_stats, 0, sizeof(g_can_stats));
    g_can_stats.ready =
        (DL_MCAN_getOpMode(VEHICLE_CAN_INST) ==
         DL_MCAN_OPERATION_MODE_NORMAL);
}

bool BSP_Can_TrySend(uint16_t standard_id, const uint8_t *data, uint8_t dlc)
{
    DL_MCAN_TxBufElement message;

    if ((standard_id > BSP_CAN_STANDARD_ID_MAX) ||
        (dlc > BSP_CAN_CLASSIC_MAX_DLC) ||
        ((dlc != 0u) && (data == NULL))) {
        ++g_can_stats.driver_errors;
        return false;
    }

    g_can_stats.ready =
        (DL_MCAN_getOpMode(VEHICLE_CAN_INST) ==
         DL_MCAN_OPERATION_MODE_NORMAL);
    if (!g_can_stats.ready) {
        ++g_can_stats.driver_errors;
        return false;
    }

    /*
     * 一个缓冲区足以发送100 Hz加速度。若上一帧仍待仲裁，本拍直接丢弃，
     * 保证控制板收到的是新数据，而不是在软件中排队的过期加速度。
     */
    if ((DL_MCAN_getTxBufReqPend(VEHICLE_CAN_INST) &
         BSP_CAN_TX_BUFFER_MASK) != 0u) {
        ++g_can_stats.busy_drops;
        return false;
    }

    memset(&message, 0, sizeof(message));
    message.id = ((uint32_t)standard_id) << 18u;
    message.rtr = 0u;       /* 数据帧。 */
    message.xtd = 0u;       /* 11位标准ID。 */
    message.esi = 0u;
    message.dlc = dlc;      /* 经典CAN中DLC 0..8直接等于字节数。 */
    message.brs = 0u;       /* 不使用CAN-FD数据段变速。 */
    message.fdf = 0u;       /* 经典CAN帧。 */
    message.efc = 0u;       /* 不写Tx Event FIFO。 */
    message.mm = 0u;
    if (dlc != 0u) {
        memcpy(message.data, data, dlc);
    }

    DL_MCAN_writeMsgRam(VEHICLE_CAN_INST, DL_MCAN_MEM_TYPE_BUF,
                        BSP_CAN_TX_BUFFER_INDEX, &message);
    if (DL_MCAN_TXBufAddReq(VEHICLE_CAN_INST,
                            BSP_CAN_TX_BUFFER_INDEX) != 0) {
        ++g_can_stats.driver_errors;
        return false;
    }

    ++g_can_stats.tx_frames;
    return true;
}

bool BSP_Can_GetStats(BSP_CanStats *stats)
{
    if (stats == NULL) {
        return false;
    }
    *stats = g_can_stats;
    return true;
}
