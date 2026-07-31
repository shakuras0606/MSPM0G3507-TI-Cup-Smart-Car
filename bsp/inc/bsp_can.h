/**
 * @file    bsp_can.h
 * @brief   MSPM0G3507 MCAN0 经典CAN非阻塞发送接口
 *
 * SysConfig负责CANFD0、PA12(CANTX)、PA13(CANRX)、40 MHz CANCLK和
 * 500 kbps位时序。本层只管理一个专用发送缓冲区，不在主循环中等待总线。
 */

#ifndef BSP_CAN_H_
#define BSP_CAN_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t tx_frames;       /**< 成功提交到MCAN发送缓冲区的帧数。 */
    uint32_t busy_drops;      /**< 上一帧仍待发送而主动丢弃的新帧数。 */
    uint32_t driver_errors;   /**< 参数、工作模式或DriverLib请求错误数。 */
    bool ready;               /**< MCAN当前处于正常工作模式。 */
} BSP_CanStats;

/** 清零软件统计并检查MCAN是否已经进入Normal模式。 */
void BSP_Can_Init(void);

/**
 * @brief 尝试发送一帧11位标准ID经典CAN数据帧。
 * @param standard_id 0x000..0x7FF标准ID。
 * @param data        数据地址；dlc为0时允许为NULL。
 * @param dlc         经典CAN数据长度0..8字节。
 * @return true=已提交发送请求；false=本次未提交且不会阻塞等待。
 */
bool BSP_Can_TrySend(uint16_t standard_id, const uint8_t *data, uint8_t dlc);

/** 读取发送统计，便于CCS Expressions诊断CAN总线。 */
bool BSP_Can_GetStats(BSP_CanStats *stats);

#endif /* BSP_CAN_H_ */
