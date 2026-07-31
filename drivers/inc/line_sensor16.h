/**
 * @file    line_sensor16.h
 * @brief   Hiwonder 8 路红外巡线模块 UART/DMA 驱动兼容接口
 *
 * 文件名与 LineSensor16_* API 暂时保留，是为了让已经调好的巡线位置环、
 * Yaw 环和速度环不需要改动。实际物理通道只有 S1~S8：
 *   - UART2：PA21(TX) / PA22(RX)，115200-8-N-1
 *   - UART 手动通信模式：只发送命令 2 读取 8 路 16 位原始模拟量
 *   - MSPM0 使用 project_config.h 中的阈值自行生成二值状态
 *   - 位置可选择原二值等权平均或原始ADC直接加权平均
 *   - bit0=S1，bit7=S8；S1 默认作为最左通道
 *
 * values[] 仍保留 16 项以兼容现有屏幕函数；只有 values[0..7] 有效，
 * values[8..15] 始终为 0。
 */

#ifndef LINE_SENSOR16_H_
#define LINE_SENSOR16_H_

#include <stdbool.h>
#include <stdint.h>

/** 兼容现有上层数组尺寸。 */
#define LINE_SENSOR16_CHANNEL_COUNT       (16u)

/** 新模块实际物理通道数。 */
#define LINE_SENSOR_PHYSICAL_CHANNEL_COUNT (8u)

typedef enum
{
    LINE_SENSOR16_SOURCE_NONE = 0,
    LINE_SENSOR16_SOURCE_DIGITAL,
    LINE_SENSOR16_SOURCE_ANALOG,
    LINE_SENSOR16_SOURCE_UART
} LineSensor16Source;

typedef struct
{
    uint16_t raw_mask;    /**< MSPM0 对模拟量阈值化后的低 8 位；bit0=S1。 */
    uint16_t active_mask; /**< 与 raw_mask 相同：1 表示该通道判定为黑线。 */
    uint16_t values[LINE_SENSOR16_CHANNEL_COUNT]; /**< S1~S8 模拟值；后 8 项为 0。 */
    int16_t position;     /**< 当前所选算法的黑线位置，左负右正。 */
    uint16_t total_strength; /**< 模拟模式为8路ADC和(超限饱和)；数字模式为黑线通道数。 */
    uint16_t peak_strength;  /**< 模拟模式为最大单路ADC；数字模式为0或1。 */
    uint8_t active_count; /**< 当前判为黑线的物理通道数，范围 0..8。 */
    bool line_lost;
    bool online;          /**< 最近一次状态字节未超过串口离线时间。 */
    uint32_t last_update_ms;
    uint32_t state_frames;  /**< MSPM0 已完成阈值化的位置样本数。 */
    uint32_t analog_frames; /**< 校验通过的 UART 模拟量帧数。 */
    uint32_t protocol_errors;
    uint32_t rx_overflows;
    LineSensor16Source source;
} LineSensor16Data;

/**
 * 初始化 UART2 RX/TX DMA，并发送 UART“手动通信模式”配置字节。
 * 这里的“手动”仅表示由主控发命令读取，不是模块的灰度学习模式。
 */
void LineSensor16_Init(void);

/**
 * @brief 非阻塞处理 UART 数据并按设定频率发起下一次读取。
 *
 * 保留 Scan 名称以兼容 app.c。函数不再扫描旧 16 路模拟复用器。
 */
void LineSensor16_Scan(void);

/** 使用低 8 位二值掩码更新位置；保留给单元测试和兼容调用。 */
void LineSensor16_UpdateDigital(uint16_t raw_mask, bool active_low);

/**
 * 使用 values[0..7] 的原始ADC直接计算加权质心。
 * 模拟位置计算不扣除判线阈值，也不使用单路限幅。
 */
void LineSensor16_UpdateAnalog(
    const uint16_t values[LINE_SENSOR16_CHANNEL_COUNT]);

/** 获取最新只读快照。 */
const LineSensor16Data *LineSensor16_GetData(void);

#endif /* LINE_SENSOR16_H_ */
