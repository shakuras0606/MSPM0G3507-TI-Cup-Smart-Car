/**
 * @file    line_sensor16.h
 * @brief   16 通道线传感器数据处理 API
 *
 * 处理 16 通道线阵传感器的数据，支持数字量（二值化）和模拟量（灰度值）输入。
 * 核心计算：加权质心法 (weighted centroid) 确定黑线在传感器阵列中的位置。
 *
 * 数据来源：
 *   - 数字量模式：外部传感器模块输出 16 位二值化掩码
 *   - 模拟量模式：外部 ADC 或传感器模块输出 16 通道 16 位灰度值
 *
 * 位置计算（加权质心法）：
 *   position = sum(channel_pos[i] * value[i]) / sum(value[i])
 *   其中 channel_pos[i] 在 -1000..+1000 范围内线性分布
 *
 * 诊断功能：
 *   - 统计活跃通道数 (active_count)
 *   - 记录峰值通道强度 (peak_strength)
 *   - 生成诊断掩码（强度 >= 峰值/2 的通道）
 *   - 线丢失检测 (line_lost)
 */

#ifndef LINE_SENSOR16_H_
#define LINE_SENSOR16_H_

#include <stdbool.h>
#include <stdint.h>

/** 线传感器通道数：16 */
#define LINE_SENSOR16_CHANNEL_COUNT (16u)

/** 线传感器数据来源枚举 */
typedef enum
{
    LINE_SENSOR16_SOURCE_NONE = 0,  /**< 未收到数据 */

    LINE_SENSOR16_SOURCE_DIGITAL,   /**< 数字量输入（二值化掩码） */

    LINE_SENSOR16_SOURCE_ANALOG     /**< 模拟量输入（灰度值） */
} LineSensor16Source;

/** 线传感器完整数据集 */
typedef struct
{
    uint16_t raw_mask;                              /**< 原始数字掩码 / 模拟量诊断掩码 */

    uint16_t values[LINE_SENSOR16_CHANNEL_COUNT];   /**< 各通道强度值（0=无信号, 65535=饱和） */

    int16_t position;                               /**< 加权质心位置 (-1000=最左, +1000=最右) */

    uint16_t total_strength;                        /**< 所有通道强度之和 */

    uint16_t peak_strength;                         /**< 最强通道的强度值 */

    uint8_t active_count;                           /**< 活跃通道计数（数字模式）或 >= 阈值通道数（模拟模式） */

    bool line_lost;                                 /**< true = 无任何通道检测到信号 */

    LineSensor16Source source;                      /**< 数据来源类型 */
} LineSensor16Data;

/**
 * @brief 初始化线传感器数据结构
 *
 * 将所有字段清零，设置 line_lost = true，source = NONE
 */
void LineSensor16_Init(void);

/**
 * @brief 使用数字量（二值化掩码）更新线传感器数据
 * @param raw_mask   16 位二值化掩码（bit[0]=通道 0）
 * @param active_low true 表示掩码中 0 为有效（低电平有效），false 表示 1 为有效
 *
 * 计算加权质心位置，无有效通道时 line_lost 置为 true
 */
void LineSensor16_UpdateDigital(uint16_t raw_mask, bool active_low);

/**
 * @brief 使用模拟量（灰度值）更新线传感器数据
 * @param values 16 个通道的灰度值数组（0=最暗, 65535=最亮）
 *
 * 使用全精度加权质心法计算位置，同时生成诊断掩码（强度 >= 峰值/2）
 */
void LineSensor16_UpdateAnalog(
    const uint16_t values[LINE_SENSOR16_CHANNEL_COUNT]);

/**
 * @brief 获取最新的线传感器数据（只读）
 * @return 指向内部数据结构的常量指针
 */
const LineSensor16Data *LineSensor16_GetData(void);

#endif /* LINE_SENSOR16_H_ */
