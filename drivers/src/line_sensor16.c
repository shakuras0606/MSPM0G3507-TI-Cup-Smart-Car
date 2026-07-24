/**
 * @file    line_sensor16.c
 * @brief   16 通道线传感器数据处理实现
 *
 * 本模块是线传感器数据处理的核心，负责将从传感器接收到的原始数据
 * 转换为赛车控制算法所需的高层信息：黑线位置、线状态、诊断信息等。
 *
 * 核心算法：加权质心法 (Weighted Centroid)
 *
 * 将 16 个传感器通道视为一维空间中的离散采样点，
 * 每个通道在空间中的坐标 channel_pos[i] 在 -1000..+1000 间线性分布：
 *
 *   channel_pos[0]  = -1000   (最左侧)
 *   channel_pos[15] = +1000   (最右侧)
 *   相邻通道间距 = 2000 / 15 ≈ 133.33
 *
 * 加权质心位置 position = sum(channel_pos[i] * value[i]) / sum(value[i])
 *
 *   其中 value[i] 在数字量模式下为 0 或 1，在模拟量模式下为灰度值（0..65535）。
 *   如果所有 value[i] 均为 0（完全无信号），则 line_lost = true。
 *
 * 诊断功能：
 *   - 活跃通道计数 (active_count)
 *   - 诊断掩码（模拟模式下强度 >= 峰值一半的通道）
 *   - 峰值/总强度统计
 */

#include "line_sensor16.h"

#include <stddef.h>
#include <string.h>

/** 模块内部状态变量（全局单例） */
static LineSensor16Data g_data;

/**
 * @brief 计算通道 i 在一维空间中的位置坐标
 * @param channel 通道索引 (0..15)
 * @return 坐标值 (-1000..+1000)
 *
 * 16 个通道均匀覆盖 -1000 到 +1000 的范围，
 * 步长为 2000 / 15 = 133（整数除法）
 */
static int16_t channel_position(uint8_t channel)
{
    /*
     * 插值公式：
     *   position = -1000 + channel * 2000 / 15
     *
     * 使用 int32_t 中间计算以避免 channel * 2000 溢出
     * (uint8_t max 255 * 2000 = 510000，在 int32_t 范围内安全)
     */
    return (int16_t)(-1000 + ((int32_t)channel * 2000) / 15);
}

void LineSensor16_Init(void)
{
    memset(&g_data, 0, sizeof(g_data));
    g_data.line_lost = true;                    /* 初始状态：线丢失 */
    g_data.source = LINE_SENSOR16_SOURCE_NONE;   /* 数据来源：未收到 */
}

void LineSensor16_UpdateDigital(uint16_t raw_mask, bool active_low)
{
    /*
     * active_low 处理：
     *   active_low = true  -> 掩码中 0 为有效（低电平有效），取反
     *   active_low = false -> 掩码中 1 为有效（高电平有效），不变
     */
    uint16_t active_mask = active_low ? (uint16_t)~raw_mask : raw_mask;
    int32_t weighted_sum = 0;
    uint8_t index;

    /* 清除旧的模拟数据，准备写入新的数字量数据 */
    memset(g_data.values, 0, sizeof(g_data.values));
    g_data.raw_mask = raw_mask;
    g_data.active_count = 0u;
    g_data.peak_strength = 0u;

    /* 遍历 16 个通道，对每个有效通道累加加权位置 */
    for (index = 0u; index < LINE_SENSOR16_CHANNEL_COUNT; ++index) {
        if ((active_mask & ((uint16_t)1u << index)) != 0u) {
            g_data.values[index] = 1u;                    /* 标记为有效 */
            weighted_sum += channel_position(index);       /* 累加加权位置 */
            ++g_data.active_count;                         /* 计数 */
        }
    }

    /*
     * 数字模式下，每个有效通道的值视为 1，因此：
     *   total_strength = active_count
     *   peak_strength  = 1 (有通道有效) 或 0 (无通道有效)
     */
    g_data.total_strength = g_data.active_count;
    g_data.peak_strength = (g_data.active_count == 0u) ? 0u : 1u;
    g_data.line_lost = (g_data.active_count == 0u);

    if (!g_data.line_lost) {
        /*
         * 加权平均：position = sum(channel_pos[i] * 1) / active_count
         * 使用 int32_t 中间计算避免溢出
         */
        g_data.position =
            (int16_t)(weighted_sum / (int32_t)g_data.active_count);
    }
    g_data.source = LINE_SENSOR16_SOURCE_DIGITAL;
}

void LineSensor16_UpdateAnalog(
    const uint16_t values[LINE_SENSOR16_CHANNEL_COUNT])
{
    uint32_t total = 0u;
    int64_t weighted_sum = 0;    /* 使用 int64_t 避免大值求和溢出 */
    uint16_t peak = 0u;
    uint16_t mask = 0u;          /* 诊断掩码 */
    uint8_t index;

    if (values == NULL) {
        return;
    }

    /*
     * 第一遍扫描：计算总和、加权和、峰值
     *
     * weighted_sum 可能非常大：
     *   最大 = 16 * 1000 * 65535 = 1,048,560,000
     *   超出 int32_t 范围 (2,147,483,647 内，安全)
     *   但使用 int64_t 作为预防措施
     */
    for (index = 0u; index < LINE_SENSOR16_CHANNEL_COUNT; ++index) {
        uint16_t value = values[index];
        g_data.values[index] = value;
        total += value;
        weighted_sum += (int32_t)channel_position(index) * value;
        if (value > peak) {
            peak = value;
        }
    }

    /*
     * 第二遍扫描（仅当有信号时）：生成诊断掩码
     *
     * 诊断掩码用于标示哪些通道"有信号"（强度 >= 峰值的一半）。
     * 这有助于调试和可视化，不影响位置计算（位置始终使用全精度）。
     *
     * 阈值 = (peak + 1) / 2：向上取整，避免小峰值时全部通道被排除
     */
    g_data.active_count = 0u;
    if (peak != 0u) {
        uint16_t threshold = (uint16_t)((peak + 1u) / 2u);
        for (index = 0u; index < LINE_SENSOR16_CHANNEL_COUNT; ++index) {
            if (g_data.values[index] >= threshold) {
                mask |= (uint16_t)1u << index;
                ++g_data.active_count;
            }
        }
    }

    /* 填充输出数据结构 */
    g_data.raw_mask = mask;
    g_data.total_strength =
        (total > UINT16_MAX) ? UINT16_MAX : (uint16_t)total;
    g_data.peak_strength = peak;
    g_data.line_lost = (total == 0u);

    if (!g_data.line_lost) {
        /*
         * 全精度加权平均位置：
         *   position = sum(channel_pos[i] * value[i]) / sum(value[i])
         *
         * 使用 int64_t 除法以确保精度
         */
        g_data.position = (int16_t)(weighted_sum / (int64_t)total);
    }
    g_data.source = LINE_SENSOR16_SOURCE_ANALOG;
}

const LineSensor16Data *LineSensor16_GetData(void)
{
    return &g_data;
}
