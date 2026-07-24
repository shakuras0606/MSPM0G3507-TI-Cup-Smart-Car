/**
 * @file    comm_protocol.h
 * @brief   AA 55 二进制帧协议：解析器 + CRC16 校验 + 帧编码
 *
 * 帧格式（字节序：小端）：
 * @code
 *   | SYNC_0 | SYNC_1 | TYPE | LENGTH | PAYLOAD ... | CRC_LO | CRC_HI |
 *   |  0xAA  |  0x55  |  1B  |   1B   |  LENGTH B   |    2B (LE)      |
 * @endcode
 *
 * SYNC_0 / SYNC_1：帧同步头，0xAA 0x55 组合在直流分量和时钟恢复方面平衡
 * TYPE：    帧类型/命令码
 * LENGTH：  PAYLOAD 字节数（0..CONFIG_COMM_MAX_PAYLOAD）
 * PAYLOAD： 有效载荷（可选）
 * CRC_LO / CRC_HI：CRC16-CCITT 校验值，覆盖 TYPE + LENGTH + PAYLOAD
 *
 * CRC 参数：
 *   - 多项式：  0x1021 (CRC-16-CCITT)
 *   - 初始值：  0xFFFF
 *   - 输入/输出不反转
 *
 * 解析器状态机按字节流式处理，每个字节喂入 CommProtocol_Feed()，
 * 当完整帧接收并校验通过后返回帧结构体。
 */

#ifndef COMM_PROTOCOL_H_
#define COMM_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "project_config.h"

/** 帧同步字节 0 */
#define COMM_SYNC_0 (0xAAu)

/** 帧同步字节 1 */
#define COMM_SYNC_1 (0x55u)

/** 通信帧结构体（解析输出 / 编码输入） */
typedef struct
{
    uint8_t type;                               /**< 帧类型/命令码 */

    uint8_t length;                             /**< 有效载荷字节数 */

    uint8_t payload[CONFIG_COMM_MAX_PAYLOAD];   /**< 有效载荷数据 */
} CommFrame;

/** 帧解析器状态枚举 */
typedef enum
{
    COMM_PARSE_SYNC_0 = 0,  /**< 等待同步字节 0 (0xAA) */

    COMM_PARSE_SYNC_1,      /**< 等待同步字节 1 (0x55) */

    COMM_PARSE_TYPE,        /**< 接收 TYPE 字节 */

    COMM_PARSE_LENGTH,      /**< 接收 LENGTH 字节，验证长度合法性 */

    COMM_PARSE_PAYLOAD,     /**< 逐字节接收 PAYLOAD */

    COMM_PARSE_CRC_LOW,     /**< 接收 CRC 低字节 */

    COMM_PARSE_CRC_HIGH     /**< 接收 CRC 高字节，校验完成后输出帧 */
} CommParseState;

/** 帧解析器实例 */
typedef struct
{
    CommParseState state;       /**< 当前解析状态 */

    CommFrame frame;            /**< 正在组装的帧缓冲区 */

    uint8_t payload_index;      /**< 当前 PAYLOAD 写入位置 */

    uint16_t calculated_crc;    /**< 正在计算的 CRC 值 */

    uint16_t received_crc;      /**< 接收到的 CRC 值（用于比对） */

    uint32_t valid_frames;      /**< 成功解析的有效帧计数 */

    uint32_t rejected_frames;   /**< 被拒绝的无效帧计数 */
} CommParser;

/**
 * @brief 初始化帧解析器
 * @param parser 解析器实例指针
 */
void CommProtocol_Init(CommParser *parser);

/**
 * @brief 向解析器喂入一个字节（流式解析）
 * @param parser 解析器实例指针
 * @param byte   输入的字节
 * @param frame  输出参数，解析完成的帧（仅当返回 true 时有效）
 * @return true  完成一帧的解析（CRC 校验通过）
 * @return false 尚未完成或帧无效
 *
 * 调用方应在主循环中从 UART 环形缓冲区逐字节读取并喂入此函数。
 * 当返回 true 时，frame 中的 type/length/payload 已经过 CRC 验证。
 */
bool CommProtocol_Feed(CommParser *parser, uint8_t byte, CommFrame *frame);

/**
 * @brief 计算 CRC16-CCITT 校验值
 * @param data   数据指针
 * @param length 数据长度（字节）
 * @return 16 位 CRC 值
 *
 * 可用于独立的 CRC 计算（不依赖解析器），例如编码帧时。
 */
uint16_t CommProtocol_Crc16(const uint8_t *data, size_t length);

/**
 * @brief 将帧结构体编码为可发送的字节流
 * @param frame    待编码的帧（TYPE + LENGTH + PAYLOAD）
 * @param output   输出缓冲区
 * @param capacity 输出缓冲区大小
 * @return 编码后的字节数，0 表示参数错误或缓冲区不足
 *
 * 编码时自动计算并附加 CRC16 校验值。帧总长度 = 6 + LENGTH 字节。
 */
size_t CommProtocol_Encode(const CommFrame *frame, uint8_t *output,
                           size_t capacity);

#endif /* COMM_PROTOCOL_H_ */
