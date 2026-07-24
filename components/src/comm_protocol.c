/**
 * @file    comm_protocol.c
 * @brief   AA 55 二进制帧协议实现
 *
 * 实现基于以下核心组件：
 *   1. CRC16-CCITT 校验码计算器（多项式 0x1021）
 *   2. 流式帧解析器状态机（逐字节喂入，无阻塞）
 *   3. 帧编码器（将结构体序列化为可发送的字节流）
 *
 * 帧格式 (共 6 + LENGTH 字节)：
 * @code
 *   SYNC | TYPE | LENGTH | PAYLOAD | CRC_LO CRC_HI
 *   2 字节 | 1 字节 | 1 字节 | LENGTH 字节 | 2 字节 (小端)
 * @endcode
 *
 * 解析器状态机转换图：
 * @code
 *   SYNC_0 --[0xAA]--> SYNC_1 --[0x55]--> TYPE --> LENGTH
 *                              |                         |
 *                              +--[0xAA]--+              |
 *                              |           |              |
 *                              +--[other]-+              |
 *                              v                          |
 *                             SYNC_0 <----+               |
 *                                                         |
 *   CRC_HIGH <-- CRC_LOW <-- PAYLOAD <--[len>0]-+         |
 *      |                                            |     |
 *      +--[CRC OK]--> 输出帧                        |     |
 *      +--[CRC FAIL]--> SYNC_0                     +-[len=0]-+
 * @endcode
 *
 * CRC16 参数：
 *   - 多项式:  0x1021 (x^16 + x^12 + x^5 + 1)
 *   - 初始值:  0xFFFF
 *   - 输入/输出数据不反转 (no reflect)
 *   - 结果异或值: 0x0000
 *   - 校验范围: TYPE + LENGTH + PAYLOAD（不包含同步头）
 */

#include "comm_protocol.h"

#include <stddef.h>
#include <string.h>

/* ============ CRC16-CCITT 计算 ============ */

/**
 * @brief 逐字节更新 CRC16-CCITT 校验值
 * @param crc   当前 CRC 值（首次调用传 0xFFFF）
 * @param value 新输入的字节
 * @return 更新后的 CRC 值
 *
 * 算法：按位查表法的软件等价实现
 *   - 将输入字节与 CRC 高字节异或
 *   - 对每一位检查 MSB，若为 1 则左移后异或多项式
 *   - 迭代 8 次
 *
 * 在本固件的 CPU 频率下，软件 CRC 计算耗时远低于 1ms（~2us/字节），
 * 不影响主循环实时性，因此不引入查表法增加代码空间。
 */
static uint16_t crc_update(uint16_t crc, uint8_t value)
{
    uint8_t bit;

    crc ^= (uint16_t)value << 8;
    for (bit = 0u; bit < 8u; ++bit) {
        crc = ((crc & 0x8000u) != 0u) ?
              (uint16_t)((crc << 1) ^ 0x1021u) :
              (uint16_t)(crc << 1);
    }
    return crc;
}

/* ============ 解析器状态管理 ============ */

/**
 * @brief 重置解析器到初始状态（等待同步头）
 * @param parser 解析器实例指针
 *
 * 在以下情况调用：
 *   - 完整帧接收完成（无论校验成功或失败）
 *   - LENGTH 字段超出最大允许值
 *   - 遇到未知解析状态
 */
static void parser_reset(CommParser *parser)
{
    parser->state = COMM_PARSE_SYNC_0;
    parser->payload_index = 0u;
    parser->calculated_crc = 0xFFFFu;
    parser->received_crc = 0u;
}

/* ============ 公共接口 ============ */

void CommProtocol_Init(CommParser *parser)
{
    if (parser == NULL) {
        return;
    }
    /* 先将整个结构体清零（包括 valid_frames / rejected_frames 计数器） */
    memset(parser, 0, sizeof(*parser));
    parser_reset(parser);
}

bool CommProtocol_Feed(CommParser *parser, uint8_t byte, CommFrame *frame)
{
    if ((parser == NULL) || (frame == NULL)) {
        return false;
    }

    switch (parser->state) {

    case COMM_PARSE_SYNC_0:
        /*
         * 等待同步字节 0 (0xAA)。
         * 只有收到正确的 0xAA 才进入下一状态，否则保持静默。
         * 这样即使数据流中混入噪声也能快速恢复同步。
         */
        if (byte == COMM_SYNC_0) {
            parser->state = COMM_PARSE_SYNC_1;
        }
        break;

    case COMM_PARSE_SYNC_1:
        /*
         * 等待同步字节 1 (0x55)。
         * 如果收到 0x55，进入 TYPE 状态。
         * 如果收到 0xAA（可能是前一帧的尾巴 + 下一帧开头重叠），重新回到 SYNC_1。
         * 任何其他字节回到 SYNC_0 重新开始。
         */
        parser->state = (byte == COMM_SYNC_1) ? COMM_PARSE_TYPE :
                        ((byte == COMM_SYNC_0) ? COMM_PARSE_SYNC_1 :
                         COMM_PARSE_SYNC_0);
        break;

    case COMM_PARSE_TYPE:
        /* 接收 TYPE 字节并初始化 CRC 计算 */
        parser->frame.type = byte;
        parser->calculated_crc = crc_update(0xFFFFu, byte);
        parser->state = COMM_PARSE_LENGTH;
        break;

    case COMM_PARSE_LENGTH:
        /*
         * 接收并验证 LENGTH 字段：
         *   - 更新 CRC
         *   - 如果长度超标 -> 丢弃此帧（rejected_frames++）并重置解析器
         *   - 如果长度为 0 -> 跳过 PAYLOAD，直接等待 CRC
         */
        parser->frame.length = byte;
        parser->calculated_crc = crc_update(parser->calculated_crc, byte);
        parser->payload_index = 0u;
        if (byte > CONFIG_COMM_MAX_PAYLOAD) {
            ++parser->rejected_frames;
            parser_reset(parser);
        } else {
            parser->state = (byte == 0u) ? COMM_PARSE_CRC_LOW :
                            COMM_PARSE_PAYLOAD;
        }
        break;

    case COMM_PARSE_PAYLOAD:
        /* 逐字节接收 PAYLOAD 并更新 CRC */
        parser->frame.payload[parser->payload_index++] = byte;
        parser->calculated_crc = crc_update(parser->calculated_crc, byte);
        if (parser->payload_index >= parser->frame.length) {
            parser->state = COMM_PARSE_CRC_LOW;
        }
        break;

    case COMM_PARSE_CRC_LOW:
        /* 接收 CRC 低字节 */
        parser->received_crc = byte;
        parser->state = COMM_PARSE_CRC_HIGH;
        break;

    case COMM_PARSE_CRC_HIGH:
        /*
         * 接收 CRC 高字节，完成一帧的解析。
         * 将 received_crc 与 calculated_crc 比对：
         *   - 匹配 -> 输出 frame，valid_frames++，重置解析器
         *   - 不匹配 -> 丢弃数据，rejected_frames++，重置解析器
         *
         * 注意：即使校验失败也重置解析器返回 SYNC_0，
         * 确保不从错误状态恢复。下一组 0xAA 0x55 同步头
         * 会重新触发同步。
         */
        parser->received_crc |= (uint16_t)byte << 8;
        if (parser->received_crc == parser->calculated_crc) {
            *frame = parser->frame;
            ++parser->valid_frames;
            parser_reset(parser);
            return true;
        }
        /* CRC 不匹配：计数后丢弃 */
        ++parser->rejected_frames;
        parser_reset(parser);
        break;

    default:
        /* 防御性编程：未知状态一律重置 */
        parser_reset(parser);
        break;
    }
    return false;
}

uint16_t CommProtocol_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;

    if (data == NULL) {
        return crc;
    }
    /* 逐字节更新 CRC */
    while (length-- != 0u) {
        crc = crc_update(crc, *data++);
    }
    return crc;
}

size_t CommProtocol_Encode(const CommFrame *frame, uint8_t *output,
                           size_t capacity)
{
    size_t total;
    uint16_t crc;

    /*
     * 参数验证：
     *   - frame/output 非空
     *   - frame->length 不超过协议最大载荷
     *   - 输出缓冲区足够容纳完整帧
     */
    if ((frame == NULL) || (output == NULL) ||
        (frame->length > CONFIG_COMM_MAX_PAYLOAD)) {
        return 0u;
    }
    total = (size_t)frame->length + 6u; /* 帧总长度 = 6 + LENGTH */
    if (capacity < total) {
        return 0u;
    }

    /* 组帧：同步头 (0xAA 0x55) + TYPE + LENGTH + PAYLOAD + CRC16 */
    output[0] = COMM_SYNC_0;
    output[1] = COMM_SYNC_1;
    output[2] = frame->type;
    output[3] = frame->length;
    if (frame->length != 0u) {
        memcpy(&output[4], frame->payload, frame->length);
    }

    /*
     * CRC 校验范围：从 TYPE 字节到 PAYLOAD 最后一个字节
     * (output[2] ~ output[4 + frame->length - 1])
     * 同步头 (0xAA 0x55) 不参与 CRC 计算
     */
    crc = CommProtocol_Crc16(&output[2], (size_t)frame->length + 2u);

    /* 小端序写入 CRC 值 */
    output[4u + frame->length] = (uint8_t)crc;          /* CRC 低字节 */
    output[5u + frame->length] = (uint8_t)(crc >> 8);   /* CRC 高字节 */
    return total;
}
