/**
 * @file    byte_ring.c
 * @brief   ISR 安全的单字节环形缓冲区（FIFO）实现
 *
 * 本模块是为中断与主循环之间的数据传递设计的轻量级环形缓冲区。
 * 核心设计理念：
 *
 *   1. 无锁并发 (lock-free concurrency)
 *      主循环只写 tail 索引（弹出端），ISR 只写 head 索引（压入端），
 *      两个写入者互不冲突，无需关中断。
 *
 *   2. 预留空位法 (one-slot gap)
 *      始终保留一个空位，以区分"满"和"空"两种状态：
 *        - 空：head == tail
 *        - 满：(head + 1) % capacity == tail
 *      因此缓冲区实际可存储 capacity - 1 个字节。
 *
 *   3. 溢出监控
 *      当 ISR 尝试写入满缓冲区时，数据被丢弃但 overflow_count 递增，
 *      主循环可通过 ByteRing_OverflowCount() 查询是否存在数据丢失。
 *
 * 限制：
 *   - capacity 最大为 65535（uint16_t 范围）
 *   - 不支持多生产者/多消费者场景
 *   - Count() 在 ISR 并发时返回值可能略有偏差
 */

#include "byte_ring.h"

#include <stddef.h>

void ByteRing_Init(ByteRing *ring, uint8_t *storage, uint16_t capacity)
{
    if (ring == NULL) {
        return;
    }
    ring->storage = storage;
    ring->capacity = capacity;

    /* 重置索引和溢出计数 */
    ring->head = 0u;
    ring->tail = 0u;
    ring->overflow_count = 0u;
}

bool ByteRing_PushFromIsr(ByteRing *ring, uint8_t value)
{
    uint16_t next;

    /*
     * 有效性检查：
     *   - ring 非空
     *   - storage 非空（已初始化）
     *   - capacity >= 2（至少需要 1 个数据位 + 1 个空位）
     */
    if ((ring == NULL) || (ring->storage == NULL) || (ring->capacity < 2u)) {
        return false;
    }

    /* 计算 head 的下一个位置（带环绕） */
    next = (uint16_t)(ring->head + 1u);
    if (next >= ring->capacity) {
        next = 0u;
    }

    /*
     * 满判断：如果 head 的下一个位置追上 tail，
     * 说明所有可用槽位已满（预留空位法）。
     * 丢弃本次数据并递增溢出计数器。
     */
    if (next == ring->tail) {
        ++ring->overflow_count;
        return false;
    }

    /* 写入数据并推进 head */
    ring->storage[ring->head] = value;
    ring->head = next;
    return true;
}

bool ByteRing_Pop(ByteRing *ring, uint8_t *value)
{
    uint16_t next;

    /*
     * 有效性检查：
     *   - ring 非空
     *   - value 非空（输出参数有效）
     *   - 缓冲区不空（head != tail）
     */
    if ((ring == NULL) || (value == NULL) || (ring->head == ring->tail)) {
        return false;
    }

    /* 从 tail 位置读取数据 */
    *value = ring->storage[ring->tail];

    /* 计算 tail 的下一个位置（带环绕） */
    next = (uint16_t)(ring->tail + 1u);
    if (next >= ring->capacity) {
        next = 0u;
    }
    ring->tail = next;
    return true;
}

uint16_t ByteRing_Count(const ByteRing *ring)
{
    uint16_t head;
    uint16_t tail;

    if (ring == NULL) {
        return 0u;
    }

    /*
     * 先快照 head 和 tail（ISR 可能在读取过程中修改 head）
     *
     * 如果 head >= tail：线性区域，直接相减
     *   例如 head=10, tail=3 -> count = 7
     * 如果 head < tail：发生了环绕
     *   例如 head=2, tail=8, capacity=10 -> count = 10 - 8 + 2 = 4
     */
    head = ring->head;
    tail = ring->tail;
    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(ring->capacity - tail + head);
}

uint32_t ByteRing_OverflowCount(const ByteRing *ring)
{
    return (ring == NULL) ? 0u : ring->overflow_count;
}
