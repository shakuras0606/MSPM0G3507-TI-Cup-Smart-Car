/**
 * @file    byte_ring.h
 * @brief   ISR 安全的单字节环形缓冲区（FIFO）
 *
 * 用于 UART 接收中断与主循环之间的数据传递。
 * 写入端在 ISR 上下文中运行（ByteRing_PushFromIsr），
 * 读取端在主循环上下文中运行（ByteRing_Pop）。
 *
 * 设计要点：
 *   - 预留一个空位以区分"满"和"空"状态（capacity >= 2）
 *   - head 仅由 ISR 写入，tail 仅由主循环写入，主流免锁
 *   - overflow_count：ISR 写入端递增，主循环只读
 *   - 所有成员标记为 volatile 以抑制编译优化重排
 *
 * 容量限制：capacity 最大为 65535（uint16_t）
 */

#ifndef BYTE_RING_H_
#define BYTE_RING_H_

#include <stdbool.h>
#include <stdint.h>

/** ISR 安全的字节环形缓冲区 */
typedef struct
{
    uint8_t *storage;               /**< 数据存储区指针 */

    uint16_t capacity;              /**< 缓冲区总容量（字节），最大 65535 */

    volatile uint16_t head;         /**< 写入索引（仅 ISR 修改） */

    volatile uint16_t tail;         /**< 读取索引（仅主循环修改） */

    volatile uint32_t overflow_count; /**< 溢出计数器（ISR 递增，主循环只读） */
} ByteRing;

/**
 * @brief 初始化环形缓冲区
 * @param ring     缓冲区结构体指针
 * @param storage  存储空间指针（由调用方分配）
 * @param capacity 存储空间大小（字节），至少为 2
 */
void ByteRing_Init(ByteRing *ring, uint8_t *storage, uint16_t capacity);

/**
 * @brief 从中断上下文压入一个字节
 * @param ring  缓冲区指针
 * @param value 要写入的字节
 * @return true  写入成功
 * @return false 缓冲区已满，数据被丢弃且 overflow_count 递增
 * @warning 仅允许在 ISR 上下文中调用
 */
bool ByteRing_PushFromIsr(ByteRing *ring, uint8_t value);

/**
 * @brief 从主循环上下文弹出一个字节
 * @param ring  缓冲区指针
 * @param value 输出参数，存放弹出的字节
 * @return true  弹出成功
 * @return false 缓冲区为空
 * @warning 仅允许在主循环（非 ISR）上下文中调用
 */
bool ByteRing_Pop(ByteRing *ring, uint8_t *value);

/**
 * @brief 查询缓冲区中当前可读取的字节数
 * @param ring 缓冲区指针
 * @return 可用字节数
 * @note   返回值可能因 ISR 并发写入而略有偏差
 */
uint16_t ByteRing_Count(const ByteRing *ring);

/**
 * @brief 查询自上次调用以来的溢出次数
 * @param ring 缓冲区指针
 * @return 溢出计数值
 */
uint32_t ByteRing_OverflowCount(const ByteRing *ring);

#endif /* BYTE_RING_H_ */
