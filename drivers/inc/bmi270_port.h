/**
 * @file    bmi270_port.h
 * @brief   Bosch BMI270 六轴 IMU 板级传输层（软件 SPI）
 *
 * 提供与 Bosch 官方 BMI270 SensorAPI 兼容的底层 SPI 传输接口。
 *
 * 当前 CONFIG_BMI270_ENABLED=0，因为 PA8/PA9 已分配给 WT61 UART1。
 * 以下 API 和软件 SPI 实现保留，重新规划引脚后可恢复使用。
 *
 * 关键设计要点：
 *   - SPI 读取时需要额外的 dummy 字节（BMI270 协议要求）
 *   - 上电后 CS 上升沿后需丢弃一次 CHIP_ID 读取（Bosch 推荐）
 *   - BoschRead/BoschWrite/BoschDelayUs 三个回调函数完全符合 SensorAPI
 *     的 bmi2_dev.intf_ptr 接口规范
 *
 * 完整的传感器功能配置（滤波器、量程、ODR 等）需要使用 Bosch 提供的
 * BMI270 配置固件 blob，不属于本板级驱动层的职责范围。
 */

#ifndef BMI270_PORT_H_
#define BMI270_PORT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** BMI270 CHIP_ID 寄存器地址 (0x00) */
#define BMI270_CHIP_ID_REGISTER (0x00u)

/** BMI270 预期的 CHIP_ID 值 (0x24) */
#define BMI270_EXPECTED_CHIP_ID (0x24u)

/**
 * @brief 初始化 BMI270 SPI 接口
 *
 * 配置 GPIO 引脚方向，执行上电后的初始化序列：
 *   - SCLK/MOSI 拉低，CS 拉高
 *   - 执行一次丢弃的 CHIP_ID 读取（满足 Bosch 上电要求）
 */
void BMI270_Port_Init(void);

/**
 * @brief 探测 BMI270 传感器是否在线上
 * @param chip_id 输出参数，读取到的 CHIP_ID 值（可为 NULL）
 * @return true  CHIP_ID 与预期值 (0x24) 匹配，传感器正常工作
 * @return false 传感器未响应或 CHIP_ID 不匹配
 */
bool BMI270_Port_Probe(uint8_t *chip_id);

/**
 * @brief 从 BMI270 寄存器连续读取数据
 * @param register_address 起始寄存器地址（bit[7]=1 即读模式标识）
 * @param data             输出缓冲区
 * @param length            读取字节数
 * @return true  读取成功
 * @return false 参数无效
 *
 * 读序列：CS 低 -> 发送(reg|0x80) -> 发送 dummy 0x00 -> 读取 length 字节 -> CS 高
 */
bool BMI270_Port_ReadRegisters(uint8_t register_address, uint8_t *data,
                               size_t length);

/**
 * @brief 向 BMI270 寄存器连续写入数据
 * @param register_address 起始寄存器地址（bit[7]=0 即写模式标识）
 * @param data             待写入的数据
 * @param length            写入字节数
 * @return true  写入成功
 * @return false 参数无效
 *
 * 写序列：CS 低 -> 发送(reg&0x7F) -> 发送 length 字节 -> CS 高
 */
bool BMI270_Port_WriteRegisters(uint8_t register_address,
                                const uint8_t *data, size_t length);

/**
 * @brief 读取 INT1 引脚电平
 * @return 0 或 1
 */
uint8_t BMI270_Port_ReadInt1(void);

/**
 * @brief 读取 INT2 引脚电平
 * @return 0 或 1
 */
uint8_t BMI270_Port_ReadInt2(void);

/* ============ Bosch SensorAPI 兼容回调函数 ============ */

/**
 * @brief Bosch SensorAPI 读取回调（符合 bmi2_dev.read 签名）
 *
 * 对 BMI270_Port_ReadRegisters() 的薄封装，增加返回值映射
 *
 * @param register_address  寄存器地址
 * @param data              输出缓冲区
 * @param length            读取长度
 * @param interface_pointer 接口指针（本实现忽略）
 * @return 0 成功, -1 失败
 */
int8_t BMI270_Port_BoschRead(uint8_t register_address, uint8_t *data,
                             uint32_t length, void *interface_pointer);

/**
 * @brief Bosch SensorAPI 写入回调（符合 bmi2_dev.write 签名）
 *
 * 对 BMI270_Port_WriteRegisters() 的薄封装，增加返回值映射
 *
 * @param register_address  寄存器地址
 * @param data              待写入数据
 * @param length            写入长度
 * @param interface_pointer 接口指针（本实现忽略）
 * @return 0 成功, -1 失败
 */
int8_t BMI270_Port_BoschWrite(uint8_t register_address, const uint8_t *data,
                              uint32_t length, void *interface_pointer);

/**
 * @brief Bosch SensorAPI 微秒延迟回调（符合 bmi2_dev.delay_us 签名）
 * @param period_us         延迟微秒数
 * @param interface_pointer 接口指针（本实现忽略）
 */
void BMI270_Port_BoschDelayUs(uint32_t period_us,
                              void *interface_pointer);

#endif /* BMI270_PORT_H_ */
