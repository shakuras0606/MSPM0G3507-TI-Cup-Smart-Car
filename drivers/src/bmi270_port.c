/**
 * @file    bmi270_port.c
 * @brief   Bosch BMI270 六轴 IMU 板级传输层实现（软件 SPI）
 *
 * 本模块是 BMI270 传感器与 Bosch 官方 SensorAPI 之间的桥梁层。
 *
 * 接口架构：
 * @code
 *   Bosch SensorAPI (bmi270.c)
 *          |
 *          v
 *   bmi2_dev.intf_ptr { .read = BMI270_Port_BoschRead,
 *                        .write = BMI270_Port_BoschWrite,
 *                        .delay_us = BMI270_Port_BoschDelayUs }
 *          |
 *          v
 *   BMI270_Port_ReadRegisters / BMI270_Port_WriteRegisters
 *          |        (本模块: 软件 SPI GPIO 位操作)
 *          v
 *   BMI270 传感器硬件 (CS=PA8, SCLK=PA12, MISO=PA13, MOSI=PA14)
 * @endcode
 *
 * SPI 协议细节：
 *   - Mode 0: CPOL=0 (空闲低电平), CPHA=0 (第一个边沿采样)
 *   - 4 线 SPI: CS (PA8), SCLK (PA12), MOSI (PA14), MISO (PA13)
 *   - 读取时需要额外发送一个 dummy 字节 (BMI270 协议要求)
 *   - 地址字节 bit[7]: 0 = 写, 1 = 读
 *   - 上电后 CS 上升沿后第一次读取需丢弃（Bosch 推荐做法）
 *
 * 为什么使用软件 SPI 而非硬件 SPI：
 *   - MSPM0G3507 的硬件 SPI0 引脚与 LCD 复用冲突
 *   - 软件 SPI 在 32 MHz CPU 下可达到约 4 MHz 速率，满足 BMI270 最大 SPI 时钟 (10 MHz)
 *   - 引脚分配更灵活，方便与 LCD/电机共用 GPIO 端口
 */

#include "bmi270_port.h"

#include "bsp_time.h"
#include "project_config.h"
#include "ti_msp_dl_config.h"

#if (CONFIG_BMI270_ENABLED != 0u)

/**
 * @brief 软件 SPI 半周期延迟
 *
 * 确保 SCLK 的翻转时序满足 BMI270 的建立/保持时间要求。
 * CONFIG_BMI270_SPI_HALF_CYCLES = 4 意味着每个半周期约 125ns @ 32MHz，
 * 对应 SPI 时钟约 4 MHz。
 */
static void spi_delay(void)
{
    delay_cycles(CONFIG_BMI270_SPI_HALF_CYCLES);
}

/**
 * @brief 软件 SPI 传输一个字节（同时发送和接收）
 * @param output 发送的字节（MSB 先）
 * @return 接收到的字节
 *
 * SPI Mode 0 时序 (CPOL=0, CPHA=0)：
 *   1. 准备 MOSI 数据
 *   2. SCLK 上升沿 -> 从机采样 MOSI，主机采样 MISO
 *   3. SCLK 恢复低电平
 *   4. 移位准备下一位
 */
static uint8_t spi_transfer(uint8_t output)
{
    uint8_t input = 0u;
    uint8_t bit;

    for (bit = 0u; bit < 8u; ++bit) {
        /* 准备 MOSI 数据 */
        if ((output & 0x80u) != 0u) {
            DL_GPIO_setPins(BMI270_IO_PORT, BMI270_IO_IMU_MOSI_PIN);
        } else {
            DL_GPIO_clearPins(BMI270_IO_PORT, BMI270_IO_IMU_MOSI_PIN);
        }
        spi_delay();                                      /* 数据建立时间 */

        DL_GPIO_setPins(BMI270_IO_PORT, BMI270_IO_IMU_SCLK_PIN);
                                                          /* SCLK 上升沿 */
        input <<= 1;                                      /* 左移准备接收位 */
        if ((DL_GPIO_readPins(BMI270_IO_PORT, BMI270_IO_IMU_MISO_PIN) &
             BMI270_IO_IMU_MISO_PIN) != 0u) {
            input |= 1u;                                  /* 采样 MISO */
        }
        spi_delay();                                      /* 数据保持时间 */

        DL_GPIO_clearPins(BMI270_IO_PORT, BMI270_IO_IMU_SCLK_PIN);
                                                          /* SCLK 恢复低 */
        output <<= 1;                                     /* 准备下一位输出 */
    }
    return input;
}

/**
 * @brief 选中 BMI270 传感器（CS 拉低）
 *
 * 在 CS 下降沿后需要一个短暂的 spi_delay() 以确保 BMI270 识别到片选信号
 */
static void select_sensor(void)
{
    DL_GPIO_clearPins(BMI270_IO_PORT, BMI270_IO_IMU_CS_PIN);
    spi_delay();  /* 确保 CS 下降沿被传感器识别 */
}

/**
 * @brief 释放 BMI270 传感器（CS 拉高）
 *
 * 在 CS 上升沿前需要一个短暂的 spi_delay() 以确保最后一个字节传输完成
 */
static void release_sensor(void)
{
    spi_delay();  /* 确保最后的数据传输完成 */
    DL_GPIO_setPins(BMI270_IO_PORT, BMI270_IO_IMU_CS_PIN);
}

void BMI270_Port_Init(void)
{
    uint8_t discarded;

    /* GPIO 初始化：SCLK/MOSI 低，CS 高（无效状态） */
    DL_GPIO_clearPins(BMI270_IO_PORT,
                      BMI270_IO_IMU_SCLK_PIN | BMI270_IO_IMU_MOSI_PIN);
    DL_GPIO_setPins(BMI270_IO_PORT, BMI270_IO_IMU_CS_PIN);
    BSP_Time_DelayMs(1u);  /* 等待电源稳定 */

    /*
     * 上电后 CS 上升沿将 BMI270 切换到 SPI 接口模式。
     * Bosch 官方文档建议在上电后的第一次 SPI 通信前，
     * 执行一次丢弃的 CHIP_ID 读取，确保 SPI 接口稳定。
     */
    (void)BMI270_Port_ReadRegisters(
        BMI270_CHIP_ID_REGISTER, &discarded, 1u);
    BSP_Time_DelayMs(1u);
}

bool BMI270_Port_Probe(uint8_t *chip_id)
{
    uint8_t value = 0u;
    bool success;

    success = BMI270_Port_ReadRegisters(
        BMI270_CHIP_ID_REGISTER, &value, 1u);

    if (chip_id != (uint8_t *)0) {
        *chip_id = value;  /* 返回读取到的 CHIP_ID（即使校验失败） */
    }

    /* 验证：SPI 通信成功 且 CHIP_ID = 0x24 */
    return success && (value == BMI270_EXPECTED_CHIP_ID);
}

bool BMI270_Port_ReadRegisters(uint8_t register_address, uint8_t *data,
                               size_t length)
{
    size_t index;

    if ((data == (uint8_t *)0) || (length == 0u)) {
        return false;
    }

    /*
     * BMI270 SPI 读时序：
     *   1. 发送 (register_address | 0x80)：bit[7]=1 表示读操作
     *   2. 发送 dummy 字节 (0x00)：BMI270 在此字节期间输出数据
     */
    select_sensor();
    (void)spi_transfer((uint8_t)(register_address | 0x80u)); /* 读命令 */
    (void)spi_transfer(0x00u);                                /* dummy 字节 */

    /*
     * 连续读取 length 个字节。
     * BMI270 支持地址自动递增，因此可以连续读取多字节
     */
    for (index = 0u; index < length; ++index) {
        data[index] = spi_transfer(0x00u);
    }
    release_sensor();
    return true;
}

bool BMI270_Port_WriteRegisters(uint8_t register_address,
                                const uint8_t *data, size_t length)
{
    size_t index;

    if ((data == (const uint8_t *)0) || (length == 0u)) {
        return false;
    }

    /*
     * BMI270 SPI 写时序：
     *   1. 发送 (register_address & 0x7F)：bit[7]=0 表示写操作
     *   2. 发送数据字节
     */
    select_sensor();
    (void)spi_transfer((uint8_t)(register_address & 0x7Fu)); /* 写命令 */
    for (index = 0u; index < length; ++index) {
        (void)spi_transfer(data[index]);
    }
    release_sensor();
    return true;
}

uint8_t BMI270_Port_ReadInt1(void)
{
    /* 读取 INT1 (PA9) 引脚电平，返回 0 或 1 */
    return (DL_GPIO_readPins(BMI270_IO_PORT, BMI270_IO_INT1_PIN) != 0u) ?
           1u : 0u;
}

uint8_t BMI270_Port_ReadInt2(void)
{
    /* 读取 INT2 (PA15) 引脚电平，返回 0 或 1 */
    return (DL_GPIO_readPins(BMI270_IO_PORT, BMI270_IO_INT2_PIN) != 0u) ?
           1u : 0u;
}

/* ============ Bosch SensorAPI 兼容回调 ============ */

/**
 * 以下三个函数完全符合 Bosch SensorAPI 的接口约定：
 *   - int8_t read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
 *   - int8_t write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
 *   - void delay_us(uint32_t period, void *intf_ptr)
 *
 * interface_pointer 参数对应 bmi2_dev.intf_ptr，
 * 本实现不依赖它（所有 SPI 操作使用全局引脚定义），因此忽略。
 */

int8_t BMI270_Port_BoschRead(uint8_t register_address, uint8_t *data,
                             uint32_t length, void *interface_pointer)
{
    (void)interface_pointer;

    /*
     * Bosch API 返回值约定：0 = 成功, 非 0 = 失败 (BMI2_OK)
     * 将 bool 返回值映射为 int8_t：true -> 0, false -> -1
     */
    return BMI270_Port_ReadRegisters(register_address, data, length) ?
           0 : -1;
}

int8_t BMI270_Port_BoschWrite(uint8_t register_address, const uint8_t *data,
                              uint32_t length, void *interface_pointer)
{
    (void)interface_pointer;
    return BMI270_Port_WriteRegisters(register_address, data, length) ?
           0 : -1;
}

void BMI270_Port_BoschDelayUs(uint32_t period_us, void *interface_pointer)
{
    (void)interface_pointer;

    /*
     * 微秒级忙等待延迟。
     * CPUCLK_FREQ / 1000000 = 32 周期/微秒 @ 32 MHz
     * 精度：每循环约 32 周期，即约 1 us
     */
    while (period_us-- != 0u) {
        delay_cycles(CPUCLK_FREQ / 1000000u);
    }
}

#else

/*
 * PA8/PA9 已改作 WT61 UART1 时保留同名空实现，使未来恢复 BMI270 无需
 * 改动其他模块，同时避免引用已经从 SysConfig 删除的 BMI270_IO 宏。
 */
void BMI270_Port_Init(void)
{
}

bool BMI270_Port_Probe(uint8_t *chip_id)
{
    if (chip_id != NULL) {
        *chip_id = 0u;
    }
    return false;
}

bool BMI270_Port_ReadRegisters(uint8_t register_address, uint8_t *data,
                               size_t length)
{
    (void)register_address;
    (void)data;
    (void)length;
    return false;
}

bool BMI270_Port_WriteRegisters(uint8_t register_address,
                                const uint8_t *data, size_t length)
{
    (void)register_address;
    (void)data;
    (void)length;
    return false;
}

uint8_t BMI270_Port_ReadInt1(void)
{
    return 0u;
}

uint8_t BMI270_Port_ReadInt2(void)
{
    return 0u;
}

int8_t BMI270_Port_BoschRead(uint8_t register_address, uint8_t *data,
                             uint32_t length, void *interface_pointer)
{
    (void)register_address;
    (void)data;
    (void)length;
    (void)interface_pointer;
    return -1;
}

int8_t BMI270_Port_BoschWrite(uint8_t register_address,
                              const uint8_t *data, uint32_t length,
                              void *interface_pointer)
{
    (void)register_address;
    (void)data;
    (void)length;
    (void)interface_pointer;
    return -1;
}

void BMI270_Port_BoschDelayUs(uint32_t period_us, void *interface_pointer)
{
    (void)interface_pointer;
    while (period_us-- != 0u) {
        delay_cycles(CPUCLK_FREQ / 1000000u);
    }
}

#endif /* CONFIG_BMI270_ENABLED */
