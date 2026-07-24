/**
 * @file    st7735.c
 * @brief   ST7735 128x160 TFT LCD 驱动实现（软件 SPI）
 *
 * 使用 GPIO 位操作 (bit-banging) 模拟 SPI 时序，无需占用硬件 SPI 外设。
 * 选择软件 SPI 而非硬件 SPI 的原因：
 *   - MSPM0G3507 的 SPI 引脚复用选项有限，硬件 SPI 可能与 BMI270 冲突
 *   - 软件 SPI 在 32 MHz CPU 下可达数 MHz 速率，刷新 128x160 屏幕足够快
 *   - 引脚分配更灵活，方便 PCB 布线
 *
 * 显示特性：
 *   - 分辨率：128 x 160 像素
 *   - 色彩格式：RGB565（16 位/像素）
 *   - 四方向硬件旋转
 *   - 内置 5x7 像素 ASCII 字体（32 个字符：空格、'-'、'.'、':'、'0'-'9'、'A'-'Z'）
 */

#include "st7735.h"

#include <stddef.h>
#include "ti_msp_dl_config.h"

/* ============ ST7735 命令代码表 ============ */
/* 所有命令码参考 ST7735 datasheet */

#define CMD_SWRESET  0x01u  /**< 软件复位 */
#define CMD_SLPOUT   0x11u  /**< 退出休眠模式 */
#define CMD_NORON    0x13u  /**< 正常显示模式 */
#define CMD_INVOFF   0x20u  /**< 关闭显示反转 */
#define CMD_DISPON   0x29u  /**< 开启显示 */
#define CMD_CASET    0x2Au  /**< 列地址设置 */
#define CMD_RASET    0x2Bu  /**< 行地址设置 */
#define CMD_RAMWR    0x2Cu  /**< 内存写入 */
#define CMD_MADCTL   0x36u  /**< 内存数据访问控制（含旋转） */
#define CMD_COLMOD   0x3Au  /**< 像素格式设置 */
#define CMD_FRMCTR1  0xB1u  /**< 帧率控制 1（正常模式） */
#define CMD_FRMCTR2  0xB2u  /**< 帧率控制 2（空闲模式） */
#define CMD_FRMCTR3  0xB3u  /**< 帧率控制 3（部分模式） */
#define CMD_INVCTR   0xB4u  /**< 显示反转控制 */
#define CMD_PWCTR1   0xC0u  /**< 电源控制 1 */
#define CMD_PWCTR2   0xC1u  /**< 电源控制 2 */
#define CMD_PWCTR3   0xC2u  /**< 电源控制 3 */
#define CMD_PWCTR4   0xC3u  /**< 电源控制 4 */
#define CMD_PWCTR5   0xC4u  /**< 电源控制 5 */
#define CMD_VMCTR1   0xC5u  /**< VCOM 控制 1 */
#define CMD_GMCTRP1  0xE0u  /**< 正极性伽马校正 */
#define CMD_GMCTRN1  0xE1u  /**< 负极性伽马校正 */

/* ============ MADCTL 寄存器位定义（旋转控制） ============ */
#define MADCTL_MY   0x80u  /**< 行地址递增方向（上下镜像） */
#define MADCTL_MX   0x40u  /**< 列地址递增方向（左右镜像） */
#define MADCTL_MV   0x20u  /**< 行列交换（旋转 90°/270°） */
#define MADCTL_BGR  0x08u  /**< 颜色顺序 BGR（ST7735 默认 BGR 而非 RGB） */

/* ============ 模块级状态变量 ============ */
/* 这些变量根据当前旋转方向动态变化 */

/** 当前逻辑显示宽度 */
static uint16_t display_width = ST7735_WIDTH;

/** 当前逻辑显示高度 */
static uint16_t display_height = ST7735_HEIGHT;

/** 当前列偏移量（GRAM 坐标 = 逻辑坐标 + offset） */
static uint8_t x_offset = ST7735_X_OFFSET;

/** 当前行偏移量 */
static uint8_t y_offset = ST7735_Y_OFFSET;

/* ============ GPIO 底层操作 ============ */

/** @brief 设置 LCD 端口的指定引脚为高电平 */
static inline void pin_set(uint32_t pin)
{
    DL_GPIO_setPins(LCD_PORT, pin);
}

/** @brief 设置 LCD 端口的指定引脚为低电平 */
static inline void pin_clear(uint32_t pin)
{
    DL_GPIO_clearPins(LCD_PORT, pin);
}

/** @brief 毫秒级忙等待延迟（仅初始化使用） */
static void delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0u) {
        delay_cycles(CPUCLK_FREQ / 1000u);
    }
}

/**
 * @brief 软件 SPI 发送一个字节（MSB 先）
 * @param value 要发送的字节
 *
 * SPI Mode 0 (CPOL=0, CPHA=0)：SCLK 空闲低电平，在上升沿采样
 *
 * 时序：
 *   SCLK \_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\__
 *   MOSI --[b7]--[b6]--[b5]-- ... --[b0]----
 *        ^                              ^
 *    先输出 MSB                     最后输出 LSB
 */
static void spi_write_byte(uint8_t value)
{
    uint8_t bit;
    for (bit = 0u; bit < 8u; ++bit) {
        pin_clear(LCD_SCLK_PIN);                        /* SCLK 低 */
        if ((value & 0x80u) != 0u) {
            pin_set(LCD_MOSI_PIN);                      /* MOSI = 1 */
        } else {
            pin_clear(LCD_MOSI_PIN);                    /* MOSI = 0 */
        }
        pin_set(LCD_SCLK_PIN);                          /* SCLK 上升沿 -> 从机采样 */
        value <<= 1;                                    /* 准备下一位 */
    }
    pin_clear(LCD_SCLK_PIN);                            /* 恢复空闲状态 */
}

/* ============ ST7735 命令/数据协议 ============ */

/**
 * @brief 发送命令 + 数据序列
 * @param command 命令字节
 * @param data    参数数据（可为 NULL）
 * @param length  参数长度
 *
 * 协议：CS 拉低 -> DC 拉低（命令）-> 发送命令字节
 *       -> DC 拉高（数据）-> 发送参数字节 -> CS 拉高
 */
static void write_command_data(uint8_t command, const uint8_t *data,
                               uint8_t length)
{
    pin_clear(LCD_CS_PIN);           /* CS = 0，选中芯片 */
    pin_clear(LCD_DC_PIN);           /* DC = 0，命令模式 */
    spi_write_byte(command);
    pin_set(LCD_DC_PIN);             /* DC = 1，数据模式 */
    while (length-- != 0u) {
        spi_write_byte(*data++);
    }
    pin_set(LCD_CS_PIN);             /* CS = 1，释放总线 */
}

/** @brief 仅发送命令（无参数） */
static void write_command(uint8_t command)
{
    write_command_data(command, NULL, 0u);
}

/**
 * @brief 设置像素写入的地址窗口
 * @param x0 起始列
 * @param y0 起始行
 * @param x1 结束列
 * @param y1 结束行
 *
 * 后续 RAMWR 命令写入的数据将按行填充此窗口。
 * 窗口坐标会自动加上偏移量以适配某些非标准屏幕。
 */
static void set_address_window(uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1)
{
    uint8_t data[4];
    x0 += x_offset;
    x1 += x_offset;
    y0 += y_offset;
    y1 += y_offset;

    /* CASET: 列地址设置，16 位参数 */
    data[0] = (uint8_t)(x0 >> 8); data[1] = (uint8_t)x0;
    data[2] = (uint8_t)(x1 >> 8); data[3] = (uint8_t)x1;
    write_command_data(CMD_CASET, data, 4u);

    /* RASET: 行地址设置，16 位参数 */
    data[0] = (uint8_t)(y0 >> 8); data[1] = (uint8_t)y0;
    data[2] = (uint8_t)(y1 >> 8); data[3] = (uint8_t)y1;
    write_command_data(CMD_RASET, data, 4u);

    /* RAMWR: 内存写入命令（后续数据为像素颜色） */
    write_command(CMD_RAMWR);
}

/**
 * @brief 开始像素数据写入阶段
 *
 * 在 RAMWR 命令后，拉低 CS、拉高 DC 准备传输像素颜色数据
 */
static void begin_pixel_data(void)
{
    pin_clear(LCD_CS_PIN);
    pin_set(LCD_DC_PIN);
}

/**
 * @brief 结束像素数据写入阶段
 */
static void end_pixel_data(void)
{
    pin_set(LCD_CS_PIN);
}

/**
 * @brief 批量写入指定颜色
 * @param color RGB565 颜色值
 * @param count 写入的像素数量
 *
 * 每个像素为 16 位，分两个字节发送（高字节先）。
 * 此函数直接操作 CS/DC 引脚以提高批量填充速度。
 */
static void write_color(uint16_t color, uint32_t count)
{
    const uint8_t high = (uint8_t)(color >> 8);  /* 颜色高字节 RRRRRGGG */
    const uint8_t low = (uint8_t)color;          /* 颜色低字节 GGGBBBBB */
    begin_pixel_data();
    while (count-- != 0u) {
        spi_write_byte(high);
        spi_write_byte(low);
    }
    end_pixel_data();
}

/* ============ 公共接口 ============ */

void ST7735_Init(void)
{
    /*
     * ST7735 初始化序列参数表。
     * 这些值是针对 128x160 模块在 5V 供电下的推荐配置，
     * 来自 Adafruit 和社区广泛验证的参数集。
     */
    static const uint8_t frm1[] = {0x01u, 0x2Cu, 0x2Du};
    static const uint8_t frm3[] = {0x01u, 0x2Cu, 0x2Du, 0x01u, 0x2Cu, 0x2Du};
    static const uint8_t gamma_p[] = {
        0x02u,0x1Cu,0x07u,0x12u,0x37u,0x32u,0x29u,0x2Du,
        0x29u,0x25u,0x2Bu,0x39u,0x00u,0x01u,0x03u,0x10u};
    static const uint8_t gamma_n[] = {
        0x03u,0x1Du,0x07u,0x06u,0x2Eu,0x2Cu,0x29u,0x2Du,
        0x2Eu,0x2Eu,0x37u,0x3Fu,0x00u,0x00u,0x02u,0x10u};
    uint8_t data[3];

    /* ----- 硬件复位序列 ----- */
    pin_set(LCD_CS_PIN | LCD_DC_PIN | LCD_RST_PIN | LCD_BL_PIN);
    pin_clear(LCD_SCLK_PIN | LCD_MOSI_PIN);
    delay_ms(10u);

    pin_clear(LCD_RST_PIN);     /* 拉低复位 */
    delay_ms(20u);
    pin_set(LCD_RST_PIN);       /* 释放复位 */
    delay_ms(120u);             /* 等待芯片内部初始化完成 */

    /* ----- 软件初始化序列 ----- */
    write_command(CMD_SWRESET);  delay_ms(150u);  /* 软复位 + 等待 */
    write_command(CMD_SLPOUT);   delay_ms(150u);  /* 退出休眠 */

    /* 帧率控制 */
    write_command_data(CMD_FRMCTR1, frm1, sizeof(frm1));
    write_command_data(CMD_FRMCTR2, frm1, sizeof(frm1));
    write_command_data(CMD_FRMCTR3, frm3, sizeof(frm3));

    /* 显示反转控制 */
    data[0] = 0x07u; write_command_data(CMD_INVCTR, data, 1u);

    /* 电源控制配置 */
    data[0] = 0xA2u; data[1] = 0x02u; data[2] = 0x84u;
    write_command_data(CMD_PWCTR1, data, 3u);
    data[0] = 0xC5u; write_command_data(CMD_PWCTR2, data, 1u);
    data[0] = 0x0Au; data[1] = 0x00u;
    write_command_data(CMD_PWCTR3, data, 2u);
    data[0] = 0x8Au; data[1] = 0x2Au;
    write_command_data(CMD_PWCTR4, data, 2u);
    data[0] = 0x8Au; data[1] = 0xEEu;
    write_command_data(CMD_PWCTR5, data, 2u);

    /* VCOM 控制 */
    data[0] = 0x0Eu; write_command_data(CMD_VMCTR1, data, 1u);

    write_command(CMD_INVOFF);   /* 关闭反转 */

    /* 像素格式：16 位/像素 (RGB565) */
    data[0] = 0x05u; write_command_data(CMD_COLMOD, data, 1u);

    /* 伽马校正表（正极性 + 负极性） */
    write_command_data(CMD_GMCTRP1, gamma_p, sizeof(gamma_p));
    write_command_data(CMD_GMCTRN1, gamma_n, sizeof(gamma_n));

    write_command(CMD_NORON); delay_ms(10u);   /* 正常显示模式 */

    ST7735_SetRotation(0u);                    /* 默认旋转方向 */

    write_command(CMD_DISPON); delay_ms(100u); /* 打开显示 */

    ST7735_FillScreen(ST7735_BLACK);           /* 清屏 */
}

void ST7735_SetRotation(uint8_t rotation)
{
    uint8_t madctl;

    /*
     * 根据旋转值设置 MADCTL 寄存器并交换宽/高和偏移量。
     * MADCTL 是 ST7735 中控制内存数据访问顺序的寄存器，
     * 通过组合 MY/MX/MV/BGR 位实现四种旋转方向。
     */
    switch (rotation & 3u) {
    default:  /* 和 case 0 相同 */
    case 0u:
        madctl = MADCTL_MX | MADCTL_MY | MADCTL_BGR;   /* 正常方向 */
        display_width = ST7735_WIDTH; display_height = ST7735_HEIGHT;
        x_offset = ST7735_X_OFFSET; y_offset = ST7735_Y_OFFSET;
        break;
    case 1u:
        madctl = MADCTL_MY | MADCTL_MV | MADCTL_BGR;   /* 右旋 90° */
        display_width = ST7735_HEIGHT; display_height = ST7735_WIDTH;
        x_offset = ST7735_Y_OFFSET; y_offset = ST7735_X_OFFSET;
        break;
    case 2u:
        madctl = MADCTL_BGR;                            /* 180° 旋转 */
        display_width = ST7735_WIDTH; display_height = ST7735_HEIGHT;
        x_offset = ST7735_X_OFFSET; y_offset = ST7735_Y_OFFSET;
        break;
    case 3u:
        madctl = MADCTL_MX | MADCTL_MV | MADCTL_BGR;   /* 左旋 90° */
        display_width = ST7735_HEIGHT; display_height = ST7735_WIDTH;
        x_offset = ST7735_Y_OFFSET; y_offset = ST7735_X_OFFSET;
        break;
    }
    write_command_data(CMD_MADCTL, &madctl, 1u);
}

uint16_t ST7735_GetWidth(void) { return display_width; }
uint16_t ST7735_GetHeight(void) { return display_height; }

void ST7735_Backlight(bool on)
{
    if (on) pin_set(LCD_BL_PIN); else pin_clear(LCD_BL_PIN);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    /* 边界检查：超出屏幕范围的像素直接丢弃 */
    if ((x >= display_width) || (y >= display_height)) return;

    /* 设置 1x1 的地址窗口，写入一个像素颜色 */
    set_address_window(x, y, x, y);
    write_color(color, 1u);
}

void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                     uint16_t color)
{
    /* 裁剪：确保绘制区域不超出屏幕边界 */
    if ((w == 0u) || (h == 0u) || (x >= display_width) ||
        (y >= display_height)) return;
    if ((uint32_t)x + w > display_width) w = display_width - x;
    if ((uint32_t)y + h > display_height) h = display_height - y;

    /* 设置矩形地址窗口，批量填充颜色 */
    set_address_window(x, y, x + w - 1u, y + h - 1u);
    write_color(color, (uint32_t)w * h);
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRect(0u, 0u, display_width, display_height, color);
}

/* ============ 5x7 像素 ASCII 字体 ============ */

/*
 * 字体索引表 (共 40 个条目)：
 *   index 0:  空格
 *   index 1:  '-'
 *   index 2:  '.'
 *   index 3:  ':'
 *   index 4-13: '0'..'9'
 *   index 14-39: 'A'..'Z'
 *
 * 每个字形为 5 列 x 7 行（实际显示 6 列 x 8 行，右边和下边各 1 像素间距）。
 * 数据格式：每个字形的 5 个字节分别对应 5 列的位图（bit 0 对应顶行）。
 */

/* 5 columns x 7 rows. Supported set: space, - . :, 0-9 and A-Z. */
static const uint8_t font5x7[][5] = {
    /* idx  0: 空格   */ {0x00,0x00,0x00,0x00,0x00},
    /* idx  1: '-'    */ {0x00,0x08,0x08,0x08,0x00},
    /* idx  2: '.'    */ {0x00,0x60,0x60,0x00,0x00},
    /* idx  3: ':'    */ {0x00,0x36,0x36,0x00,0x00},
    /* idx  4: '0'    */ {0x3E,0x51,0x49,0x45,0x3E},
    /* idx  5: '1'    */ {0x00,0x42,0x7F,0x40,0x00},
    /* idx  6: '2'    */ {0x42,0x61,0x51,0x49,0x46},
    /* idx  7: '3'    */ {0x21,0x41,0x45,0x4B,0x31},
    /* idx  8: '4'    */ {0x18,0x14,0x12,0x7F,0x10},
    /* idx  9: '5'    */ {0x27,0x45,0x45,0x45,0x39},
    /* idx 10: '6'    */ {0x3C,0x4A,0x49,0x49,0x30},
    /* idx 11: '7'    */ {0x01,0x71,0x09,0x05,0x03},
    /* idx 12: '8'    */ {0x36,0x49,0x49,0x49,0x36},
    /* idx 13: '9'    */ {0x06,0x49,0x49,0x29,0x1E},
    /* idx 14: 'A'    */ {0x7E,0x11,0x11,0x11,0x7E},
    /* idx 15: 'B'    */ {0x7F,0x49,0x49,0x49,0x36},
    /* idx 16: 'C'    */ {0x3E,0x41,0x41,0x41,0x22},
    /* idx 17: 'D'    */ {0x7F,0x41,0x41,0x22,0x1C},
    /* idx 18: 'E'    */ {0x7F,0x49,0x49,0x49,0x41},
    /* idx 19: 'F'    */ {0x7F,0x09,0x09,0x09,0x01},
    /* idx 20: 'G'    */ {0x3E,0x41,0x49,0x49,0x7A},
    /* idx 21: 'H'    */ {0x7F,0x08,0x08,0x08,0x7F},
    /* idx 22: 'I'    */ {0x00,0x41,0x7F,0x41,0x00},
    /* idx 23: 'J'    */ {0x20,0x40,0x41,0x3F,0x01},
    /* idx 24: 'K'    */ {0x7F,0x08,0x14,0x22,0x41},
    /* idx 25: 'L'    */ {0x7F,0x40,0x40,0x40,0x40},
    /* idx 26: 'M'    */ {0x7F,0x02,0x0C,0x02,0x7F},
    /* idx 27: 'N'    */ {0x7F,0x04,0x08,0x10,0x7F},
    /* idx 28: 'O'    */ {0x3E,0x41,0x41,0x41,0x3E},
    /* idx 29: 'P'    */ {0x7F,0x09,0x09,0x09,0x06},
    /* idx 30: 'Q'    */ {0x3E,0x41,0x51,0x21,0x5E},
    /* idx 31: 'R'    */ {0x7F,0x09,0x19,0x29,0x46},
    /* idx 32: 'S'    */ {0x46,0x49,0x49,0x49,0x31},
    /* idx 33: 'T'    */ {0x01,0x01,0x7F,0x01,0x01},
    /* idx 34: 'U'    */ {0x3F,0x40,0x40,0x40,0x3F},
    /* idx 35: 'V'    */ {0x1F,0x20,0x40,0x20,0x1F},
    /* idx 36: 'W'    */ {0x3F,0x40,0x38,0x40,0x3F},
    /* idx 37: 'X'    */ {0x63,0x14,0x08,0x14,0x63},
    /* idx 38: 'Y'    */ {0x07,0x08,0x70,0x08,0x07},
    /* idx 39: 'Z'    */ {0x61,0x51,0x49,0x45,0x43}
};

/**
 * @brief 将 ASCII 字符映射到字体索引
 * @param c ASCII 字符
 * @return 字体表索引（0-39），不支持的字符返回 0（空格）
 */
static uint8_t glyph_index(char c)
{
    if (c == ' ') return 0u;
    if (c == '-') return 1u;
    if (c == '.') return 2u;
    if (c == ':') return 3u;
    if ((c >= '0') && (c <= '9')) return (uint8_t)(4 + c - '0');
    /* 小写字母自动转大写 */
    if ((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
    if ((c >= 'A') && (c <= 'Z')) return (uint8_t)(14 + c - 'A');
    return 0u; /* 不支持的字符显示为空格 */
}

void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t foreground,
                     uint16_t background, uint8_t scale)
{
    uint8_t col, row;
    const uint8_t *glyph = font5x7[glyph_index(c)];

    if (scale == 0u) scale = 1u;

    /*
     * 字符完整位于屏幕内时，只设置一次地址窗口并连续写 6*8*scale^2
     * 个像素。旧实现每个字符设置 48 次窗口，动态刷屏会非常慢。
     */
    if (((uint32_t)x + (uint32_t)6u * scale <= display_width) &&
        ((uint32_t)y + (uint32_t)8u * scale <= display_height)) {
        uint8_t sy;
        uint8_t sx;

        set_address_window(x, y,
                           x + (uint16_t)6u * scale - 1u,
                           y + (uint16_t)8u * scale - 1u);
        begin_pixel_data();
        for (row = 0u; row < 8u; ++row) {
            for (sy = 0u; sy < scale; ++sy) {
                for (col = 0u; col < 6u; ++col) {
                    uint8_t bits = (col < 5u) ? glyph[col] : 0u;
                    uint16_t color =
                        ((bits & (1u << row)) != 0u) ?
                        foreground : background;
                    for (sx = 0u; sx < scale; ++sx) {
                        spi_write_byte((uint8_t)(color >> 8));
                        spi_write_byte((uint8_t)color);
                    }
                }
            }
        }
        end_pixel_data();
        return;
    }

    /* 屏幕边缘的非完整字符使用带裁剪的通用矩形路径。 */
    for (col = 0u; col < 6u; ++col) {
        uint8_t bits = (col < 5u) ? glyph[col] : 0u;
        for (row = 0u; row < 8u; ++row) {
            uint16_t color =
                ((bits & (1u << row)) != 0u) ? foreground : background;
            ST7735_FillRect(x + (uint16_t)col * scale,
                           y + (uint16_t)row * scale,
                           scale, scale, color);
        }
    }
}

void ST7735_DrawString(uint16_t x, uint16_t y, const char *text,
                       uint16_t foreground, uint16_t background, uint8_t scale)
{
    uint16_t cursor = x;

    if (scale == 0u) scale = 1u;

    /* 逐字符绘制，自动换行 */
    while ((text != NULL) && (*text != '\0')) {
        if (*text == '\n') {
            cursor = x;                              /* 换行：回到起始列 */
            y += (uint16_t)8u * scale;               /* 下移 8*s 像素 */
        } else {
            ST7735_DrawChar(cursor, y, *text, foreground, background, scale);
            cursor += (uint16_t)6u * scale;          /* 右移 6*s 像素 */
        }
        ++text;
    }
}

uint16_t ST7735_Color565(uint8_t red, uint8_t green, uint8_t blue)
{
    /*
     * RGB565 编码：
     *   位 15-11: 红色 (5 位, 取 red 高 5 位)
     *   位 10-5:  绿色 (6 位, 取 green 高 6 位)
     *   位 4-0:   蓝色 (5 位, 取 blue 高 5 位)
     *
     *   red   & 0xF8 -> 保留高 5 位，低 3 位清零
     *   green & 0xFC -> 保留高 6 位，低 2 位清零
     *   blue  >> 3   -> 右移 3 位，保留高 5 位
     */
    return (uint16_t)(((uint16_t)(red & 0xF8u) << 8) |
                      ((uint16_t)(green & 0xFCu) << 3) | (blue >> 3));
}
