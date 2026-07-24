/**
 * @file    screen_task.c
 * @brief   ST7735 状态显示任务实现
 *
 * 固定文字只在初始化绘制一次，周期刷新时仅覆盖数值区域，减少软件 SPI
 * 占用。刷屏期间中断保持开启，编码器 10 ms 快照和 UART DMA 均可继续。
 */

#include "screen_task.h"

#include <stddef.h>

#include "st7735.h"

#define SCREEN_VALUE_X       (48u)
#define SCREEN_VALUE_W       (78u)
#define SCREEN_VALUE_H       (8u)

/** 把整数转换成右对齐的固定宽度字符串，避免引入 printf。 */
static void format_integer(int32_t value, char output[12])
{
    char reverse[10];
    uint32_t magnitude;
    uint8_t count = 0u;
    uint8_t position = 0u;
    bool negative = value < 0;

    magnitude = negative ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    do {
        reverse[count++] = (char)('0' + magnitude % 10u);
        magnitude /= 10u;
    } while ((magnitude != 0u) && (count < sizeof(reverse)));

    if (negative) {
        output[position++] = '-';
    }
    while (count != 0u) {
        output[position++] = reverse[--count];
    }
    output[position] = '\0';
}

/** 把角度格式化为一位小数，例如 -123.4。 */
static void format_angle(float value, char output[12])
{
    char reverse[8];
    int32_t tenths;
    uint32_t magnitude;
    uint8_t count = 0u;
    uint8_t position = 0u;
    bool negative;

    if (value > 9999.9f) {
        value = 9999.9f;
    } else if (value < -9999.9f) {
        value = -9999.9f;
    }
    tenths = (int32_t)(value * 10.0f +
                       ((value >= 0.0f) ? 0.5f : -0.5f));
    negative = tenths < 0;
    magnitude = negative ? (uint32_t)(-(int64_t)tenths) :
                           (uint32_t)tenths;

    /* 先保存小数点右侧一位，再保存整数部分。 */
    reverse[count++] = (char)('0' + magnitude % 10u);
    magnitude /= 10u;
    do {
        reverse[count++] = (char)('0' + magnitude % 10u);
        magnitude /= 10u;
    } while ((magnitude != 0u) && (count < sizeof(reverse)));

    if (negative) {
        output[position++] = '-';
    }
    while (count > 1u) {
        output[position++] = reverse[--count];
    }
    output[position++] = '.';
    output[position++] = reverse[0];
    output[position] = '\0';
}

static void draw_value(uint16_t y, const char *text, uint16_t color)
{
    ST7735_FillRect(SCREEN_VALUE_X, y, SCREEN_VALUE_W,
                    SCREEN_VALUE_H, ST7735_BLACK);
    ST7735_DrawString(SCREEN_VALUE_X, y, text, color,
                      ST7735_BLACK, 1u);
}

void ScreenTask_Init(void)
{
    ST7735_Init();
    ST7735_DrawString(4u, 3u, "TI3507 CAR", ST7735_CYAN,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 12u, "SET:", ST7735_CYAN,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 22u, "RPM1:", ST7735_WHITE,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 40u, "RPM2:", ST7735_WHITE,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 66u, "YAW:", ST7735_YELLOW,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 84u, "PITCH:", ST7735_YELLOW,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 102u, "ROLL:", ST7735_YELLOW,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 132u, "WT61 WAIT", ST7735_RED,
                      ST7735_BLACK, 1u);
}

void ScreenTask_Update(int32_t target_rpm, int32_t rpm_m1, int32_t rpm_m2,
                       float yaw_deg, float pitch_deg, float roll_deg,
                       bool wt61_online)
{
    char text[12];

    format_integer(target_rpm, text);
    draw_value(12u, text, ST7735_CYAN);
    format_integer(rpm_m1, text);
    draw_value(22u, text, ST7735_GREEN);
    format_integer(rpm_m2, text);
    draw_value(40u, text, ST7735_GREEN);
    format_angle(yaw_deg, text);
    draw_value(66u, text, ST7735_YELLOW);
    format_angle(pitch_deg, text);
    draw_value(84u, text, ST7735_YELLOW);
    format_angle(roll_deg, text);
    draw_value(102u, text, ST7735_YELLOW);

    ST7735_FillRect(4u, 132u, 90u, 8u, ST7735_BLACK);
    ST7735_DrawString(4u, 132u,
                      wt61_online ? "WT61 OK" : "WT61 WAIT",
                      wt61_online ? ST7735_GREEN : ST7735_RED,
                      ST7735_BLACK, 1u);
}
