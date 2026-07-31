/**
 * @file    screen_task.c
 * @brief   ST7735 状态显示任务实现
 *
 * 固定文字只在初始化绘制一次，周期刷新时仅覆盖数值区域，减少软件 SPI
 * 占用。刷屏期间中断保持开启，编码器 10 ms 快照和 UART DMA 均可继续。
 */

#include "screen_task.h"

#include <stddef.h>

#include "project_config.h"
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

/** 从0开始的比赛计时显示为秒.十分之一秒，例如“12.3s”。 */
static void format_race_time(uint32_t elapsed_ms, char output[12])
{
    uint32_t tenths = elapsed_ms / 100u;
    uint32_t seconds = tenths / 10u;
    uint8_t position = 0u;
    char reverse[8];
    uint8_t count = 0u;

    if (seconds > 9999u) {
        seconds = 9999u;
    }
    do {
        reverse[count++] = (char)('0' + seconds % 10u);
        seconds /= 10u;
    } while ((seconds != 0u) && (count < sizeof(reverse)));
    while (count != 0u) {
        output[position++] = reverse[--count];
    }
    output[position++] = '.';
    output[position++] = (char)('0' + tenths % 10u);
    output[position++] = 's';
    output[position] = '\0';
}

static const char *race_state_text(ScreenRaceState state, bool infinite_mode)
{
    switch (state) {
        case SCREEN_RACE_RUN:
            return infinite_mode ? "LOOP" : "RUN";
        case SCREEN_RACE_BRAKE:
            return "BRAKE";
        case SCREEN_RACE_DONE:
            return "DONE";
        case SCREEN_RACE_FAULT:
            return "FAULT";
        case SCREEN_RACE_WAIT:
        default:
            return infinite_mode ? "I-WAIT" : "WAIT";
    }
}

static void draw_value(uint16_t y, const char *text, uint16_t color)
{
    ST7735_FillRect(SCREEN_VALUE_X, y, SCREEN_VALUE_W,
                    SCREEN_VALUE_H, ST7735_BLACK);
    ST7735_DrawString(SCREEN_VALUE_X, y, text, color,
                      ST7735_BLACK, 1u);
}

/** 转换成固定8字符，屏幕从左到右依次显示S1...S8状态位。 */
static void format_line_raw_bits(uint16_t raw_mask, char output[9])
{
    uint8_t channel;

    for (channel = 0u; channel < 8u; ++channel) {
        output[channel] =
            ((raw_mask & ((uint16_t)1u << channel)) != 0u) ? '1' : '0';
    }
    output[8] = '\0';
}

/** 每行显示连续2路16位原始模拟值，固定5位十进制。 */
static void format_adc_row(const uint16_t values[16], uint8_t first,
                           char output[12])
{
    uint8_t item;
    uint8_t position = 0u;

    for (item = 0u; item < 2u; ++item) {
        uint16_t value = values[(uint8_t)(first + item)];

        output[position++] = (char)('0' + (value / 10000u));
        output[position++] = (char)('0' + ((value / 1000u) % 10u));
        output[position++] = (char)('0' + ((value / 100u) % 10u));
        output[position++] = (char)('0' + ((value / 10u) % 10u));
        output[position++] = (char)('0' + (value % 10u));
        if (item != 1u) {
            output[position++] = ' ';
        }
    }
    output[position] = '\0';
}

void ScreenTask_Init(void)
{
    char text[12];

    ST7735_Init();
    ST7735_DrawString(4u, 3u, "TI3507 CAR", ST7735_CYAN,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(76u, 3u, "WAIT", ST7735_YELLOW,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 12u, "TIME:", ST7735_CYAN,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 22u, "RPM1:", ST7735_WHITE,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 40u, "RPM2:", ST7735_WHITE,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(4u, 58u, "YAW:", ST7735_YELLOW,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(0u, 70u, "RAW S1-8 T:", ST7735_CYAN,
                       ST7735_BLACK, 1u);
    format_integer(CONFIG_LINE_SENSOR_ADC_THRESHOLD, text);
    ST7735_DrawString(72u, 70u, text, ST7735_CYAN,
                      ST7735_BLACK, 1u);
    ST7735_DrawString(0u, 122u, "STATE S1->S8", ST7735_CYAN,
                       ST7735_BLACK, 1u);
#if (CONFIG_LINE_SENSOR_POSITION_MODE == \
     CONFIG_LINE_SENSOR_POSITION_ANALOG_WEIGHTED)
    ST7735_DrawString(4u, 144u, "APOS:", ST7735_CYAN,
                       ST7735_BLACK, 1u);
#else
    ST7735_DrawString(4u, 144u, "DPOS:", ST7735_CYAN,
                       ST7735_BLACK, 1u);
#endif
}

void ScreenTask_Update(int32_t rpm_m1, int32_t rpm_m2,
                       float yaw_deg, bool wt61_online,
                       const uint16_t line_adc_values[16],
                       uint16_t line_raw_mask,
                       int16_t line_position, bool line_lost,
                       uint32_t race_elapsed_ms,
                       ScreenRaceState race_state,
                       bool infinite_mode)
{
    char text[12];
    char raw_bits[9];
    char adc_row[12];
    uint8_t row;

    format_race_time(race_elapsed_ms, text);
    draw_value(12u, text, ST7735_CYAN);
    format_integer(rpm_m1, text);
    draw_value(22u, text, ST7735_GREEN);
    format_integer(rpm_m2, text);
    draw_value(40u, text, ST7735_GREEN);
    format_angle(yaw_deg, text);
    draw_value(58u, text, ST7735_YELLOW);

    for (row = 0u; row < 4u; ++row) {
        uint16_t y = (uint16_t)(80u + (uint16_t)row * 10u);

        format_adc_row(line_adc_values, (uint8_t)(row * 2u), adc_row);
        ST7735_FillRect(0u, y, 114u, SCREEN_VALUE_H, ST7735_BLACK);
        ST7735_DrawString(0u, y, adc_row,
                          ST7735_GREEN, ST7735_BLACK, 1u);
    }

    format_line_raw_bits(line_raw_mask, raw_bits);
    ST7735_FillRect(0u, 132u, 96u, SCREEN_VALUE_H, ST7735_BLACK);
    ST7735_DrawString(0u, 132u, raw_bits,
                      ST7735_WHITE, ST7735_BLACK, 1u);

    ST7735_FillRect(SCREEN_VALUE_X, 144u, SCREEN_VALUE_W,
                    SCREEN_VALUE_H, ST7735_BLACK);
    if (line_lost) {
        ST7735_DrawString(SCREEN_VALUE_X, 144u, "LOST",
                          ST7735_RED, ST7735_BLACK, 1u);
    } else {
        format_integer(line_position, text);
        ST7735_DrawString(SCREEN_VALUE_X, 144u, text,
                          ST7735_CYAN, ST7735_BLACK, 1u);
    }

    ST7735_FillRect(76u, 3u, 52u, SCREEN_VALUE_H, ST7735_BLACK);
    ST7735_DrawString(76u, 3u, wt61_online ?
                      race_state_text(race_state, infinite_mode) : "IMU!",
                      wt61_online ?
                      ((race_state == SCREEN_RACE_FAULT) ?
                       ST7735_RED : ST7735_GREEN) : ST7735_RED,
                      ST7735_BLACK, 1u);
}
