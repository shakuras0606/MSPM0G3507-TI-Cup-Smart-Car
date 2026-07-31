/**
 * @file    st7735.h
 * @brief   ST7735 128x160 TFT LCD 驱动 API（软件 SPI）
 *
 * 使用 GPIO 位操作（bit-banging）实现的软件 SPI 驱动，
 * 不依赖硬件 SPI 外设，适合引脚灵活分配的场合。
 *
 * 支持：
 *   - 128x160 像素 RGB565 色彩显示
 *   - 四种屏幕旋转方向
 *   - 内置 5x7 像素 ASCII 字体（空格、数字、大写字母、'-'、'.'、':'）
 *   - 像素缩放（整数倍放大）
 *   - BLK硬接3.3V，软件不再控制背光
 *
 * 颜色格式：RGB565（红 5 位、绿 6 位、蓝 5 位）
 *
 * 引脚：PB8(MOSI), PB9(SCLK), PB10(RST), PB11(DC), PB14(CS)
 *       BLK直接接3.3V；PB26专用于巡线模块S2。
 */

#ifndef ST7735_H_
#define ST7735_H_

#include <stdbool.h>
#include <stdint.h>

/** LCD 物理分辨率：宽度（像素） */
#define ST7735_WIDTH   128u

/** LCD 物理分辨率：高度（像素） */
#define ST7735_HEIGHT  160u

/**
 * 某些 128x160 模块的 GRAM 原点与显示原点之间有偏移。
 * 若显示内容整体偏移，可尝试调整这两个值。
 * 当前模块实测右边和下边各残留一条未覆盖彩线，说明可见区相对 GRAM
 * 原点分别偏移一列和一行，因此配置为 (1, 1)。
 *
 * ST7735 数据手册说明控制器可能使用 132x162 显存，而玻璃可见区为
 * 128x160；窗口偏移属于屏幕模组参数，不是少画一个像素。
 */
#define ST7735_X_OFFSET  1u
#define ST7735_Y_OFFSET  1u

/* ========== RGB565 预定义颜色 ========== */
#define ST7735_BLACK    0x0000u     /**< 黑色   R:0   G:0   B:0   */
#define ST7735_BLUE     0x001Fu     /**< 蓝色   R:0   G:0   B:31  */
#define ST7735_RED      0xF800u     /**< 红色   R:31  G:0   B:0   */
#define ST7735_GREEN    0x07E0u     /**< 绿色   R:0   G:63  B:0   */
#define ST7735_CYAN     0x07FFu     /**< 青色   R:0   G:63  B:31  */
#define ST7735_MAGENTA  0xF81Fu     /**< 品红   R:31  G:0   B:31  */
#define ST7735_YELLOW   0xFFE0u     /**< 黄色   R:31  G:63  B:0   */
#define ST7735_WHITE    0xFFFFu     /**< 白色   R:31  G:63  B:31  */

/**
 * @brief 初始化 ST7735 LCD
 *
 * 执行：硬件复位 -> 软复位 -> 退出休眠 -> 帧率/电源/伽马/色彩模式配置
 * -> 设置默认旋转方向 -> 打开显示 -> 清屏为黑色
 */
void ST7735_Init(void);

/**
 * @brief 设置屏幕旋转方向
 * @param rotation 旋转值 (0-3)
 *                 0: 正常   1: 右旋 90°
 *                 2: 180°   3: 左旋 90°
 *
 * 旋转后 ST7735_GetWidth() / ST7735_GetHeight() 会自动交换
 */
void ST7735_SetRotation(uint8_t rotation);

/**
 * @brief 获取当前逻辑宽度（像素）
 * @return 旋转后的屏幕宽度
 */
uint16_t ST7735_GetWidth(void);

/**
 * @brief 获取当前逻辑高度（像素）
 * @return 旋转后的屏幕高度
 */
uint16_t ST7735_GetHeight(void);

/**
 * @brief 兼容旧代码的背光接口；BLK硬接3.3V，因此本函数不执行GPIO操作。
 * @param on 参数被忽略。
 */
void ST7735_Backlight(bool on);

/**
 * @brief 在指定坐标绘制一个像素点
 * @param x     横坐标（0 <= x < width）
 * @param y     纵坐标（0 <= y < height）
 * @param color RGB565 颜色值
 */
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief 填充矩形区域
 * @param x     矩形左上角横坐标
 * @param y     矩形左上角纵坐标
 * @param w     矩形宽度（像素）
 * @param h     矩形高度（像素）
 * @param color RGB565 颜色值
 *
 * 超出屏幕边界的部分会被自动裁剪
 */
void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                     uint16_t color);

/**
 * @brief 填充整个屏幕为指定颜色
 * @param color RGB565 颜色值
 */
void ST7735_FillScreen(uint16_t color);

/**
 * @brief 在指定坐标绘制一个字符（5x7 字体，带缩放）
 * @param x          字符左上角横坐标
 * @param y          字符左上角纵坐标
 * @param c          要绘制的 ASCII 字符
 * @param foreground 前景色 RGB565
 * @param background 背景色 RGB565
 * @param scale      缩放倍数（1 = 原始 6x8 像素, >=1）
 */
void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t foreground,
                     uint16_t background, uint8_t scale);

/**
 * @brief 绘制字符串（5x7 字体，支持换行符 '\n'）
 * @param x          字符串起始横坐标
 * @param y          字符串起始纵坐标
 * @param text       以 '\0' 结尾的字符串
 * @param foreground 前景色 RGB565
 * @param background 背景色 RGB565
 * @param scale      缩放倍数
 */
void ST7735_DrawString(uint16_t x, uint16_t y, const char *text,
                       uint16_t foreground, uint16_t background,
                       uint8_t scale);

/**
 * @brief 将 8-8-8 RGB 分量转换为 RGB565 格式
 * @param red   红色分量 (0-255)
 * @param green 绿色分量 (0-255)
 * @param blue  蓝色分量 (0-255)
 * @return RGB565 编码的 16 位颜色值
 */
uint16_t ST7735_Color565(uint8_t red, uint8_t green, uint8_t blue);

#endif /* ST7735_H_ */
