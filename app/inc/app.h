/**
 * @file    app.h
 * @brief   应用层 API
 *
 * 应用层负责：
 *   - WT61 DMA 数据解析
 *   - 编码器中断快照后的 RPM 更新
 *   - VOFA 100 Hz DMA 遥测
 *   - ST7735 5 Hz 低优先级局部刷新
 *
 * 调用时序：Board_Init() -> App_Init() -> 主循环中反复调用 App_RunOnce()
 */

#ifndef APP_H_
#define APP_H_

void App_Init(void);    /**< 初始化电机、编码器、WT61、VOFA 和屏幕 */

void App_RunOnce(void); /**< 主循环反复调用：运行各个非阻塞协作式任务 */

#endif /* APP_H_ */
