/**
 * @file OLED_SSD1309.h
 * @brief SSD1309 OLED driver (Adafruit-style API, I2C2 / HAL).
 *
 * 仅在单个 FreeRTOS 任务内调用刷屏接口；本驱动不使用 mutex。
 * 硬件: I2C2 PH4=SCL PH5=SDA
 */

/* ============ 屏幕参数（改这里） ============ */
#define SSD1309_I2C_ADDR    0x3Cu   /* 7-bit；模块标 0x78 = 本宏左移 1 位 */
#define SSD1309_WIDTH       128u
#define SSD1309_HEIGHT      64u     /* 128×64；换 32 屏改 32，init 自动适配 */

#define SSD1309_WHITE       1u
#define SSD1309_BLACK       0u
#define SSD1309_INVERSE     2u
/* ========================================== */

#ifndef OLED_SSD1309_H
#define OLED_SSD1309_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#if (SSD1309_HEIGHT != 32u) && (SSD1309_HEIGHT != 64u)
#error "SSD1309_HEIGHT must be 32 or 64"
#endif

#define SSD1309_PAGES  (SSD1309_HEIGHT / 8u)

/** 初始化 SSD1309；成功返回 1 */
uint8_t ssd1309_begin(void);

void ssd1309_clearDisplay(void);
void ssd1309_display(void);
void ssd1309_invertDisplay(uint8_t inv);

void ssd1309_setCursor(int16_t x, int16_t y);
void ssd1309_setTextSize(uint8_t size);
void ssd1309_setTextColor(uint16_t color);

void ssd1309_print(const char *s);
void ssd1309_println(const char *s);
int  ssd1309_printf(const char *fmt, ...);

void ssd1309_drawPixel(int16_t x, int16_t y, uint16_t color);
void ssd1309_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ssd1309_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ssd1309_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* OLED_SSD1309_H */
