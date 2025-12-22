/*
 *
 *      STM32F103C8T6 + OLED(SSD1306) + SPI协议
 *
 *      使用硬件IO进行实现
 *
 *      硬件引脚：
 *      SPI1: PA5 (SCK), PA7 (MOSI)  芯片本身默认功能脚
 * 		CS: PA4。
 *		DC: PA3。
 *		RST: PA2。
 */

#ifndef INC_SPI_OLED_H_
#define INC_SPI_OLED_H_

//如果定义了使用硬件GPIO口
#define USE_HARDWARE_SPI 	0

#include "stm32f103xb.h"
#include "clock/systick.h"

#if USE_HARDWARE_SPI
// 硬件 SPI协议 Pin definitions
#define OLED_CS_GPIO GPIOA
#define OLED_CS_PIN 4
#define OLED_DC_GPIO GPIOA
#define OLED_DC_PIN 3
#define OLED_RST_GPIO GPIOA
#define OLED_RST_PIN 2
#else
// 软件 实现 SPI协议 Pin definitions
#define OLED_CS_GPIO GPIOA
#define OLED_CS_PIN 4
#define OLED_DC_GPIO GPIOA
#define OLED_DC_PIN 3
#define OLED_RST_GPIO GPIOA
#define OLED_RST_PIN 2
#define OLED_SCK_GPIO GPIOA
#define OLED_SCK_PIN 5
#define OLED_MOSI_GPIO GPIOA
#define OLED_MOSI_PIN 7
#endif


void mcu_init();
void oled_init();
//全亮，试屏用
void oled_fill_white();
//全灭，清屏
void oled_clear();
void oled_set_position(uint8_t column, uint8_t page);
// 每次偏移从0，0开始输出 eg: my_printf("Hello %s! Number: %d", "World", 123);
void my_printf(const char *fmt, ...);
// 跟随输出
void oled_write_string(const char *str);
void oled_write_char(char ch);

/**
 * 滚动函数
 * 滚动是硬件持续进行的，除非调用 oled_scroll_stop()，否则一直滚。
 * 如果要更新内容，建议先 oled_scroll_stop() → 清屏/重写 → 再启动滚动。
 * 滚动期间仍然可以写入新数据（会实时反映）。
 */
void oled_scroll_stop(void);
void oled_scroll_left(uint8_t start_page, uint8_t end_page, uint8_t speed);
void oled_scroll_right(uint8_t start_page, uint8_t end_page, uint8_t speed);
void oled_set_vertical_scroll_area(uint8_t top_fixed_rows, uint8_t scroll_rows);
void oled_scroll_diag_right(uint8_t start_page, uint8_t end_page, uint8_t speed, uint8_t vertical_offset);
void oled_scroll_diag_left(uint8_t start_page, uint8_t end_page, uint8_t speed, uint8_t vertical_offset);

#endif /* INC_SPI_OLED_H_ */
