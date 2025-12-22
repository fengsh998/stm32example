#ifndef INC_OLED_U8G2_H_
#define INC_OLED_U8G2_H_

#include "u8g2.h"
#include <math.h>  // 需要 sin() 函数

// 初始化
void oled_u8g2_init(void);
// 清屏
void clear(void);
/**
 * @param line 输出的行偏移，在128*64的屏里，u8x8里行只有8行
 */
void oled_log(uint8_t line, const char* format, ...);

//============ Demo ==========
//示例一：显示hello world
void testHelloWorld();
/*
 * 示例二：显示折线图
 	 main {
 	 	 ...
 	 	 int counter = 0;
 	 	 while (1) {
 	 	 	 // 更新模拟数据（比如正弦波或随机数）
			update_data(20 + (counter % 15));
			// 绘制图表
			draw_chart_demo();
			counter++;
			delay_ms(100); // 采样频率
 	 	 }
 	 }
 */
void update_data(uint8_t new_val);
void draw_chart_demo();

//示例三：柱状图（Bar Chart）
void draw_bar_chart(void);
//示例4：绘制正弦波（Sine Wave）
void draw_sine_wave(void);

//示例5：实时温度变化折线图（监测曲线）
/**
 * // 初始化数据
    for (int i = 0; i < DATA_POINTS; i++) temp_data[i] = 25.0;

    while (1) {
        float temp = 25.0 + 10 * sin((float)data_index / 20);  // 模拟温度变化
        update_temperature_chart(temp);
        delay_ms(200);  // 每 200ms 更新一次
    }
 */
#define DATA_POINTS 128  // 使用全屏宽度

extern float temp_data[DATA_POINTS];  // 存储温度历史（可从 ADC 获取）
extern int data_index;
void update_temperature_chart(float new_temp);

#endif /* INC_OLED_U8G2_H_ */
