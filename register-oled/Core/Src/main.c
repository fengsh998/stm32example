#include "main.h"
#include "oled/spi-oled.h"

void oled_demo1() {
	oled_clear();
	// 第一行写超长文本（前后加空格更美观）
	oled_set_position(0, 0);
	my_printf("    Welcome to STM32 OLED! Long scrolling message demo...    ");

	// 其他行写固定内容
	oled_set_position(0, 2);
	my_printf("Temperature: 25.6 C");
	oled_set_position(0, 4);
	my_printf("Humidity:    60%");
	oled_set_position(0, 6);
	my_printf("Status:      OK");

	// 只滚动第一行（page 0）
	oled_set_vertical_scroll_area(8, 48);        // 顶部固定1行（8像素），剩余可滚动
	oled_scroll_left(0, 0, 0x07);                // 左滚，最快速度
}

void oled_demo2() {
	oled_clear();
	my_printf("Line 1: Hello");
	my_printf("Line 2: STM32");
	my_printf("Line 3: SSD1306");
	my_printf("Line 4: Scroll Demo");
	my_printf("Line 5: 2025-12-19");
	my_printf("Line 6: Running...");
	my_printf("Line 7: End");

	oled_scroll_left(0, 7, 0x00);  // 全屏左滚，中等速度
}

void oled_demo3() {
	oled_clear();
	my_printf("Line 1: Hello");
	my_printf("Line 2: STM32");
	my_printf("Line 3: SSD1306");
	my_printf("Line 4: Scroll Demo");
	my_printf("Line 5: 2025-12-19");
	my_printf("Line 6: Running...");
	my_printf("Line 7: End");

	oled_scroll_right(0, 7, 0x07);  // 全屏右滚，最快
	delay_ms(10000);
	oled_scroll_stop();            // 10秒后停止
}

void oled_demo4() {
	oled_clear();
	my_printf("Line 1: Hello");
	my_printf("Line 2: STM32");
	my_printf("Line 3: SSD1306");
	my_printf("Line 4: Scroll Demo");
	my_printf("Line 5: 2025-12-19");
	my_printf("Line 6: Running...");
	my_printf("Line 7: End");
	oled_scroll_diag_left(0, 7, 0x07, 10);  // 左+向下滚动，垂直偏移10
}

int main(void) {
	//系统时钟配置
	SystemClock_Config();
	//滴塔时钟初始化
	SysTick_Init_ms();

	mcu_init();
	oled_init();

	oled_demo1();
	//	oled_demo2();
	//	oled_demo3();
	//	oled_demo4();

	//GPIO 使能配置
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//清除 PC13 的 4 个配置位，设置为浮空输入 (0000)
	GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
	GPIOC->CRH |= GPIO_CRH_MODE13_1;   // 10MHz 输出推挽

	while (1) {
		GPIOC->BSRR = GPIO_BSRR_BR13;   // LED 灭
		delay_ms(500);
		GPIOC->BSRR = GPIO_BSRR_BS13;   // LED 亮
		delay_ms(500);
	}
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
