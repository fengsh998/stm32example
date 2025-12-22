#include "main.h"
#include "oled-u8g2.h"
#include "uart-log.h"

int main(void) {
	//系统时钟配置
	SystemClock_Config();
	//滴塔时钟初始化
	SysTick_Init_ms();

	UART1_Init(115200);

	// 3. 初始化 OLED (SSD1306 128x64)
	oled_u8g2_init();

	//GPIO 使能配置
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//清除 PC13 的 4 个配置位，设置为浮空输入 (0000)
	GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
	GPIOC->CRH |= GPIO_CRH_MODE13_1;   // 10MHz 输出推挽

//	testHelloWorld();
//	clear();
//	int counter = 0;

	draw_sine_wave();
//	for (int i = 0; i < DATA_POINTS; i++) temp_data[i] = 25.0;

	while (1) {
		GPIOC->BSRR = GPIO_BSRR_BR13;   // LED 灭
		delay_ms(500);
		GPIOC->BSRR = GPIO_BSRR_BS13;   // LED 亮
		delay_ms(500);

		my_log("Hello Mac!\r\n");
		delay_ms(1000);

//		float temp = 25.0 + 10 * sin((float)data_index / 20);  // 模拟温度变化
//		        update_temperature_chart(temp);
//		        delay_ms(200);  // 每 200ms 更新一次

//		// 更新模拟数据（比如正弦波或随机数）
//		update_data(20 + (counter % 15));
//		// 绘制图表
//		draw_chart_demo();
//
//		oled_log(1, "System Running...");
//		oled_log(2, "Val: %d", counter);
//
//		counter++;
//		delay_ms(100); // 采样频率
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
