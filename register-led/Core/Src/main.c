#include "main.h"

// GPIO初始化
void gpio_init(void) {
	// 1. 使能GPIOB时钟
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	// 2. 配置PB4-PB7为推挽输出，速度50MHz
	// GPIOB_CRL寄存器配置引脚0-7，每个引脚4位
	// 模式：11 = 输出模式，最大速度50MHz
	// 配置：00 = 通用推挽输出模式
	// 所以每个引脚配置为：0011 (0x3)

	// 清除PB4-PB7的配置位
	GPIOB->CRL &= ~(0xFFFF << (4 * 4));  // 清除PB4-PB7的16位

	// 设置PB4-PB7为推挽输出，50MHz
	GPIOB->CRL |= (0x33 << (4 * 4));     // PB4, PB5 = 0011
	GPIOB->CRL |= (0x33 << (6 * 4));     // PB6, PB7 = 0011

	// 初始状态：所有LED熄灭（高电平或低电平取决于硬件设计）
	// 假设LED低电平点亮，初始设为高电平熄灭
	GPIOB->ODR |= (0xF << 4);  // PB4-PB7全部设为高电平
}

int main(void) {
	//系统时钟配置
	SystemClock_Config();
	//滴塔时钟初始化
	SysTick_Init_ms();

	//GPIO 使能配置
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//清除 PC13 的 4 个配置位，设置为浮空输入 (0000)
	GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
	GPIOC->CRH |= GPIO_CRH_MODE13_1;   // 10MHz 输出推挽

	// 初始化GPIO B区
	gpio_init();

	// 流水灯模式数组
	uint8_t led_patterns[] = { 0x0E,  // 1110 - PB4亮
			0x0D,  // 1101 - PB5亮
			0x0B,  // 1011 - PB6亮
			0x07,  // 0111 - PB7亮
			0x0F,  // 1111 - 全灭
			0x00,  // 0000 - 全亮
			0x0A,  // 1010 - PB4,PB6亮
			0x05   // 0101 - PB5,PB7亮
			};

	while (1) {
		GPIOC->BSRR = GPIO_BSRR_BR13;   // LED 灭
		delay_ms(500);
		GPIOC->BSRR = GPIO_BSRR_BS13;   // LED 亮
		delay_ms(500);

		// 循环显示各种流水灯模式
		for (int i = 0; i < sizeof(led_patterns); i++) {
			// 清除PB4-PB7
			GPIOB->ODR &= ~(0xF << 4);

			// 设置新的LED状态
			GPIOB->ODR |= (led_patterns[i] << 4);

			// 延时
			delay_ms(300);
		}
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
