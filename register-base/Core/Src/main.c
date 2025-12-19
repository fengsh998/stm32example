#include "main.h"



int main(void)
{
	//系统时钟配置
	SystemClock_Config();
	//滴塔时钟初始化
	SysTick_Init_ms();

	//GPIO 使能配置
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//清除 PC13 的 4 个配置位，设置为浮空输入 (0000)
	GPIOC->CRH   &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
	GPIOC->CRH   |= GPIO_CRH_MODE13_1;   // 10MHz 输出推挽

	while (1)
	{
		GPIOC->BSRR = GPIO_BSRR_BR13;   // LED 灭
		delay_ms(500);
		GPIOC->BSRR = GPIO_BSRR_BS13;   // LED 亮
		delay_ms(500);
	}
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
