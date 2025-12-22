#include "clock/osc_clock.h"
#include "stm32f1xx.h"

/**
 * HSE 和 LSE 的配置主要完成了以下任务：
	HSE (高速外部晶振) 配置：
	启动并稳定外部高速晶振（如 8 MHz）。
	通过 PLL（锁相环） 将时钟频率倍增（例如到 72 MHz），形成 SYSCLK (系统时钟)。
	配置 AHB, APB1, APB2 三条主要总线的分频系数，确定了 CPU、SRAM、DMA 以及大部分外设的最高工作频率。
	LSE (低速外部晶振) 配置：
	启动并稳定外部低速晶振（32.768 kHz）。
	将该时钟源分配给 RTC（实时时钟）。

	如果需要使有外设还需要：
	使能外设时钟
		这是最重要的一步。即使系统时钟已经启动，外设本身的时钟门控还是关闭的，以节省功耗。
		配置步骤： 必须写入 RCC 的使能寄存器：
		APB2 外设： 例如 GPIOA, USART1, SPI1 等，需要置位 RCC_APB2ENR 寄存器中对应的位。
		APB1 外设： 例如 USART2, Timers 2-7, I2C1 等，需要置位 RCC_APB1ENR 寄存器中对应的位。
		AHB 外设： 例如 DMA, FLASH, SRAM 等，需要置位 RCC_AHBENR 寄存器中对应的位。
		示例： 如果要使用 GPIOA，必须设置 RCC->APB2ENR |= (1U << 2);（Bit 2 是 IOPFEN，即 GPIOA 时钟使能）。
 */
void SystemClock_Config(void)
{
	SystemClock_HSE_Config();
	SystemClock_LSE_Config();
}

// 8MHz 晶振时钟
void SystemClock_HSE_Config(void) {
	//外部高速晶振配置(8MHz)
	//1、使能HSE 设置CR寄存器第16位为1时开启外部时钟
	RCC->CR |= RCC_CR_HSEON; // 或直接写(1U << 16);
	//并等待HSE准备就绪
	while (!(RCC->CR & RCC_CR_HSERDY)); // 或 while (!(RCC->CR & (1U << 17)));

	//2、配置FLASH延迟（对于 72 MHz 的系统时钟，通常需要设置 2 个等待周期）
	// 设置等待周期(Latency)
	FLASH->ACR &= ~((uint32_t)0x07); //清除LATENCY,Bit[2:0]
	FLASH->ACR |= (uint32_t)0x02; //设置为2个等待周期,48MHz < SysClock < 72MHz
	// 设置使能预取缓冲区PRFTBE:(Prefetch Buffer) = 1
	FLASH->ACR |= (1U << 4);

	//3、配置分频器(CFGR),AHB，APB1，APB2
	RCC->CFGR &= ~(0xFU << 4);   //AHB(HCLK) 不分频
	// 先清除 PPRE1 位，再设置 /2 分频
	RCC->CFGR &= ~RCC_CFGR_PPRE1;
	RCC->CFGR |= (0x4U << 8);    //APB1(PCLK1) 最高36MHz 需分频
	RCC->CFGR &= ~(0x7U << 11);  //APB2(PLCK2) 最高72MHz 不分频

	//4、配置PLL(锁相环)
	//PLL 源选择
	RCC->CFGR |= (1U << 16); //选择 HSE 作为 PLL 输入源 (Bit 16),选择 PLL 源并设置倍频系数
	//HSE 分频选择
	RCC->CFGR &= ~(1U << 17); //如果 HSE 是 8 MHz，HSE 输入 PLL 前不分频 (Bit 17 = 0) 或二分频 (Bit 17 = 1)通常不分频
	//PLL 倍频选择
	RCC->CFGR |= (0x7U << 18); //设置 PLL 倍频因子 (Bits 21:18)。例如，要达到 72 MHz (8 MHz * 9), 设置倍频为 *9

	//5、使能PLL并等待其准备就绪
	RCC->CR |= (1U << 24); //使能 PLL (PLLON): 置位 RCC_CR 寄存器的 PLLON 位 (Bit 24)
	//等待 PLL 稳定 (PLLRDY): 持续检查 RCC_CR 寄存器的 PLLRDY 位 (Bit 25) 是否为 '1'
	while (!(RCC->CR & (1U << 25)));
	// 切换系统时钟源到PLL
	RCC->CFGR |= (0x2U << 0);//设置 RCC_CFGR 寄存器的 SW 位 (Bits 1:0) 为 0b10，选择 PLL 作为系统时钟 (SYSCLK)
	// 等待系统时钟切换完成
	while ((RCC->CFGR & (0x3U << 2)) != (0x2U << 2));

}

// LSE RTC(Real-Time Clock) 其标准频率为 32.768 kHz
void SystemClock_LSE_Config(void) {
	//1、使能电源和备份域时钟
	//置位 RCC_APB1ENR 寄存器的 PWREN 位 (Bit 28) 以使能电源接口时钟。这是访问备份域寄存器 (包括 RTC 和 LSE) 的前提
	RCC->APB1ENR |= (1U << 28);
	//置位 PWR_CR 寄存器的 DBP 位 (Bit 8) 以使能对 备份域 (Backup Domain) 的写访问
	PWR->CR |= (1U << 8);

	//2、使能LSE
	//置位 RCC_BDCR 寄存器的 LSEON 位 (Bit 0)
	RCC->BDCR |= (1U << 0);
	//3、等待LSE准备就绪，持续检查 RCC_BDCR 寄存器的 LSERDY 位 (Bit 1) 是否为 '1'
	while (!(RCC->BDCR & (1U << 1)));
	//4、配置RTC时钟源(可选)，选择 LSE 作为 RTC 的时钟源
	//设置 RTC 时钟源 (RTCSEL): 设置 RCC_BDCR 寄存器的 RTCSEL 位 (Bits 9:8) 为 0b01 (LSE)
	RCC->BDCR &= ~(0x3U << 8); //清除RTCSEL
	RCC->BDCR |= (0x1U << 8);  //设置为LSE

	//5、使能RTC时钟(可选)
	RCC->BDCR |= (1U << 15); //置位 RCC_BDCR 寄存器的 RTCEN 位 (Bit 15) 以使能 RTC 时钟

}

void SystemClock_LSE_Config_Def(void) {
	//1、使能电源和备份域时钟
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_DBP;
	//2、使能LSE,并等待就绪
	RCC->BDCR |= RCC_BDCR_LSEON;
	while (!(RCC->BDCR & RCC_BDCR_LSERDY));
	//3、配置RTC时钟源，使能RTC时钟
	// 清除 RTCSEL 和 RTCEN 位
	RCC->BDCR &= ~(RCC_BDCR_RTCSEL | RCC_BDCR_RTCEN);
	// 设置 RTCSEL 为 LSE，并设置 RTCEN
	RCC->BDCR |= (RCC_BDCR_RTCSEL_LSE | RCC_BDCR_RTCEN);
}

