#include "uart-log.h"
#include <string.h> // 必须引用，用于 memset 和 strlen
#include "clock/systick.h"

/**
 *		需要硬件USB转TTL 然后安下面的连接，电脑串口通讯软件mac下的话使用coolterm
 *
 *
 * 		PA9(TX)           RX
 * 		PA10(RX)          TX
 * 		GND               GND
 *      3.3V			  3.3V (如果版子有供电，则此线可不接)
 *
 */

// 初始化 UART1 (PA9=TX, PA10=RX)
void UART1_Init(uint32_t baudrate) {
	// 1. 使能时钟
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

	// 2. 初始化引脚前，先给 PA9 (TX) 一个确定的高电平（闲置电平）
	// 这样切换到复用模式时，电平是从高到高，不会产生起始位毛刺
	GPIOA->BSRR = (1 << 9);

	// 3. 配置 PA9 (TX) 为复用推挽 (0xB)
	GPIOA->CRH = (GPIOA->CRH & ~(0xF << 4)) | (0xB << 4);
	// 配置 PA10 (RX) 为浮空输入 (0x4)
	GPIOA->CRH = (GPIOA->CRH & ~(0xF << 8)) | (0x4 << 8);

	// 4. 设置波特率 (假设 72MHz)
	USART1->BRR = 72000000 / baudrate;

	// 5. 先使能串口，但不立即发送
	USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

	// 6. 重要：等待一小会儿让电平稳定
	delay_ms(10);

	// 7. 清除可能存在的标志位
	(void) USART1->SR;
	(void) USART1->DR;
}

// 发送单个字符
void UART_SendChar(char c) {
	while (!(USART1->SR & USART_SR_TXE))
		; // 等待发送寄存器空
	USART1->DR = c;
}

// 发送字符串 (补充到你的 my_log 中)
void UART_SendString(const char *str) {
	while (*str) {
		UART_SendChar(*str++);
	}
}

void my_log(const char *format, ...) {
	char buffer[128]; // 根据内存需求调整大小
	va_list args;
	va_start(args, format);
	// 格式化字符串
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	// UART 发送字符串的函数
	UART_SendString(buffer);
//	UART_SendString("\r\n"); // 串口换行符
	// 显式发送十六进制码，确保顺序正确
	UART_SendChar(0x0D); // \r
	UART_SendChar(0x0A); // \n
}

