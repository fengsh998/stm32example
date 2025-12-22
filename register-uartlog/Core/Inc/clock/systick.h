/**
 * 核心参数

	系统时钟 (HCLK/SYSCLK)： f(SYSCLK) = 72,000,000 Hz (72 MHz)
	滴答周期 (T)： 1 ms (毫秒)
	SysTick 重载值 (Reload Value)：Reload=(Clock×T)−1
	Reload=(72,000,000 Hz×0.001 s)−1=72000−1=71999
 */

#ifndef INC_SYSTICK_H_
#define INC_SYSTICK_H_

#include <stdio.h>
#include <stdint.h>
#include <errno.h>

void SysTick_Init_ms(void);
void SysTick_Handler(void);
// 延时毫秒
void delay_ms(uint32_t ms);
// 延时微秒
void delay_us(uint32_t us);

// 专门给proteus的假延时用的，因为proteus的执行上不够精确
void delay_ms_proteus(uint32_t ms);

#endif /* INC_SYSTICK_H_ */
