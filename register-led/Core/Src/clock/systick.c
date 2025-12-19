#include "clock/systick.h"
#include "stm32f103xb.h"

// 定义一个全局变量，用于记录毫秒数，uint32_t最大值2^32−1=4,294,967,295
/**
 * 溢出所需时间: g_ms_ticks 每 1 毫秒 (ms) 增加 1。我们可以计算出它从 0 溢出到最大值需要多长时间即溢出时间=最大值×每步时间溢出时间=4,294,967,295 ms
 * 约为49.7天
 */
volatile uint32_t g_ms_ticks = 0;

#define RELOAD_VALUE 71999 // (72000000 / 1000) - 1

void SysTick_Init_ms(void)
{
    // 1. 设置重载值：SysTick->LOAD 寄存器
    // SysTick 是一个 24 位定时器，最大值为 0xFFFFFF
    SysTick->LOAD = RELOAD_VALUE;

    // 2. 清除当前值：SysTick->VAL 寄存器
    // 写入任何值都会清零计数器
    SysTick->VAL = 0;

    // 3. 配置控制寄存器：SysTick->CTRL 寄存器
    // (1U << 2) | (1U << 1) | (1U << 0)
    // CLKSOURCE (Bit 2) = 1：使用 AHB 时钟 (72MHz)
    // TICKINT   (Bit 1) = 1：使能 SysTick 中断
    // ENABLE    (Bit 0) = 1：使能 SysTick 定时器
    SysTick->CTRL = (1U << 2) | (1U << 1) | (1U << 0);

    // 4.设置中断优先级
    NVIC_SetPriority(SysTick_IRQn, 0);
}

/**
  * @brief  SysTick 中断服务函数
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    // 增加全局毫秒计数器
    g_ms_ticks++;
}

/**
  * @brief  毫秒级延时函数
  * @param  ms: 要延时的毫秒数 (1ms ~ 4294967295ms)
  * @retval None
  */
void delay_ms(uint32_t ms)
{
    uint32_t start_time = g_ms_ticks; // 记录延时开始时的滴答数

    // 循环等待，直到当前滴答数 - 开始滴答数 >= 设定的延时毫秒数
    // 由于 g_ms_ticks 是 volatile 变量，每次都会重新从内存读取，确保正确性
    // 由于无符号整数的数学特性，即使发生了回绕，这种减法操作（时间差）仍然是正确的，天然免疫于 SysTick 溢出
    while ((g_ms_ticks - start_time) < ms)
    {
        // 处理器进入低功耗模式或空转，等待中断
        // 在实际应用中，这里可能只是一个空循环 (NOP)，或者等待其他任务调度
    }
}

void delay_us(uint32_t us)
{
    // 假设系统时钟 72MHz（STM32F103 最常见），1us ≈ 72 次循环
    // 实测系数大概在 65~70 之间，这里取 68 最准
    uint32_t cycles = us * 68;
    while(cycles--)
        __NOP();
}

// 专门给 Proteus 用的毫秒延时（其实是“假”毫秒，但视觉上正确）
void delay_ms_proteus(uint32_t ms)
{
    // 72MHz 下，这个系数在 Proteus 里大概相当于真实 1ms
    // 你可以微调 8000 ~ 15000 之间
    volatile uint32_t count = ms * 10000;
    while(count--) __NOP();
}
