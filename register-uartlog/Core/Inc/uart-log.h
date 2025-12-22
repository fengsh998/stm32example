#ifndef INC_UART_LOG_H_
#define INC_UART_LOG_H_

#include "stm32f103xb.h"
#include <stdio.h>
#include <stdarg.h>

void UART1_Init(uint32_t baudrate);
void my_log(const char *format, ...);

#endif /* INC_UART_LOG_H_ */
