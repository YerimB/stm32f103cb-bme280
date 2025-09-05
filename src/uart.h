#ifndef UART_H
#define UART_H

#include "stm32f103xb.h"

void uart_init(void);
void uart_send_char(char c);
void uart_print_uint(uint32_t n);
void uart_print_str(const char *str);

#endif // UART_H