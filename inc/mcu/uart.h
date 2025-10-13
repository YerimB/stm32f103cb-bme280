#ifndef UART_H
#define UART_H

#include "stm32f103xb.h"

void uart_init(void);
void uart_send_char(const char c);
void uart_print_str(const char *str);
void uart_print_int(int32_t n);
void uart_print_hex(int32_t n);

#endif // UART_H