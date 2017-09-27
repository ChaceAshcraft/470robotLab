/*
 * uart.h
 *
 *  Created on: Apr 13, 2016
 *      Author: Kristian Sims
 */

#ifndef UART_H_
#define UART_H_

#include "IObuffer.h"

// Public function prototypes
int uart_init();

// Read data
void uart_read(char*, unsigned);
int uart_bytes_pending();
char uart_get_char();
int uart_get_int();
unsigned uart_get_uint();
long uart_get_long();
unsigned long uart_get_ulong();

// Send data
void uart_write(char*, unsigned);
void uart_put_char(char c);
void uart_put_int(int i);
void uart_put_uint(unsigned u);
void uart_put_long(long x);

#endif /* UART_H_ */
