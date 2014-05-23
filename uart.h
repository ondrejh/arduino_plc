/** uart module header */

#ifndef __UART_H
#define __UART_H

void sprint_int(char* str, int i);

/// uart initialization
void init_uart(void);

/// uart putchar function
int8_t uart_putchar(uint8_t c);

/// uart printf function
uint8_t uart_puts(char* str);

void strcp(char* deststr, char* str);
int strlen(char* str);

#endif
