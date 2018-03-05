#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED
#include <stdint.h>
#include <string.h>

//~ #define DEBUG_PRINT(x) UART_println(x)
 
#ifdef __cplusplus
extern "C" {
#endif

void UART_write(uint8_t *data, int len);
void UART_print(char *str);
void UART_printInt(int data);
void UART_println(char *str);

#ifdef __cplusplus
}
#endif
#endif // UART_H_INCLUDED
