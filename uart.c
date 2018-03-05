#include <stdlib.h>
#include "driver_init.h"
//~ #include "utils.h"
#include "uart.h"

void UART_write(uint8_t *data, int len)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&UART, &io);
	usart_sync_enable(&UART);

	io_write(io, data, len);
}

void UART_print(char *str){
    int len = strlen(str);
    UART_write((uint8_t *)str, len);    
}

void UART_println(char *str) {
    UART_print(str);
    UART_write((uint8_t *)"\n\r", 2);
}

void UART_printInt(int data) {
    char intbuf[10];
    
    itoa(data, intbuf, 16);
    UART_print(intbuf);
}


