#include <assert.h>
#include <stdio.h>
#include "uart.h"



//~ static buf[100];

//~ void debug_print(char *fmt, ...) {
    //~ do {
        //~ if (DEBUG) {
            //~ sprintf(buf, "%s:%d:%s(): ", fmt, __FILE__, __LINE__, __func__, __VA_ARGS__);
            //~ UART_println(buf);
        //~ }
    //~ } while (0);
//~ }

void myvAssertCalled( const char *file, int line ) {
    UART_print((char *)file);
    UART_print(":");    
    UART_printInt(line);    
    UART_println(":");    
}

void debug_print( const char* format, const char* file, int line, const char* func)
{
    UART_print((char *)file);
    UART_print(":");    
    UART_print((char *)func);
    UART_print(":");    
    UART_printInt(line);    
    UART_print(":");    
    UART_println((char *)format);    
}
