#ifndef MYASSERT_H_INCLUDED
#define MYASSERT_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

//~ #include "driver_init.h"
//~ #include "rtos_start.h"
//~ #include "usb_start.h"

extern void debug_print( const char* format, const char* file, const int line, const char* func);

#define DEBUG_PRINT(x) debug_print(x,  __FILE__, __LINE__, __func__)

//~ void vAssertCalled( char *file, char* line );
void myvAssertCalled( const char *file, int line );


#ifdef __cplusplus
}
#endif
#endif
