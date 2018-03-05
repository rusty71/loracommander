/*
 * Code copied from usb_start.h
 */
#ifndef L2_CONSOLE_MAIN_H
#define L2_CONSOLE_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void l2console_task(void *p);
void start_l2console(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // L2_CONSOLE_MAIN_H
