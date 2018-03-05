#include "FreeRTOS.h"
#include "task.h"
#include "driver_init.h"
#include "cdc_drv.h"
#include "uart.h"
#include "rtos_start.h"

extern "C" {
void pre_sleep(TickType_t *time) {
    gpio_set_pin_level(LED_YELLOW,false);
}

void post_sleep(TickType_t *time) {
    gpio_set_pin_level(LED_YELLOW,true);
}
}


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	system_init();
    UART_println((char *)"UART test");
    cdc_init();

    FREERTOS_example();

	/* Replace with your application code */
	while (1) {
	}
}
