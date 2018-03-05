#include <atmel_start.h>

extern "C" {
void pre_sleep(TickType_t time) {
    gpio_set_pin_level(LED_YELLOW,false);
}

void post_sleep(TickType_t time) {
    gpio_set_pin_level(LED_YELLOW,true);
}
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

    cdcd_acm_example();

    FREERTOS_example();

	/* Replace with your application code */
	while (1) {
	}
}
