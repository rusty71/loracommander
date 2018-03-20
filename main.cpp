#include "FreeRTOS.h"
#include "task.h"
#include "driver_init.h"
#include "cdc_drv.h"
#include "radio_drv.h"
#include "uart.h"
#include "l2console.h"

extern "C" {
void pre_sleep(TickType_t *time) {
    gpio_set_pin_level(LED_YELLOW,false);
}

void post_sleep(TickType_t *time) {
    gpio_set_pin_level(LED_YELLOW,true);
}

//~ void TC3_Handler(void){
    //~ static bool toggle = true;
    
    //~ if(toggle){
        //~ toggle = false;
        //~ gpio_set_pin_level(LED_YELLOW,true);
    //~ }
    //~ else {
        //~ toggle = true;
        //~ gpio_set_pin_level(LED_YELLOW,false);
    //~ }
    //~ hri_tc_clear_INTFLAG_MC0_bit(TC3);
//~ }

}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
    system_init();


    //~ HWTIMER_init();
    
    //~ while(1)
        //~ ;
    ASSERT(true);
    
    start_l2console();

	vTaskStartScheduler();
}

//https://github.com/yorickdewid/Chat-Server/blob/master/chat_server.c
//https://github.com/andregasser/chatsrv
//https://www.beartooth.com/
//https://www.kickstarter.com/projects/sonnet/sonnet-decentralized-mobile-communication/
//https://www.thethingsnetwork.org/forum/t/lorawan-pager-project/6992
