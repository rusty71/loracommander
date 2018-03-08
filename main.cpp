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

void TC3_Handler(void){
    static bool toggle = true;
    
    if(toggle){
        toggle = false;
        gpio_set_pin_level(LED_YELLOW,true);
    }
    else {
        toggle = true;
        gpio_set_pin_level(LED_YELLOW,false);
    }
    hri_tc_clear_INTFLAG_MC0_bit(TC3);

    //~ hri_tccount32_write_CC_reg(TC3, 1 ,0x0); /* Compare/Capture Value: 0x0 */

    //~ NVIC_ClearPendingIRQ(TC3_IRQn);
    //~ hri_tc_clear_INTEN_MC0_bit(TC3);
    //~ hri_tc_write_CTRLA_ENABLE_bit(TC3, 1 << TC_CTRLA_ENABLE_Pos); /* Enable: enabled */
	//~ hri_tc_write_INTEN_reg(TC3,
	                       //~ 1 << TC_INTENSET_MC0_Pos       /* Match or Capture Channel 0 Interrupt Enable: enabled */
	                           //~ | 0 << TC_INTENSET_MC1_Pos /* Match or Capture Channel 1 Interrupt Enable: disabled */
	                           //~ | 0 << TC_INTENSET_SYNCRDY_Pos /* Synchronization Ready Interrupt Enable: disabled */
	                           //~ | 0 << TC_INTENSET_ERR_Pos     /* Error Interrupt Enable: disabled */
	                           //~ | 0 << TC_INTENSET_OVF_Pos);   /* Overflow Interrupt enable: disabled */


    
}

}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
    system_init();


    //~ HWTIMER_init();
    
    while(1)
        ;
    ASSERT(true);
    
    start_l2console();

	vTaskStartScheduler();
}

//https://github.com/yorickdewid/Chat-Server/blob/master/chat_server.c
//https://github.com/andregasser/chatsrv
//https://www.beartooth.com/
//https://www.kickstarter.com/projects/sonnet/sonnet-decentralized-mobile-communication/
//https://www.thethingsnetwork.org/forum/t/lorawan-pager-project/6992
