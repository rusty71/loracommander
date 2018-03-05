initial project setup

Used Atmel Start as base.
Added C++ support
Added tickless support for ARM_CM0 from STM32FreeRTOS
Tickless is working with Systick so 2.87Hz maximum wake up. Need to change to 32 bit RTC to achieve lower wakeup frequencies 
Tickless with USB makes it wake up 1kHz. Need to disable USB in low power mode
