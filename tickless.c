/**
 * \file
 *
 * \brief FreeRTOS demo application tick setup function and tickless function
 *
 * Copyright (C) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "atmel_start.h"

/**
 *
 * The tickless feature of FreeRTOS allows the device to sleep for peroids
 * longer than the predefined OS tick rate. This is useful when no tasks
 * are ready to execute for more than one OS tick. The benefit
 * of this is lower power consumption.
 *
 * The tickless feature is implemented using a timer, configured with the
 * desired timeout value, to wake the device. The same timer is also used to
 * generate the system tick, so that time is kept in the optimal way,
 * eliminating drift in most cases. If some other source wakes the device
 * before the sleep period is complete, but after more than one OS tick,
 * there will be a slight drift, as the timers count value must be corrected.
 *
 */

/* Frequency of timer */
#define TIMER_HZ (1024)

/* Value per os tick of timer */
#define TIMER_RELOAD_VALUE_ONE_TICK (TIMER_HZ / configTICK_RATE_HZ)
#define TIMER_RELOAD_VALUE_ONE_TICK 1

/* One tick init for 1 ms */
#define TIMER_INTERVAL_TICK 1

/*  Maximum value of timer */
#define TIMER_MAX_COUNT (0xffffffff)

/* Maximum possible suppressed ticks with timer */
//~ #define TIMER_MAX_POSSIBLE_SUPPRESSED_TICKS (TIMER_MAX_COUNT / TIMER_RELOAD_VALUE_ONE_TICK)
#define TIMER_MAX_POSSIBLE_SUPPRESSED_TICKS 0xffffffff

/* External declaration of freeRTOS SysTick handler */
extern void xPortSysTickHandler(void);

/* Function for setting up timer */
void vPortSetupTimerInterrupt(void);

/* Tickless mode control variable */
bool volatile tickless_mode_enable = true;

struct timer_task TIMER_task;

/*
 * \Example of using TIMER
 *
 * \param[in] timer_task Pointer to timer_task structure
 */
static void TIMER_systick_cb(const struct timer_task *const timer_task)
{
	xPortSysTickHandler();
}

/*
 * \Prototype for empty_callback for sleep timer
 *
 * \param[in] timer_task Pointer to timer_task structure
 */
static void TIMER_empty_cb(const struct timer_task *const timer_task)
{
}

/*
 * \brief Initialize and start timer for tick
 *
 * Function that sets up a timer to use for os tick. The same timer is also
 * used as the sleep timer.
 * The function is weakly defined in freeRTOS, and redefined here.
 */
void vPortSetupTimerInterrupt(void)
{
	TIMER_task.interval = TIMER_INTERVAL_TICK;
	TIMER_task.cb       = TIMER_systick_cb;
	TIMER_task.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER, &TIMER_task);
	timer_start(&TIMER);
}

/*
 * \brief Configure sleep timer and sleep
 *
 * Function to configure timer for sleep, and calculate time slept.
 *
 * \param[in] xExpectedIdleTime the number of ticks task want to sleep.
 */
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    TickType_t xModifiableIdleTime;

	/* Are we running tickless now ? */
	if (!tickless_mode_enable) {
		return;
	}

	/* Reconfigure the timer to act as sleep timer */
	TIMER_task.cb = TIMER_empty_cb;

	/* Check that the offset is not greater than the range of the timer */
	if (xExpectedIdleTime > TIMER_MAX_POSSIBLE_SUPPRESSED_TICKS) {
		xExpectedIdleTime = TIMER_MAX_POSSIBLE_SUPPRESSED_TICKS;
	}

	/* Set sleep time, -1 because we want to wake up before the last tick */
	TIMER_task.interval = (xExpectedIdleTime - 1) * TIMER_INTERVAL_TICK;

	/* Check if we still should sleep */
	if (eTaskConfirmSleepModeStatus() == eAbortSleep) {
		/* Reset the timer to act as SysTick */
		TIMER_task.cb       = TIMER_systick_cb;
		TIMER_task.interval = TIMER_INTERVAL_TICK;
	} else {
		if (xExpectedIdleTime > 0) {
            xModifiableIdleTime = xExpectedIdleTime;
            configPRE_SLEEP_PROCESSING( &xModifiableIdleTime );


			/* Data sync barrier before sleep */
			__DSB();
			/* Go to sleep */
			__WFI();

			configPOST_SLEEP_PROCESSING( &xExpectedIdleTime );

			/* Reset counter to less than one os tick */
			vTaskStepTick(TIMER.time);
			TIMER.time = 0;
		}
		/* Reset the timer to act as SysTick */
		TIMER_task.cb       = TIMER_systick_cb;
		TIMER_task.interval = TIMER_INTERVAL_TICK;

		/* Make sure that the counter hasn't passed the CC before callback was registered */
		if (TIMER.time > TIMER_INTERVAL_TICK) {
			/* If so, reload count value, and step one tick	*/
			vTaskStepTick(1);
		}
	}
}
