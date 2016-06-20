/* interrupt.c - Implementation of Arduino Interrupts and External Interrupts API */

/*
 * Copyright (c) 2015, Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <zephyr.h>
#include "interrupt.h"
#include <arch/cpu.h>	/* irq_lock/irq_unlock */


#define SLEEPTIME  10
#define SLEEPTICKS (SLEEPTIME * sys_clock_ticks_per_sec / 1000)

/* interrupt action descriptor */
volatile intDescrt_t intDescrt = {
    true,
    { {false, -1, NULL, LOW, LOW},
    {false, -1, NULL, LOW, LOW} }
};


/*******************************************************************************
* interrupts() - Re-enables interrupts. Init or after they've been disabled by
*                noInterrupts().
*
* Arguments: none
*
* Returns:   None
*
* Syntax:   interrupts()
*/
void interrupts(void)
{
    intDescrt.isEnabled = true;
}


/*******************************************************************************
* noInterrupts() - Disables interrupts. Can re-enable them with interrupts()
*
* Arguments: none
*
* Returns:   None
*
* Syntax:   noInterrupts()
*/
void noInterrupts(void)
{
  intDescrt.isEnabled = false;
}

/*******************************************************************************
* attachInterrupt() - Attach digital Pins With Interrupts
*
* Arguments: int - interrupt : the number of the interrupt
*            callback : the ISR to call when the interrupt occurs
*            int - mode : define which modes should trigger the interrupt
*               LOW, to trigger the interrupt whenever the pin is low
*               CHANGE, to trigger the interrupt whenever the pin changes value
*               RISING, to trigger when the pin goes from low to high
*               FALLING, for when the pin goes from high to low.
*
* Returns:  none
*
* Syntax:   attachInterrupt(interrupt, ISR, mode)
*
* Note:
*       Board           Int0  Int1
*       ---------------------------
*       Galileo Gen2    3     4   (Digital Pins)
*
*/
void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
{
    if ((pin != INT0) && (pin != INT1)) {
        return;
    }

    intDescrt.ints[pin].isAttached = true;
    intDescrt.ints[pin].id = pin;
    intDescrt.ints[pin].pHandler = callback;
    intDescrt.ints[pin].mode = mode;
    intDescrt.ints[pin].laststate = 0;
}

/*******************************************************************************
* detachInterrupt() - Turns off the given interrupt
*
* Arguments: int - interrupt : the number of the interrupt to disable
*
* Returns:  none
*
* Syntax:   detachInterrupt(uint32_t pin)
*/
void detachInterrupt(uint32_t pin)
{
    if ((pin != INT0) && (pin != INT1)) {
        return;
    }

    intDescrt.ints[pin].isAttached = false;
    intDescrt.ints[pin].id = INVALID_INTERRUPT;
    intDescrt.ints[pin].pHandler = NULL;
    intDescrt.ints[pin].mode = 0;
    intDescrt.ints[pin].laststate = 0;
}


/*
 * Entry point to a interrupt monitoring thread
 * This routine runs as a task in the microkernel environment
 */
void intMonitor(void)
{
	static int i, pinValue, lastState;

	while (1) {

		if (intDescrt.isEnabled) {
			//task_mutex_lock_wait(MT_MUT);
			task_mutex_lock(MT_MUT, TICKS_UNLIMITED);

        	for (i = 0; i < NUM_OF_INTS; i++)	{
            	if (!intDescrt.ints[i].isAttached)
                	continue;

            	pinValue = digitalRead(interruptToDigitalPin(i));
            	lastState = intDescrt.ints[i].laststate;

            	/* state changes, rising or falling */
            	if (pinValue != lastState)    {
                	if ((intDescrt.ints[i].mode == CHANGE) ||
                    	((lastState == LOW) && (intDescrt.ints[i].mode == RISING)) ||
                    	((lastState == HIGH) && (intDescrt.ints[i].mode == FALLING)))   {
                    	intDescrt.ints[i].pHandler();

                    	PRINT("I%d ", i);
                	}
            	}
	            else {	/* state unchanges, high or low */
	                if (((lastState == LOW) && (intDescrt.ints[i].mode == LOW)) ||
	                    ((lastState == HIGH) && (intDescrt.ints[i].mode == HIGH)))    {
	                    intDescrt.ints[i].pHandler();

	                    PRINT("I%d ", i);
	                }
	            }

            	intDescrt.ints[i].laststate = pinValue;
			} /* end for-loop */
			task_mutex_unlock(MT_MUT);
		} else {
			task_yield();
		}
		task_sleep(SLEEPTICKS);
	}
}