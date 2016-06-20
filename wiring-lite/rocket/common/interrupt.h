/* interrupt.h - Arduino lite interrupt header file*/

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

#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#if defined(CONFIG_STDOUT_CONSOLE)
  #include <stdio.h>
#else
  #include <misc/printk.h>
#endif

/*
 * There are multiple tasks doing printfs and they may conflict.
 * Therefore use puts() instead of printf().
 */
#if defined(CONFIG_STDOUT_CONSOLE)
  #define PRINTF(...) {char output[256]; sprintf(output, __VA_ARGS__); puts(output); }
#else
  #define PRINTF(...) printk(__VA_ARGS__)
#endif


#define INT0                0
#define INT1                1
#define PIN_INT0            3
#define PIN_INT1            4
#define NUM_OF_INTS         2
#define INVALID_INTERRUPT   -1


/* Translate the actual digital pin to the specific interrupt number */
#define digitalPinToInterrupt(pin)  ((pin) == PIN_INT0 ? INT0 : \
                                    ((pin) == PIN_INT1 ? INT1 : INVALID_INTERRUPT))
/* Translate the actual interrupt number to the specific digital pin  */
#define interruptToDigitalPin(pin)  ((pin) == INT0 ? PIN_INT0 : \
                                    ((pin) == INT1 ? PIN_INT1 : INVALID_INTERRUPT))


/* variables for polled interrupt */
typedef struct interrupt_action_t {
    bool isAttached;
    uint32_t id;
    void (*pHandler)(void);
    uint32_t mode;
    int laststate;
} intAction_t;

/* general polled interrupt control */
typedef struct interrupt_description_t {
    bool isEnabled;
    intAction_t ints[NUM_OF_INTS];
} intDescrt_t;

#endif /* _INTERRUPT_H_ */