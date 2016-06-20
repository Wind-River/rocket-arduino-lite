/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at 
 * 
 * http://www.apache.org/licenses/LICENSE-2.0 
 * 
 * Unless required by applicable law or agreed to in writing, software distributed 
 * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES 
 * OR CONDITIONS OF ANY KIND, either express or implied. See the License for
 * the specific language governing permissions and limitations under the License. 
 */

#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#ifdef __cplusplus
typedef bool boolean;
extern "C" {
#endif

#include "zephyr.h"
#include "wiring-math.h"

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif


typedef bool boolean;
typedef unsigned int uint_t;
typedef unsigned char byte;

#define FALSE 0
#define TRUE 1

#define	LOW		0x0
#define	HIGH	0x1
#define	RISING	0x02
#define	FALLING	0x04
#define	CHANGE	0x08

#define	INPUT	0x00
#define	OUTPUT	0x01
#define	INPUT_PULLUP	0x02
#define	OUTPUT_FAST		0x03
#define INPUT_FAST		0x04

#define MS_PER_SEC	1000
#define US_PER_SEC	10000000

#define	DEFAULT	0

#define PWM_FREQUENCY 970

/*
#define true 0x1
#define false 0x0
*/


#define	LED_BUILTIN		13

/* Digital I/O */
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(register uint8_t pin, register uint8_t value);
int digitalRead(uint_t pin);

/* Analog I/O */
void analogReference(uint8_t type);
uint16_t analogRead(uint8_t pin);
void analogWrite(uint32_t pin, uint32_t value);

/* Advanced I/O */
void tone(uint8_t pin, unsigned int frequency, unsigned long duration); /* XXX: Add duration default of 0 */
void noTone(uint8_t pin);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPing, uint8_t bitOrder);
unsigned long pulseIn(uint8_t pin, uint8_t value, unsigned long timeout); /* XXX: Add timeout default of 1000000L */

/* Time */
unsigned long mills(void);
unsigned long micros(void);
void delay(unsigned long ms); 
void delayMicroseconds(unsigned int us);



/* Random Numbers */
void randomSeed(uint32_t seed);
long random(long min, long max); /* XXX: add min default of 0 */

/* Bits and Bytes */
#define lowByte(x)		((uint8_t) ((x) & 0xff))
#define highByte(x)		((uint8_t) ((x) >> 8))
#define bitRead(x,n)	(((x) >> (n)) & 0x01)
#define bitWrite(x,n,b)	(b ? bitSet(x,n) : bitClear(x,n))
#define bitClear(x,n)	((x) &= ~(1UL << (n)))
#define bitSet(x,n)		((x) |= (1UL << (bit)))
#define bit(n)			(1UL << (b))

#define IO0		0
#define IO1		1
#define IO2		2
#define IO3		3
#define IO4		4
#define IO5		5
#define IO6		6
#define IO7		7
#define IO8		8
#define IO9		9
#define IO10	10
#define IO11	11
#define IO12	12
#define IO13	13
#define IO14	14
#define IO15	15
#define IO16	16
#define IO17	17
#define IO18	18
#define IO19	19
#ifdef CONFIG_BOARD_GALILEO

#define A0		14
#define A1		15
#define A2		16
#define A3		17
#define A4		18
#define A5		19

#define SDA     18
#define SCL     19

#endif /* CONFIG_BOARD_GALILEO */

#define SCK     13
#define MISO    12
#define MOSI    11
#define SS	10

#ifdef CONFIG_BOARD_FRDM_K64F

#define A0		24
#define A1		25
#define A2		26
#define A3		27
#define A4		28
#define A5		29

#define SDA     14
#define SCL     15
 
#endif /* CONFIG_BOARD_FRDM_K64F*/

#define SCK     13
#define MISO    12
#define MOSI    11


/* External Interrupts */
void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode);
void detachInterrupt(uint32_t pin);

/* Interrupts */
void interrupts(void);
void noInterrupts(void);

extern void arduino(void);
extern void main(void);

extern void setup(void);
extern void loop(void);

#ifdef __cplusplus
}
#endif

#endif /* _ARDUINO_H_ */




