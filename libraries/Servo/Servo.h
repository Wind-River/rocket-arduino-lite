/*
 * Copyright (c) 2016 Wind River Systems, Inc.
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
 
#ifndef _SERVO_H_
#define _SERVO_H_

#include <zephyr.h>
#include "Arduino-lite.h"

#define MIN_ANGLE               0       /* min angle of rotation */
#define MAX_ANGLE               180     /* max angle of rotation */
#define MIN_PULSE_WIDTH         544     /* min pulse period */
#define MAX_PULSE_WIDTH         2400    /* max pulse period */
#define DEFAULT_PULSE_WIDTH     1500    /* default pulse width when servo is attached */
#define INVALID_SERVO_PIN       -1      /* invalid pin */
#define MAX_NUMBER_OF_SERVOS    6       /* same number of pins in PWM */

/*******************************************************************************
* Servo: servo class, data and methods
*******************************************************************************/
class Servo
{
public:
    Servo();
    void attach(int pin);
    void attach(int pin, int min, int max);
    void detach(void);
    bool attached(void);
    int read(void);
    void write(int angle);
    void writeMicroseconds(int uS);
private:
    int servos_index;
    int servo_pin;
    int min;
    int max;
    int pulse_width;
};

#endif /* _SERVO_H_ */
