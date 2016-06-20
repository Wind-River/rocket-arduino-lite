/* Servo.cpp - Servo Library to control RC (hobby) servo motors */

/*
 * Copyright (c) 2016, Wind River Systems, Inc.
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
#include "Servo.h"

static int totalCounts = 0;         /* number of activated servos */

Servo::Servo()
{
    if ( totalCounts < MAX_NUMBER_OF_SERVOS) {
        this->servos_index = totalCounts;
        this->min = MIN_PULSE_WIDTH;
        this->max = MAX_PULSE_WIDTH;
        this->pulse_width = DEFAULT_PULSE_WIDTH;
    } else {
        this->servos_index = INVALID_SERVO_PIN;
    }
    this->servo_pin = INVALID_SERVO_PIN;
}

/*******************************************************************************
* attach()  - Attach the Servo variable to a pin
*
* Arguments: pin: the number of the pin that the servo is attached to
*            min (optional): the pulse width, in microseconds, corresponding to
*                 the minimum (0-degree) angle on the servo (defaults to 544)
*            max (optional): the pulse width, in microseconds, corresponding to
*                 the maximum (180-degree) angle on the servo (defaults to 2400)
*
* Returns:   None
*
* Syntax:   servo.attach(pin)
*           servo.attach(pin, min, max)
*/
void Servo::attach(int pin, int min, int max)
{
    if (this->servos_index == INVALID_SERVO_PIN) {
        return;
    }

    this->servo_pin = pin;
    if (min < MIN_PULSE_WIDTH) {
        this->min = min = MIN_PULSE_WIDTH;
    }
    this->max = constrain(max, this->min, MAX_PULSE_WIDTH);
    totalCounts++;

    pinMode((uint8_t)this->servo_pin, (uint8_t)OUTPUT);
}

void Servo::attach(int pin)
{
    if (this->servos_index == INVALID_SERVO_PIN) {
        return;
    }

    this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

/*******************************************************************************
* detach()  - Detach the Servo variable from its pin.
*
* Arguments: None
* Returns:   None
*
* Syntax:   servo.detach()
*/
void Servo::detach(void)
{
    if (this->servos_index == INVALID_SERVO_PIN) {
        return;
    }
    pinMode((uint8_t)this->servo_pin, (uint8_t)INPUT);
    this->servo_pin = INVALID_SERVO_PIN;
    this->min = MIN_PULSE_WIDTH;
	this->max = MAX_PULSE_WIDTH;
	this->pulse_width = DEFAULT_PULSE_WIDTH;
	totalCounts--;
}

/*******************************************************************************
* attached()  - Check whether the Servo variable is attached to a pin
*
* Arguments: None
* Returns:   bol  - true if the servo is attached to pin; false otherwise.
*
* Syntax:   servo.attached()
*/
bool Servo::attached(void)
{
    if (this->servo_pin == INVALID_SERVO_PIN) {
        return false;
    } else if ((this->servo_pin) != 3 && (this->servo_pin) != 5 &&
                (this->servo_pin) != 6 && (this->servo_pin) != 9 &&
                (this->servo_pin) != 10 && (this->servo_pin) != 11 ) {
        return false;
    } else {
        return true;
    }
}

/*******************************************************************************
* read()  - Read the current angle of the servo
*
* Arguments: None
*
* Returns:   The angle of the servo, from 0 to 180 degrees
*
* Syntax:   servo.read()
*/
int Servo::read(void)
{
    return map(this->pulse_width, this->min, this->max, MIN_ANGLE, MAX_ANGLE);
}


/*******************************************************************************
* write()  - Writes a value to the servo, controlling the shaft accordingly
*
* Arguments: angle: the value to write to the servo, from 0 to 180
*
* Returns:   none
*
* Syntax:   servo.write(angle)
*/
void Servo::write(int angle)
{
    int rotate;

    /* according to Arduino reference lib, if this angle will be bigger than 200,
    it should be considered as microsenconds */
    if (angle < 200) {
        /* limits the range of rotation values to between 0 and 180 degrees */
        rotate = constrain(angle, MIN_ANGLE, MAX_ANGLE);
        this->pulse_width = 
            map(rotate, MIN_ANGLE, MAX_ANGLE, this->min, this->max);

        /* Measured with servo in Grove Starter Kit V2.0 servo often respond to
        values between 726 and 2600 */
        this->pulse_width =
            map(this->pulse_width, this->min, this->max, 726, 2600);

        digitalWrite(this->servo_pin, HIGH);
        delayMicroseconds(this->pulse_width);
        digitalWrite(this->servo_pin, LOW);
    } else {
        /* user is passing microseconds */
        writeMicroseconds(angle);
	}

}

/*******************************************************************************
* writeMicroseconds()  - Writes a value in microseconds (uS) to the servo,
*                        controlling the shaft accordingly
*
* Arguments: uS: the value of the parameter in microseconds (int)
*
* Returns:   none
*
* Syntax:   servo.writeMicroseconds(uS)
*/
void Servo::writeMicroseconds(int uS)
{
    int rotate;

    /* checking and limiting the boundaries */
    rotate = constrain(uS, this->min, this->max);

    /* Measured with servo in Grove Starter Kit V2.0 servo often respond to
    values between 726 and 2600 */
    this->pulse_width =
        map(rotate, this->min, this->max, 726, 2600);

    digitalWrite(this->servo_pin, HIGH);
	delayMicroseconds(this->pulse_width);
	digitalWrite(this->servo_pin, LOW);
}
