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
 
#ifndef _WIRE_H_
#define _WIRE_H_

#include <zephyr.h>
#include "Arduino-lite.h"

#define byte uint8_t
#define TXBUFSZ     32
#define RCVBUFSZ    32

/* device bindings */
extern struct device *gpio07;
extern struct device *gpioLegacy;
extern struct device *gpio_sus;
extern struct device *gpioEXP0;
extern struct device *gpioEXP1;
extern struct device *gpioEXP2;
extern struct device *i2c;
extern struct device *pinmux;

typedef enum {
	I2C_MASTER,
	I2C_SLAVE,
	I2C_UNKNOWN
} I2C_MODE;


/*******************************************************************************
* Wire: Wire class, data and methods
*******************************************************************************/
class Wire
{
public:
    Wire();
    void begin(void);
    void begin(uint8_t);
    void begin(int);
    uint8_t requestFrom(uint8_t, uint8_t);
    int requestFrom(int, int);
    uint8_t requestFrom(uint8_t, uint8_t, bool);
    int requestFrom(int, int, bool);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    int endTransmission(void);
    int endTransmission(bool);
    uint8_t write(byte);
    uint8_t write(const char *);
    uint8_t write(uint8_t *, int);
    uint8_t available(void);
    uint8_t read(void);
	void onReceive(void(*)(int));
	void onRequest(void(*)(void));

private:
    I2C_MODE mode;
    uint8_t slaveAddress;
    int status;
    int errorstate;
    int startTX;
    uint8_t txAddress;
    uint8_t rxBuffer[RCVBUFSZ];
    uint8_t rxShift;
    uint8_t rxRemain;
    uint8_t txBuffer[TXBUFSZ];
    uint8_t txRemain;
};

#endif /* _WIRE_H_ */
