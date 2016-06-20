/* Wire.cpp - Wire Library for communicating with I2C */

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

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#include <i2c.h>
#include <pinmux.h>
#include "Wire.h"
#include <string.h>     /* memset */

Wire::Wire()
{
    this->mode = I2C_UNKNOWN;
    this->status = DEV_OK;
    this->errorstate = DEV_OK;
    this->txAddress = 0;
    this->startTX = 0;
    this->rxShift = 0;
    this->rxRemain = 0;
    this->txRemain = 0;
    memset(&this->rxBuffer[0], 0, sizeof(this->rxBuffer));
    memset(&this->txBuffer[0], 0, sizeof(this->txBuffer));
}

/*******************************************************************************
* Wire.begin()  - Initiate the Wire library and join the I2C bus as a master
*                 or slave. This should normally be called only once..
*
* Arguments: address  - the 7-bit slave address (optional); if not specified,
*                        join the bus as a master.
*
* Returns:   None
*
* Syntax:   Wire.begin()
*           Wire.begin(address)
*/
void Wire::begin(void)
{
    this->mode = I2C_MASTER;
}

void Wire::begin(uint8_t address)
{
    this->mode = I2C_SLAVE;
    if (this->mode == I2C_SLAVE) {
        /* the 7-bit slave address */
        this->slaveAddress = address << 1;
    }
}

void Wire::begin(int address)
{
    this->begin((uint8_t)address);
}

/*******************************************************************************
* Wire.requestFrom() - Used by the master to request bytes from a slave device
*
* Arguments: address  - the 7-bit address of device to request bytes from
*            quantity - the number of bytes to request
*            stop     - Boolean
*                       True will send a stop message after the request,
*                       releasing the bus.
*                       False will continually send a restart after the request,
*                       keeping the connection active
*
* Returns:  byte - the number of bytes returned from the slave device
*
* Syntax:   Wire.requestFrom(address, quantity)
*           Wire.requestFrom(address, quantity, stop)
*
* Usage: requestFrom() + available() + read()
*/
uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity)
{
    return this->requestFrom(address, quantity, false);
}

uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity, bool stop)
{
    this->rxShift = 0;
    this->rxRemain = min(quantity, RCVBUFSZ);

    if (!i2c) {
        return 0;
    }
    this->status =
        i2c_read(i2c, &this->rxBuffer[0], this->rxRemain, address);
    if (this->status != DEV_OK) {
        return 0;
    } else {
        return this->rxRemain;
    }
}

int Wire::requestFrom(int address, int quantity)
{
    return (int)this->requestFrom((uint8_t)address, (uint8_t)quantity, false);
}

int Wire::requestFrom(int address, int quantity, bool stop)
{
    return (int)this->requestFrom((uint8_t)address, (uint8_t)quantity, stop);
}

/*******************************************************************************
* Wire.beginTransmission() - Begin a transmission to the I2C slave device
*                            with the given address
*
* Arguments: address: the 7-bit address of the device to transmit to
*
* Returns:  none
*
* Syntax:   beginTransmission(address)
*/
void Wire::beginTransmission(uint8_t address)
{
    if (this->startTX) {
        return;
    }
    this->startTX = 1;
    this->txAddress = (uint8_t)address;
}

void Wire::beginTransmission(int address)
{
    this->beginTransmission((uint8_t)address);
}

/*******************************************************************************
* Wire.endTransmission()  - Ends a transmission to a slave device that was begun
*                           by beginTransmission() and transmits the bytes that
*                           were queued by write()
*
* Arguments: stop - Boolean
*                   True will send a stop message, releasing the bus after
*                   transmission.
*                   False will send a restart, keeping the connection active
*
* Returns:  byte, which indicates the status of the transmission:
*           0:  success
*           1:  data too long to fit in transmit buffer
*           2:  received NACK on transmit of address
*           3:  received NACK on transmit of data
*           4:  other error
*
* Syntax:   endTransmission()
*           endTransmission(stop)
*/
int Wire::endTransmission(bool stop)
{
    this->startTX = 0;
    this->txRemain = 0;
    memset(&this->txBuffer[0], 0, sizeof(this->txBuffer));

    return 0;
}

int Wire::endTransmission(void)
{
    return this->endTransmission(false);
}

/*******************************************************************************
* Wire.write()  - Writes data from a slave device in response to a request from
*                 a master, or
*                 queues bytes for transmission from a master to slave device
*                 (in-between calls to beginTransmission() & endTransmission())
*
* Arguments:  value: a value to send as a single byte
*             string: a string to send as a series of bytes
*             data: an array of data to send as bytes
*             length: the number of bytes to transmit
*
* Returns:  byte: will return the number of bytes written, though reading that
*                 number is optional
*
* Syntax:   Wire.write(value)
*           Wire.write(string)
*           Wire.write(data, length)
*/
uint8_t Wire::write(uint8_t value)
{
    uint8_t str_val[] = {0, 0};

    if (!i2c) {
        return 0;
    }

    str_val[0] = value;
    this->status = i2c_write(i2c, str_val, 1, (uint16_t)this->txAddress);
    if (this->status != DEV_OK) {
        return 0;
    } else {
        return 1;
    }
}

uint8_t Wire::write(const char *string)
{
    int int_val;

    if (!i2c) {
        return 0;
    }
    int_val = strlen(string);

    return this->write((uint8_t *)string, int_val);
}

uint8_t Wire::write(uint8_t *data, int length)
{
    int i, int_val;
    uint8_t str_val[TXBUFSZ];
    uint16_t target = 0;

    if (!i2c) {
        return 0;
    }

    int_val = min(length, TXBUFSZ);
    /* formatted 1st data array by the corresponding data len */
    for (i = 0; i < int_val; i++)  {
        str_val[i] = data[i];
    }
    
    target = (uint16_t)this->txAddress;
    this->status = i2c_write(i2c, str_val, (uint32_t)int_val, target);
    if (this->status != DEV_OK) {
        return 0;
    } else {
        return (uint8_t)int_val;
    }
}


/*******************************************************************************
* Wire.available() - Returns the number of bytes available for retrieval with
*                    read()
*
* Arguments: none
*
* Returns:  The number of bytes available for reading.
*
* Syntax:   Wire.available()
*/
uint8_t Wire::available(void)
{
    return (this->rxRemain - this->rxShift);
}


/*******************************************************************************
* Wire.read() - Reads a byte that was transmitted from a slave device to a
*               master after a call to requestFrom() or was transmitted from a
*               master to a slave. read() inherits from the Stream utility class
*
* Arguments: none
*
* Returns:  The next byte received
*
* Syntax:   Wire.read()
*/
uint8_t Wire::read(void)
{
    uint8_t requested = this->rxBuffer[this->rxShift];

    this->rxShift++;
    return requested;
}

void (*onRequestCallback)(void);
void (*onReceiveCallback)(int);

/*******************************************************************************
* Wire.onReceive() - Registers a function to be called when a slave device
*                    receives a transmission from a master
*
* Arguments: handler: the function to be called when the slave receives data
*
* Returns:  none
*
* Syntax:   Wire.onReceive(handler)
*/
void onReceive(void(*pHandler)(int))
{
    onReceiveCallback = pHandler;
}

/*******************************************************************************
* Wire.onRequest() - Register a function to be called when a master requests
*                    data from this slave device
*
* Arguments: handler: the function to be called, takes no parameters and
*                     returns nothing
*
* Returns:  none
*
* Syntax:   Wire.onRequest(handler)
*/
void onRequest(void(*pHandler)(void))
{
    onRequestCallback = pHandler;
}