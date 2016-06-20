/*
 * Copyright (c) 2015-2016 Wind River Systems, Inc.
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

/*
 * DESCRIPTION
 * Header file for Arduino LCD library
 */

#ifndef _GROVELCD_H_
#define _GROVELCD_H_

#include "Wire.h"
#include "hd44780Lcd.h"

/* Device I2C Address */
#define LCD_ADDR 0x003e
#define RGB_ADDR 0x0062

/* color define */
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

#define REG_RED         0x04
#define REG_GREEN       0x03
#define REG_BLUE        0x02

#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

/*******************************************************************************
* GroceLcd: GroveLcd class, data and methods
*******************************************************************************/
class GroveLcd
{
public:
    GroveLcd();
    void begin(int, int);
    void begin(uint8_t, uint8_t);
    void home(void);
    void clear(void);
    void print(char *);
    void setRGB(int, int, int);
    void setRGB(uint8_t, uint8_t, uint8_t);
    void noDisplay(void);
    void display(void);
    void noCursor(void);
    void cursor(void);
    void noBlink(void);
    void blink(void);
    void scrollDisplayLeft(void);
    void scrollDisplayRight(void);
    void leftToRight(void);
    void rightToLeft(void);
    void noAutoscroll(void);
    void autoscroll(void);
private:
    inline void lcdCommandWrite(uint8_t);
    inline void lcdDataWrite(uint8_t);
    inline void lcdDataWrite(char *, int);
    uint8_t lcd_flag;

};

#endif /* _GROVELCD_H_ */
