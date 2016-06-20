/* GroveLcd.cpp - Arduino LCD Library for Grove-LCD RGB Backlight module*/

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

#include <stdlib.h>
#include <string.h>
#include "GroveLcd.h"

/* Transmit/Receive data through Arduino Wire API */
Wire lcdwire;

GroveLcd::GroveLcd()
{
}

/*******************************************************************************
* begin() - Initializate the Grove LCD RGB Backlight
*
* Arguments: uint8_t cols - number of LCD lines
*            uint8_t rows - number of LCD rows
*
* Returns:   none
*
* Syntax:   begin(cols, rows)
*/
void GroveLcd::begin(uint8_t cols, uint8_t rows)
{
    uint8_t buf[] = {0, 0};

    PRINT("Initialize LCD ... ");

    if (i2c == NULL)    {
        PRINT("failed! \n");
        return;
    }

    lcdwire.begin();

    clear();

    /* initialization must consist of at least a Function Set command */
    lcdwire.beginTransmission(LCD_ADDR);
    /* 2 line display */
    buf[0] = cols;
    buf[1] = rows;
    lcdwire.write(buf, sizeof(buf));

    lcd_flag = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    buf[0] = 0;
    buf[1] = LCD_INPUTSET | lcd_flag;
    lcdwire.write(buf, sizeof(buf));
    lcdwire.endTransmission();

    PRINT("done\n");
}

void GroveLcd::begin(int cols, int rows)
{
    begin((uint8_t)cols, (uint8_t)rows);
}


/*******************************************************************************
* home() - Set cursor position to home (zero)
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   home()
*/
void GroveLcd::home(void)
{
    lcdCommandWrite(LCD_CURSORHOME);

    /* this command takes a long time, wait time > 1.52 ms */
    task_sleep(45 * sys_clock_ticks_per_sec / 1000);
}


/*******************************************************************************
* clear() - Clear entire display & set cursor position to home
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   clear()
*/
void GroveLcd::clear(void)
{
    lcdCommandWrite(LCD_CLEAR);

    /* this command takes a long time, wait time > 1.52 ms */
    task_sleep(45 * sys_clock_ticks_per_sec / 1000);
}


/*******************************************************************************
* print() - Print a string on display
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   print()
*/
void GroveLcd::print(char *string)
{
    int len = 0;
    
    len = strlen(string);
    lcdDataWrite(string, len);
}


/*******************************************************************************
* setRGB() - Set the color of LCD Backlight
*
* Arguments: uint8_t red - the value of additive primaries color RED
*            uint8_t green - the value of additive primaries color GREEN
*            uint8_t blue - the value of additive primaries color BLUE
*
* Returns:   none
*
* Syntax:   setRGB(red, green, blue)
*/
void GroveLcd::setRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t buf[] = {0, 0};

    lcdwire.beginTransmission(RGB_ADDR);	    /* #define RGB_ADDR 0x0062 */

    buf[0] = 0; buf[1] = 0;
    lcdwire.write(buf, sizeof(buf));
    buf[0] = 1; buf[1] = 0;
    lcdwire.write(buf, sizeof(buf));
    buf[0] = 8; buf[1] = 0xaa;
    lcdwire.write(buf, sizeof(buf));

    buf[0] = 4; buf[1] = red;
    lcdwire.write(buf, sizeof(buf));
    buf[0] = 3; buf[1] = green;
    lcdwire.write(buf, sizeof(buf));
    buf[0] = 2; buf[1] = blue;
    lcdwire.write(buf, sizeof(buf));

    lcdwire.endTransmission();
}

void GroveLcd::setRGB(int red, int green, int blue)
{
    setRGB((uint8_t)red, (uint8_t)green, (uint8_t)blue);
}


/*******************************************************************************
* noDisplay() - Turn the display off (quickly)
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   noDisplay()
*/
void GroveLcd::noDisplay(void)
{
    lcd_flag &= ~LCD_DISPLAYON;
    lcdCommandWrite(LCD_DISPLAYCONTROL | lcd_flag);
}


/*******************************************************************************
* noDisplay() - Turn the display on (quickly)
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   display()
*/
void GroveLcd::display(void)
{
    lcd_flag |= LCD_DISPLAYON;
    lcdCommandWrite(LCD_DISPLAYCONTROL | lcd_flag);
}


/*******************************************************************************
* noCursor() - Turns the underline cursor off
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   noCursor()
*/
void GroveLcd::noCursor(void)
{
    lcd_flag |= LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcdCommandWrite(LCD_DISPLAYCONTROL | lcd_flag);
}


/*******************************************************************************
* cursor() - Turns the underline cursor on
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   cursor()
*/
void GroveLcd::cursor(void)
{
    lcd_flag |= LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF;
    lcdCommandWrite(LCD_DISPLAYCONTROL | lcd_flag);
}


/*******************************************************************************
* noBlink() - Turn off the blinking cursor
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   noBlink()
*/
void GroveLcd::noBlink(void)
{
    lcd_flag |= LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF;
    lcdCommandWrite(LCD_DISPLAYCONTROL | lcd_flag);
}


/*******************************************************************************
* blink() - Turn on the blinking cursor
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   blink()
*/
void GroveLcd::blink(void)
{
    lcd_flag |= LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON;
    lcdCommandWrite(LCD_DISPLAYCONTROL | lcd_flag);
}


/*******************************************************************************
* scrollDisplayLeft() - Scroll the display text from left
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   scrollDisplayLeft()
*/
void GroveLcd::scrollDisplayLeft(void)
{
    lcdCommandWrite(LCD_SHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}


/*******************************************************************************
* scrollDisplayRight() - Scroll the display text from right
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   scrollDisplayRight()
*/
void GroveLcd::scrollDisplayRight(void)
{
    lcdCommandWrite(LCD_SHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}


/*******************************************************************************
* leftToRight() - Set text that flows Left to Right
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   leftToRight()
*/
void GroveLcd::leftToRight(void)
{
    lcd_flag |= LCD_ENTRYLEFT;
    lcdCommandWrite(LCD_INPUTSET | lcd_flag);
}


/*******************************************************************************
* rightToLeft() - Set text that flows Right to Left
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   rightToLeft()
*/
void GroveLcd::rightToLeft(void)
{
    lcd_flag &= ~LCD_ENTRYLEFT;
    lcdCommandWrite(LCD_INPUTSET | lcd_flag);
}


/*******************************************************************************
* autoscroll() - Turn on scrolling text from the cursor
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   autoscroll()
*/
void GroveLcd::autoscroll(void)
{
    lcd_flag |= LCD_ENTRYSHIFTINCREMENT;
    lcdCommandWrite(LCD_INPUTSET | lcd_flag);
}


/*******************************************************************************
* noAutoscroll() - Turn off scrolling text from the cursor
*
* Arguments: none
*
* Returns:   none
*
* Syntax:   noAutoscroll()
*/
void GroveLcd::noAutoscroll(void)
{
    lcd_flag &= ~LCD_ENTRYSHIFTINCREMENT;
    lcdCommandWrite(LCD_INPUTSET | lcd_flag);
}


/*******************************************************************************
* lcdCommandWrite() - Send commnad to control LCD
*
* Arguments: uint8_t cmds - code of the opertional instructions
*
* Returns:   none
*
* Syntax:   lcdCommandWrite()
*/
void GroveLcd::lcdCommandWrite(uint8_t cmds)
{
    uint8_t buf[] = {0, cmds};

    lcdwire.beginTransmission(LCD_ADDR);	        /* #define LCD_ADDR 0x003e */
    lcdwire.write(buf, sizeof(buf));
    lcdwire.endTransmission();    
}


/*******************************************************************************
* lcdDataWrite() - Write a char to LCD display
*
* Arguments: uint8_t value - value of display char
*
* Returns:   none
*
* Syntax:   lcdDataWrite()
*/
void GroveLcd::lcdDataWrite(uint8_t value)
{
    uint8_t buf[] = {LCD_SETCGRAMADDR, value};

    lcdwire.beginTransmission(LCD_ADDR);	        /* #define LCD_ADDR 0x003e */
    lcdwire.write(buf, sizeof(buf));
    lcdwire.endTransmission();    
}


/*******************************************************************************
* lcdDataWrite() - Write a string to LCD display
*
* Arguments: char *string - value of display char
*            int len - length of data
*
* Returns:   none
*
* Syntax:   lcdDataWrite()
*/
void GroveLcd::lcdDataWrite(char *string, int len)
{
    int i;
    uint8_t buf[] = {LCD_SETCGRAMADDR, 0};

    lcdwire.beginTransmission(LCD_ADDR);            /* #define LCD_ADDR 0x003e */
    for (i = 0; i < len; i++) {
        buf[1] = string[i];
        lcdwire.write(buf, sizeof(buf));
    }
    lcdwire.endTransmission();
}
