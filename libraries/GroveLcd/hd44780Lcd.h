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
 * Header file for Hitachi HD44780 LCD controller
 */

#ifndef _HD44780LCD_H_
#define _HD44780LCD_H_

/*
 * Main LCD Commands
 */
#define LCD_CLEAR               0x01    /* Clear Display */
#define LCD_CURSORHOME          0x02    /* Move cursor to HOME position.  Display unchanged */
#define LCD_INPUTSET            0x04    /* Cursor move direction */
#define LCD_DISPLAYCONTROL      0x08    /* Display, Cursor, blink on/off */
#define LCD_SHIFT               0x10    /* Remove cursor and whole display */
#define LCD_FUNCTIONSET         0x20    /* Set DL, Display, Line, Font */
#define LCD_SETCGRAMADDR        0x40    /* Set CGRAM Address */
#define LCD_SETDDRAMADDR        0x80    /* Set DDRAM Address */

/*
 * Display entry mode (OR/AND with LCD_INPUTSET)
 */
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/*
 * Display & Cursor control (OR/AND with LCD_DISPLAYCONROL)
 */
#define LCD_DISPLAYON           0x04	/* Turn Display on */
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02    /* Turn Cursor on */
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01    /* Set Cursor to blink */
#define LCD_BLINKOFF            0x00

/*
 * Display/Cursor shift (OR/AND with LCD_SHIFT)
 */
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

/*
 * Function set (OR/AND LCD_FUNCTIONSET)
 */
#define LCD_8BITMODE            0x10    /* 8-bit mode */
#define LCD_4BITMODE            0x00    /* 4-bit mode */
#define LCD_2LINE               0x08    /* Two line mode */
#define LCD_1LINE               0x00    /* One line mode */
#define LCD_5x10DOTS            0x04	/* 5x10 dot Font Style */
#define LCD_5x8DOTS             0x00    /* 5x8 dot Font Style */

#endif /* _HD44780LCD_H_ */
