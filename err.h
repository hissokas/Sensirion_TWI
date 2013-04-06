/*
    File:       err.h
    Version:    0.1.0
    Date:       Feb. 25, 2006
    
    C header file for HD44780 LCD module using a 74HCT164 serial in, parallel 
    out out shift register to operate the LCD in 8-bit mode.  Example schematic
    usuing a Powertip PC-1202A LCD available at http://www.micahcarrick.com
    
    Uses the avr-libc library for gcc (avr-gcc).
    
    ****************************************************************************
    Copyright (C) 2006 Micah Carrick   <email@micahcarrick.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
    ****************************************************************************
*/
#ifndef _ERR_H
#define _ERR_H

#include <inttypes.h>

/* AVR port and pins connected to '164 and/or LCD */
#define ERR_PORT                PORTA

#define ERR_RSDS_PIN            5
#define ERR_ENABLE_PIN          6
#define ERR_CLOCK_PIN           7



#define ERR_BACKLIGHT_PIN       7

/* number of columns on the display */
#define ERR_COLS 20

/* LCD commands */
#define ERR_CMD_HOME            0x02    /* cursor home */
#define ERR_CMD_ENTRY_DEC       0x04    /* decrement, no shift */
#define ERR_CMD_ENTRY_DEC_SHIFT 0x05    /* decrement with shift */
#define ERR_CMD_ENTRY_INC       0x06    /* increment, no shift */
#define ERR_CMD_ENTRY_INC_SHIFT 0x07    /* increment with shift */
#define ERR_CMD_CLEAR           0x01    /* clear, cursor home */
#define ERR_CMD_OFF             0x08    /* display off */
#define ERR_CMD_ON              0x0C    /* display on, cursor off */
#define ERR_CMD_ON_BLINK        0x0D    /* display on, cursor off, blink char */
#define ERR_CMD_ON_CURSOR       0x0E    /* display on, cursor on */
#define ERR_CMD_ON_CURSOR_BLINK 0x0F    /* display on, cursor on, blink char */
#define ERR_CMD_CURSOR_LEFT     0x10    /* decrement cursor (left) */
#define ERR_CMD_CURSOR_RIGHT    0x14    /* increment cursor (right) */
#define ERR_CMD_SHIFT_LEFT      0x18    /* shift (left) */
#define ERR_CMD_SHIFT_RIGHT     0x1C    /* shift (right)*/

#define ERR_LINE_1              0x80 /* bit 7 is always set: 0x80 = 0x00 */ 
#define ERR_LINE_2              0xC0 /* 2nd line at position 40 */ 

/* function set: (always 8-bit with the shift-register circuit I'm using */
#define ERR_FUNCTION_8x1        0x30   /* 5x7 characters, single line display */
#define ERR_FUNCTION_8x2        0x38   /* 5x7 characters, 2 line display */

//Fleury
#define ERR_LINES           2     /**< number of visible lines of the display */
#define ERR_DISP_LENGTH    20     /**< visibles characters per line of the display */
#define ERR_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define ERR_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */
#define ERR_START_LINE3  0x14     /**< DDRAM address of first char of line 3 */
#define ERR_START_LINE4  0x54     /**< DDRAM address of first char of line 4 */
#define ERR_CGRAM             6      /* DB6: set CG RAM address             */
#define ERR_DDRAM             7      /* DB7: set DD RAM address             */



/* F_CPU is used by delay routines from util/delay.h. May already be defined */
#ifndef F_CPU
#define F_CPU 1000000  /* 1 MHz CPU clock */
#endif

/* functions */
void err_backlight(int);
void err_initialize(uint8_t, uint8_t, uint8_t);
void err_load_byte(uint8_t);
void err_send_cmd(void);
void err_send_char(void);
void err_strobe_E(void);
void err_cls(void);
void err_clr_line(uint8_t Linie);
void err_clr_part(uint8_t Linie, uint8_t von, uint8_t bis);
void err_cursor_home(void);
void err_move_cursor(uint8_t, uint8_t);
void err_inc_cursor(uint8_t);
void err_dec_cursor(uint8_t);
void err_gotoxy(uint8_t x, uint8_t y);
void err_putc(const char c);
void err_puts(const char *s);
void err_putint(uint8_t zahl);
void err_putint1(uint8_t zahl);	//einstellige Zahl
void err_putint2(uint8_t zahl);	//zweistellige Zahl
void err_puthex(uint8_t zahl);

#endif
