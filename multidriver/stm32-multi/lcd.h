/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 LCD driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Adrian Kepka, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LCD_H_
#define _LCD_H_

#define LCDDRV_CHAR_DEGREE 0xf8


typedef enum _lcd_symbols_t {
	LCDSYM_SMALL_ONE	= 1 << 0,
	LCDSYM_CLOCK		= 1 << 1,
	LCDSYM_DATE			= 1 << 2,
	LCDSYM_WARN			= 1 << 3,
	LCDSYM_BATT			= 1 << 4,
	LCDSYM_LOWBATT		= 1 << 5,
	LCDSYM_NOBATT		= 1 << 6,
	LCDSYM_LOCK			= 1 << 7,
	LCDYSM_FACTORY		= 1 << 8,
	LCDSYM_FIRE			= 1 << 9,
	LCDSYM_COM			= 1 << 10,
	LCDSYM_POUND		= 1 << 11,
	LCDSYM_EUR			= 1 << 12,
	LCDSYM_M3			= 1 << 13,
	LCDSYM_SLASH_TOP	= 1 << 14,
	LCDSYM_H_TOP		= 1 << 15,
	LCDSYM_H_BOT		= 1 << 16,
	LCDSYM_K			= 1 << 17,
	LCDSYM_W			= 1 << 18,
	LCDSYM_FRAME 		= 1 << 19,
	LCDSYM_FRAME_SMALL	= 1 << 20,
	LCDSYM_BAR0 		= 1 << 21,
	LCDSYM_BAR1			= 1 << 22,
	LCDSYM_BAR2			= 1 << 23,
	LCDSYM_DOT1_BOT		= 1 << 24,
	LCDSYM_DOT2_TOP		= 1 << 25,
	LCDSYM_DOT2_BOT 	= 1 << 26,
	LCDSYM_DOT3_BOT		= 1 << 27,
	LCDSYM_DOT4_TOP		= 1 << 28,
	LCDSYM_DOT4_BOT		= 1 << 29,
	LCDSYM_TOTAL 		= 30
} lcd_symbols_t;


void lcddrv_update(void);


void lcd_showString(const char *text);


void lcd_showSymbols(unsigned int sym_mask, unsigned int state);


void lcd_showSmallString(const char *text);


void lcd_enable(int on);


int lcd_setBacklight(unsigned char val);


int lcddrv_init(void);


#endif
