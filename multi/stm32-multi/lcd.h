/*
 * Phoenix-RTOS
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


void lcd_update(void);


void lcd_showString(const char *text);


void lcd_showSymbols(unsigned int sym_mask, unsigned int state);


void lcd_showSmallString(const char *text);


void lcd_enable(int on);


int lcd_setBacklight(unsigned char val);


int lcd_init(void);


#endif
