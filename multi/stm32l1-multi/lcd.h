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

#include "stm32l1-multi.h"


void lcd_getDisplay(lcdmsg_t *disp);


void lcd_setDisplay(lcdmsg_t *disp);


int lcd_init(void);


#endif
