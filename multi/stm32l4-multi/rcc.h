/*
 * Phoenix-RTOS
 *
 * STM32L4 reset and clock controler driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef RCC_H_
#define RCC_H_


void pwr_lock(void);


void pwr_unlock(void);


int rcc_setHsi(int state);


int rcc_init(void);


#endif
