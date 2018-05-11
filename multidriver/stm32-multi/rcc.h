/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 reset and clock controler driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RCC_H_
#define _RCC_H_


inline void pwr_lock(void);


inline void pwr_unlock(void);


int rcc_setHsi(int state);


int rcc_devClk(int dev, int state);


int rcc_init(void);

#endif
