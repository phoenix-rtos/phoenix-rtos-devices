/*
 * Phoenix-RTOS
 *
 * STM32L1 external interrupts driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#ifndef _EXTI_H_
#define _EXTI_H_

#include <sys/interrupt.h>


int exti_configure(unsigned int line, unsigned char mode, unsigned char edge);


int syscfg_mapexti(unsigned int line, int port);


int exti_init(void);


#endif
