/*
 * Phoenix-RTOS
 *
 * STM32L4 external interrupts driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#ifndef EXTI_H_
#define EXTI_H_

#include <sys/interrupt.h>


int exti_configure(unsigned int line, unsigned char mode, unsigned char edge);


int syscfg_mapexti(unsigned int line, int port);


int exti_clear_irq(unsigned int line);


int exti_init(void);


#endif
