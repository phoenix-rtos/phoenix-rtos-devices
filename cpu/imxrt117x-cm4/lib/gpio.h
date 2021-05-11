/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 core gpio driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef M4GPIO_H_
#define M4GPIO_H_

#include <stdint.h>

// clang-format off
enum { gpio1 = 0, gpio2, gpio3, gpio4, gpio5, gpio6, gpio7, gpio8, gpio9, gpio10, gpio11, gpio12, gpio13};


enum { gpio_in = 0, gpio_out };
// clang-format on


int gpio_setPin(int port, int pin, int state);


int gpio_togglePin(int port, int pin);


int gpio_setPort(int port, uint32_t mask, uint32_t val);


int gpio_getPort(int port, unsigned int *val);


int gpio_setDir(int port, int pin, int dir);


void gpio_init(void);

#endif
