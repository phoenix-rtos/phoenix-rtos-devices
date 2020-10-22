/*
 * Phoenix-RTOS
 *
 * STM32L4 GPIO driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef GPIO_H_
#define GPIO_H_


int gpio_setPort(int port, unsigned int mask, unsigned int val);


int gpio_getPort(int port, unsigned int *val);


int gpio_configPin(int port, char pin, char mode, char af, char otype, char ospeed, char pupd);


int gpio_init(void);


#endif
