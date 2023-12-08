/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _MULTI_GPIO_H_
#define _MULTI_GPIO_H_


#include <sys/msg.h>


void gpio_handleMsg(msg_t *msg, int dev);


int gpio_createDevs(oid_t *oid);


int gpio_init(void);


#endif
