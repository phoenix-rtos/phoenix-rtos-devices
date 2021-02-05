/*
 * Phoenix-RTOS
 *
 * i.MX RT GPIO driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#include <sys/msg.h>

int gpio_handleMsg(msg_t *msg, int dev);


int gpio_init(void);

#endif
