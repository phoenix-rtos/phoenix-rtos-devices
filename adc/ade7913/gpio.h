/*
 * Phoenix-RTOS
 *
 * Helper gpio functions
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <sys/platform.h>

#ifndef GPIO_H
#define GPIO_H

void gpio_setPin(oid_t *device, int gpio, int pin, int state);
void gpio_setDir(oid_t *device, int gpio, int pin, int dir);

#endif  // GPIO_H
