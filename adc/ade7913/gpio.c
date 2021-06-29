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

#include <imxrt-multi.h>

#include "gpio.h"


void gpio_setPin(oid_t *device, int gpio, int pin, int state)
{
	msg_t msg;
	multi_i_t *imsg = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_port;
	imsg->gpio.port.val = !!state << pin;
	imsg->gpio.port.mask = 1 << pin;

	msgSend(device->port, &msg);
}


void gpio_setDir(oid_t *device, int gpio, int pin, int dir)
{
	msg_t msg;
	multi_i_t *imsg = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_dir;
	imsg->gpio.dir.val = !!dir << pin;
	imsg->gpio.dir.mask = 1 << pin;

	msgSend(device->port, &msg);
}