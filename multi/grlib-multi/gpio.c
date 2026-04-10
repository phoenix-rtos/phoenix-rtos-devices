/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO driver (multi wrapper)
 *
 * Copyright 2023, 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>
#include <errno.h>

#include <libgrgpio.h>

#include "gpio.h"
#include "grlib-multi.h"


static grgpio_ctx_t gpio_ctx[GPIO_PORT_CNT];


void gpio_handleMsg(msg_t *msg, int dev)
{
	dev -= id_gpio0;

	if ((dev < 0) || (dev >= GPIO_PORT_CNT)) {
		msg->o.err = -EINVAL;
		return;
	}

	grgpio_handleMsg(&gpio_ctx[dev], msg);
}


int gpio_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < GPIO_PORT_CNT; i++) {
		oid->id = id_gpio0 + i;
		if (grgpio_createDev(oid, i) < 0) {
			return -1;
		}
	}
	return 0;
}


int gpio_init(void)
{
	for (unsigned int i = 0; i < GPIO_PORT_CNT; i++) {
		if (grgpio_init(&gpio_ctx[i], i) < 0) {
			return -1;
		}
	}
	return 0;
}
