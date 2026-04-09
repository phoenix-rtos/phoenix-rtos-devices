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
#include <stdio.h>
#include <sys/types.h>
#include <posix/utils.h>

#include <libgrgpio.h>
#include <grgpio-msg.h>

#include "gpio.h"
#include "grlib-multi.h"


static struct {
	grgpio_ctx_t gpioCtx[GPIO_PORT_CNT];
} common;


static void gpio_handleDevCtl(grgpio_ctx_t *ctx, msg_t *msg)
{
	gpio_i_t *idevctl = (gpio_i_t *)msg->i.raw;
	gpio_o_t *odevctl = (gpio_o_t *)msg->o.raw;

	switch (idevctl->type) {
		case gpio_setPort:
			grgpio_setPortVal(ctx, idevctl->mask, idevctl->val);
			msg->o.err = 0;
			break;

		case gpio_getPort:
			odevctl->val = grgpio_getPortVal(ctx);
			msg->o.err = 0;
			break;

		case gpio_setDir:
			grgpio_setPortDir(ctx, idevctl->mask, idevctl->val);
			msg->o.err = 0;
			break;

		case gpio_getDir:
			odevctl->val = grgpio_getPortDir(ctx);
			msg->o.err = 0;
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


void gpio_handleMsg(msg_t *msg, int dev)
{
	dev -= id_gpio0;

	if ((dev < 0) || (dev >= GPIO_PORT_CNT)) {
		msg->o.err = -EINVAL;
		return;
	}

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = 0;
			break;

		case mtDevCtl:
			gpio_handleDevCtl(&common.gpioCtx[dev], msg);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


int gpio_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < GPIO_PORT_CNT; i++) {
		oid->id = id_gpio0 + i;
		char buf[8];
		if (snprintf(buf, sizeof(buf), "gpio%u", i) >= (int)sizeof(buf)) {
			return -1;
		}


		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


int gpio_init(void)
{
	for (unsigned int i = 0; i < GPIO_PORT_CNT; i++) {
		if (grgpio_init(&common.gpioCtx[i], i) < 0) {
			return -1;
		}
	}
	return 0;
}
