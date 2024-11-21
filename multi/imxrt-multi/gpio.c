/*
 * Phoenix-RTOS
 *
 * i.MX RT GPIO driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdint.h>
#include <errno.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include "common.h"
#include "gpio.h"

/* clang-format off */

enum { gpio_dr = 0, gpio_gdir, gpio_psr, gpio_icr1, gpio_icr2, gpio_imr,
	gpio_isr, gpio_edge_sel, gpio_dr_set, gpio_dr_clear, gpio_dr_toggle };

/* clang-format on */

struct {
	volatile uint32_t *base[GPIO_PORTS];
	handle_t lock;
} gpio_common;


int gpio_setPort(int port, uint32_t mask, uint32_t val)
{
	unsigned int set, clr, t;

	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	set = val & mask;
	clr = ~val & mask;

	/* DR_SET & DR_CLEAR registers are not functional */
	mutexLock(gpio_common.lock);
	t = *(gpio_common.base[port] + gpio_dr) & ~clr;
	*(gpio_common.base[port] + gpio_dr) = t | set;
	mutexUnlock(gpio_common.lock);
	return EOK;
}


int gpio_getPort(int port, uint32_t *val)
{
	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	*val = *(gpio_common.base[port] + gpio_dr);
	return EOK;
}


int gpio_setDir(int port, uint32_t mask, uint32_t val)
{
	unsigned int set, clr, t;

	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	set = val & mask;
	clr = ~val & mask;

	mutexLock(gpio_common.lock);
	t = *(gpio_common.base[port] + gpio_gdir) & ~clr;
	*(gpio_common.base[port] + gpio_gdir) = t | set;
	mutexUnlock(gpio_common.lock);
	return EOK;
}


int gpio_getDir(int port, uint32_t *val)
{
	if ((port <= 0) || (port > GPIO_PORTS)) {
		return -EINVAL;
	}

	port -= 1;
	*val = *(gpio_common.base[port] + gpio_gdir);
	return EOK;
}


static void gpio_handleDevCtl(msg_t *msg, int port)
{
	multi_i_t *imsg = (multi_i_t *)msg->i.raw;
	multi_o_t *omsg = (multi_o_t *)msg->o.raw;

	msg->o.err = EOK;

	switch (imsg->gpio.type) {
		case gpio_set_port:
			msg->o.err = gpio_setPort(port, imsg->gpio.port.mask, imsg->gpio.port.val);
			break;

		case gpio_get_port:
			msg->o.err = gpio_getPort(port, &omsg->val);
			break;

		case gpio_set_dir:
			msg->o.err = gpio_setDir(port, imsg->gpio.dir.mask, imsg->gpio.dir.val);
			break;

		case gpio_get_dir:
			msg->o.err = gpio_getDir(port, &omsg->val);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


int gpio_handleMsg(msg_t *msg, int dev)
{
	dev -= id_gpio1 - 1; /* Port number will be verified later */

	switch (msg->type) {
		case mtGetAttr:
		case mtSetAttr:
		case mtOpen:
		case mtClose:
		case mtWrite:
		case mtRead:
			msg->o.err = EOK;
			break;

		case mtDevCtl:
			gpio_handleDevCtl(msg, dev);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}

	return 0;
}


int gpio_init(void)
{
	int i;
	static const void *addresses[] = { GPIO1_BASE, GPIO2_BASE, GPIO3_BASE,
		GPIO4_BASE, GPIO5_BASE, GPIO6_BASE, GPIO7_BASE, GPIO8_BASE, GPIO9_BASE,
		GPIO10_BASE, GPIO11_BASE, GPIO12_BASE, GPIO13_BASE };

#ifndef __CPU_IMXRT117X
	platformctl_t pctl;
	static const int clocks[] = { GPIO1_CLK, GPIO2_CLK, GPIO3_CLK,
		GPIO4_CLK, GPIO5_CLK, GPIO6_CLK, GPIO7_CLK, GPIO8_CLK, GPIO9_CLK,
		GPIO10_CLK, GPIO11_CLK, GPIO12_CLK, GPIO13_CLK };

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.state = clk_state_run;

	for (i = 0; i < sizeof(clocks) / sizeof(clocks[0]); ++i) {
		if (clocks[i] < 0)
			continue;

		pctl.devclock.dev = clocks[i];
		if (platformctl(&pctl) < 0)
			return -1;
	}
#endif

	for (i = 0; i < sizeof(gpio_common.base) / sizeof(gpio_common.base[0]); ++i)
		gpio_common.base[i] = (void *)addresses[i];

	return 0;
}
