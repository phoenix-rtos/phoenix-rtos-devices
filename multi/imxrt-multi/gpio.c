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


#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/threads.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "common.h"
#include "gpio.h"



enum { gpio_dr = 0, gpio_gdir, gpio_psr, gpio_icr1, gpio_icr2, gpio_imr,
	gpio_isr, gpio_edge_sel, gpio_dr_set, gpio_dr_clear, gpio_dr_toggle };


struct {
	volatile uint32_t *base[GPIO_PORTS];
	handle_t lock;
} gpio_common;


static void gpio_handleDevCtl(msg_t *msg, int port)
{
	unsigned int set, clr, t;

	multi_i_t *imsg = (multi_i_t *)msg->i.raw;
	multi_o_t *omsg = (multi_o_t *)msg->o.raw;

	omsg->err = EOK;

	switch (imsg->gpio.type) {
		case gpio_set_port :
			/* DR_SET & DR_CLEAR registers are not functional */
			set = imsg->gpio.port.val & imsg->gpio.port.mask;
			clr = ~imsg->gpio.port.val & imsg->gpio.port.mask;

			mutexLock(gpio_common.lock);
			t = *(gpio_common.base[port] + gpio_dr) & ~clr;
			*(gpio_common.base[port] + gpio_dr) = t | set;
			mutexUnlock(gpio_common.lock);
			break;

		case gpio_get_port :
			omsg->val = *(gpio_common.base[port] + gpio_dr);
			break;

		case gpio_set_dir :
			set = imsg->gpio.dir.val & imsg->gpio.dir.mask;
			clr = ~imsg->gpio.dir.val & imsg->gpio.dir.mask;

			mutexLock(gpio_common.lock);
			t = *(gpio_common.base[port] + gpio_gdir) & ~clr;
			*(gpio_common.base[port] + gpio_gdir) = t | set;
			mutexUnlock(gpio_common.lock);
			break;

		case gpio_get_dir :
			omsg->val = *(gpio_common.base[port] + gpio_gdir);
			break;

		default:
			omsg->err = -ENOSYS;
			break;
	}
}


int gpio_handleMsg(msg_t *msg, int dev)
{
	dev -= id_gpio1;

	if (dev >= GPIO_PORTS)
		return -EINVAL;

	switch (msg->type) {
		case mtGetAttr:
		case mtSetAttr:
		case mtOpen:
		case mtClose:
		case mtWrite:
		case mtRead:
			msg->o.io.err = EOK;
			break;

		case mtDevCtl:
			gpio_handleDevCtl(msg, dev);
			break;

		default:
			msg->o.io.err = -ENOSYS;
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
