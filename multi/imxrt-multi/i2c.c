/*
 * Phoenix-RTOS
 *
 * i.MX RT i2c driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <errno.h>

#include "common.h"
#include "gpio.h"

#define I2C1_POS 0
#define I2C2_POS (I2C1_POS + I2C1)
#define I2C3_POS (I2C2_POS + I2C2)
#define I2C4_POS (I2C3_POS + I2C3)

#define I2C_CNT (I2C1 + I2C2 + I2C3 + I2C4)

static const int i2cConfig[] = { I2C1, I2C2, I2C3, I2C4 };


static const int i2cPos[] = { I2C1_POS, I2C2_POS, I2C3_POS, I2C4_POS };


enum { veridr = 0, paramr, mcr = 4, msr, mier, mder, mscfgr0, mscfgr1, mscfgr2, mscfgr3, mdmr = 16,
	mccr0 = 18, mccr1 = 20, mfcr = 22, mfsr, mtdr, mrdr = 28, scr = 68, ssr, sier, sder,
	scfgr1 = 73, scfgr2, samr = 80, sasr = 84, star, stdr= 88, srdr= 92 };


int i2c_handleMsg(msg_t *msg, int dev)
{
	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.io.err = EOK;
			break;
		default:
			msg->o.io.err = -EINVAL;
	}

	return EOK;
}


int i2c_init(void)
{
	int i, dev;

	static const struct {
		volatile uint32_t *base;
		int clk;
		unsigned irq;
	} info[] = {
		{ I2C1_BASE, I2C1_CLK, I2C1_IRQ },
		{ I2C2_BASE, I2C2_CLK, I2C2_IRQ },
		{ I2C3_BASE, I2C3_CLK, I2C3_IRQ },
		{ I2C4_BASE, I2C4_CLK, I2C4_IRQ }
	};

	/* FIXME: remove following statements when driver implementation will be finished */
	(void)i;
	(void)i2cPos;

	/* TODO: initialize peripherals */
	for (i = 0, dev = 0; dev < sizeof(i2cConfig) / sizeof(i2cConfig[0]); ++dev) {
		if (!i2cConfig[dev])
			continue;

		if (common_setClock(info[dev].clk, clk_state_run) < 0)
			return -EFAULT;
	}

	return 0;
}
