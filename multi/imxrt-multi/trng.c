/*
 * Phoenix-RTOS
 *
 * i.MX RT TRNG driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "trng.h"
#include "common.h"


enum { mctl = 0, scmisc, pkrrng, pkrmax, sdctl, sblim, frqmin, frqcnt, scmc, scr1c,
	scr2c, scr3c, scr4c, scr5c, scr6c, status, ent, pkrcnt10 = 32, pkrcnt32,
	pkrcnt54, pkrcnt76, pkrcnt98, pkrcntba, pkrcntdc, pkrcntfe, sec_cfg, int_ctrl,
	int_mask, int_status, reserved_0, vid1 = 60, vid2 };


#define TRNG_MCTL_RST  0x40
#define TRNG_MCTL_VAL  0x400
#define TRNG_MCTL_ERR  0x1000
#define TRNG_MCTL_PRGM 0x10000

#define TRNG_ENT_COUNT 16

struct {
	volatile uint32_t *base;
} trng_common;


static uint32_t trng_readEnt(int index)
{
	uint32_t data;

	if (index > TRNG_ENT_COUNT)
		return 0;

	data = *(trng_common.base + ent + index);

	/* Dummy read, error workaround */
	if (index == TRNG_ENT_COUNT - 1)
		(void)*(trng_common.base + ent);

	return data;
}


static int trng_read(char *data, size_t size)
{
	uint32_t val;
	uint32_t mctlVal;
	size_t offs = 0, chunk;
	int index = 0;

	while (offs < size) {
		/* Wait for valid entropy or error */
		do {
			mctlVal = *(trng_common.base + mctl);
		} while ((mctlVal & (TRNG_MCTL_VAL | TRNG_MCTL_ERR)) == 0);

		if (mctlVal & TRNG_MCTL_ERR)
			return offs;

		val = trng_readEnt(index);
		chunk = (size - offs < sizeof(val)) ? size - offs : sizeof(val);
		memcpy(data + offs, &val, chunk);

		offs += chunk;
		index = (index + 1) % TRNG_ENT_COUNT;
	}

	/* Start new entropy generation */
	(void)trng_readEnt(TRNG_ENT_COUNT - 1);

	return offs;
}


int trng_handleMsg(msg_t *msg)
{
	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.io.err = EOK;
			break;
		case mtRead:
			msg->o.io.err = trng_read(msg->o.data, msg->o.size);
			break;
		case mtGetAttr:
			msg->o.attr.err = -EINVAL;
			break;
		default:
			msg->o.io.err = -EINVAL;
	}

	return EOK;
}


int trng_init(void)
{
	trng_common.base = TRNG_BASE;

	if (common_setClock(pctl_clk_trng, clk_state_run) < 0)
		return -EFAULT;

	/* Set to program mode */
	*(trng_common.base + mctl) |= TRNG_MCTL_PRGM;

	/* Reset to default state */
	*(trng_common.base + mctl) |= TRNG_MCTL_RST;

	/* Set to run mode */
	*(trng_common.base + mctl) &= ~TRNG_MCTL_PRGM;
	/* TODO: access bit? */

	/* Start generation */
	(void)trng_readEnt(TRNG_ENT_COUNT - 1);

	return EOK;
}
