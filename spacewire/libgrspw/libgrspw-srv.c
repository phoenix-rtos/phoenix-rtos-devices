/*
 * Phoenix-RTOS
 *
 * RMAP wrapper for GRSPW messages
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <errno.h>
#include <stdio.h>
#include <posix/utils.h>

#include <libgrspw-srv.h>


static struct {
	spw_dev_t dev[SPW_CNT];
} common;


static struct {
	unsigned int active;
} spw_info[] = {
	{ .active = SPW0_ACTIVE },
	{ .active = SPW1_ACTIVE },
	{ .active = SPW2_ACTIVE },
	{ .active = SPW3_ACTIVE },
	{ .active = SPW4_ACTIVE },
	{ .active = SPW5_ACTIVE },
};

_Static_assert(SPW_CNT <= sizeof(spw_info) / sizeof(spw_info[0]), "SPW_CNT exceeds spw_info array size");

/* Message handling */


static void handleDevCtl(msg_t *msg, int dev)
{
	if ((dev < 0) || (dev >= (sizeof(common.dev) / sizeof(common.dev[0])))) {
		msg->o.err = -ENODEV;
		return;
	}

	const spw_i_t *ictl = (spw_i_t *)msg->i.raw;
	spw_o_t *octl = (spw_o_t *)msg->o.raw;
	spw_dev_t *spw = &common.dev[dev];

	switch (ictl->type) {
		case spw_config:
			msg->o.err = spw_configure(spw, &ictl->task.config);
			break;

		case spw_rxConfig:
			msg->o.err = spw_rxConfigure(spw, &octl->val, ictl->task.rxConfig.nPackets);
			break;

		case spw_rx:
			msg->o.err = spw_rxRead(spw, msg->o.data, msg->o.size, &octl->val, &ictl->task.rx);
			break;

		case spw_tx:
			msg->o.err = spw_transmit(spw, msg->i.data, msg->i.size, &ictl->task.tx);
			break;

		case spw_xfer:
			msg->o.err = spw_xferOp(spw, msg->i.data, msg->i.size, msg->o.data, msg->o.size, &ictl->task.xfer, &octl->val);
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


void spwsrv_handleMsg(msg_t *msg, int dev)
{
	dev -= id_spw0;
	switch (msg->type) {
		case mtDevCtl:
			handleDevCtl(msg, dev);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


/* Initialization */


int spwsrv_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < SPW_CNT; i++) {
		if (spw_info[i].active == 0) {
			continue;
		}

		char buf[8];
		if (snprintf(buf, sizeof(buf), "spw%d", i) >= sizeof(buf)) {
			return -1;
		}

		oid->id = id_spw0 + i;
		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


int spwsrv_init(void)
{
	for (unsigned int i = 0; i < SPW_CNT; i++) {
		if (spw_info[i].active == 0) {
			continue;
		}
		if (spw_initDev(i, &common.dev[i]) < 0) {
			return -1;
		}
	}
	return 0;
}
