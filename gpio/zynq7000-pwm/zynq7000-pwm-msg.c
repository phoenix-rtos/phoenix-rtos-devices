/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq 7000 PWM driver binary interface
 *
 * Copyright 2023 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <string.h>
#include "zynq7000-pwm-priv.h"
#include "zynq7000-pwm-msg.h"


int zynq7000pwm_set(oid_t *oid, uint32_t compval[ZYNQ7000_PWM_CHANNELS], uint8_t mask)
{
	int ret;
	msg_t msg;
	zynq7000pwm_imsg_t *i = (zynq7000pwm_imsg_t *)msg.i.raw;
	zynq7000pwm_omsg_t *o = (zynq7000pwm_omsg_t *)msg.o.raw;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.type = mtDevCtl;
	i->type = zynq7000pwm_msgSet;
	i->mask = mask;
	memcpy(i->compval, compval, sizeof(i->compval));

	ret = msgSend(oid->port, &msg);
	if (ret >= 0) {
		ret = o->err;
	}

	return ret;
}


int zynq7000pwm_get(oid_t *oid, uint32_t compval[ZYNQ7000_PWM_CHANNELS])
{
	int ret;
	msg_t msg;
	zynq7000pwm_imsg_t *i = (zynq7000pwm_imsg_t *)msg.i.raw;
	zynq7000pwm_omsg_t *o = (zynq7000pwm_omsg_t *)msg.o.raw;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.type = mtDevCtl;
	i->type = zynq7000pwm_msgGet;

	ret = msgSend(oid->port, &msg);
	if (ret >= 0) {
		ret = o->err;
	}

	if (ret >= 0) {
		memcpy(compval, o->compval, sizeof(o->compval));
	}

	return ret;
}
