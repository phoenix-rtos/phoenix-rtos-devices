/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq7000 / ZynqMP PWM driver binary interface
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
#include "zynq-pwm-priv.h"
#include "zynq-pwm-msg.h"


int zynqpwm_set(oid_t *oid, uint32_t compval[ZYNQ_PWM_CHANNELS], uint8_t mask)
{
	int ret;
	msg_t msg;
	zynqpwm_imsg_t *i = (zynqpwm_imsg_t *)msg.i.raw;
	zynqpwm_omsg_t *o = (zynqpwm_omsg_t *)msg.o.raw;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.type = mtDevCtl;
	i->type = zynqpwm_msgSet;
	i->mask = mask;
	msg.oid = *oid;
	memcpy(i->compval, compval, sizeof(i->compval));

	ret = msgSend(oid->port, &msg);
	if (ret >= 0) {
		ret = o->err;
	}

	return ret;
}


int zynqpwm_get(oid_t *oid, uint32_t compval[ZYNQ_PWM_CHANNELS])
{
	int ret;
	msg_t msg;
	zynqpwm_imsg_t *i = (zynqpwm_imsg_t *)msg.i.raw;
	zynqpwm_omsg_t *o = (zynqpwm_omsg_t *)msg.o.raw;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.type = mtDevCtl;
	i->type = zynqpwm_msgGet;
	msg.oid = *oid;
	ret = msgSend(oid->port, &msg);
	if (ret >= 0) {
		ret = o->err;
	}

	if (ret >= 0) {
		memcpy(compval, o->compval, sizeof(o->compval));
	}

	return ret;
}
