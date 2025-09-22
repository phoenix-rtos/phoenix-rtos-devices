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
#include <rcpwm/api.h>
#include <rcpwm/msg.h>


int rcpwm_set(oid_t *oid, uint32_t compval[RCPWM_CHANNELS], uint8_t mask)
{
	int ret;
	msg_t msg;
	rcpwm_imsg_t *i = (rcpwm_imsg_t *)msg.i.raw;
	rcpwm_omsg_t *o = (rcpwm_omsg_t *)msg.o.raw;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.type = mtDevCtl;
	i->type = rcpwm_msgSet;
	i->mask = mask;
	msg.oid = *oid;
	memcpy(i->compval, compval, sizeof(i->compval));

	ret = msgSend(oid->port, &msg);
	if (ret >= 0) {
		ret = o->err;
	}

	return ret;
}


int rcpwm_get(oid_t *oid, uint32_t compval[RCPWM_CHANNELS])
{
	int ret;
	msg_t msg;
	rcpwm_imsg_t *i = (rcpwm_imsg_t *)msg.i.raw;
	rcpwm_omsg_t *o = (rcpwm_omsg_t *)msg.o.raw;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.type = mtDevCtl;
	i->type = rcpwm_msgGet;
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
