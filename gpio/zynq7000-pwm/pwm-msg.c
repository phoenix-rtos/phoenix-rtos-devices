/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq 7000 PWM driver message API
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdint.h>
#include <sys/msg.h>
#include <errno.h>
#include <unistd.h>

#include "pwm-msg.h"


int pwmmsg_batchWrite(const pwm_ctx_t *ctx, pwm_data_t *vals, int n)
{
	int err;
	msg_t msg;

	if (ctx == NULL || vals == NULL || n == 0) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	msg.i.data = (void *)vals;
	msg.i.size = sizeof(pwm_data_t) * n;
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.i.io.oid = ctx->oid;

	err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	return EOK;
}


int pwmmsg_read(const pwm_ctx_t *ctx, uint32_t *val)
{
	int err;
	msg_t msg;

	if (ctx == NULL || val == NULL) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = (void *)val;
	msg.o.size = sizeof(uint32_t);

	msg.i.io.oid = ctx->oid;

	err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	return EOK;
}


int pwmmsg_write(const pwm_ctx_t *ctx, const uint32_t val)
{
	int err;
	msg_t msg;
	pwm_data_t data;

	if (ctx == NULL) {
		return -EINVAL;
	}

	data.id = ctx->oid.id;
	data.val = val;

	msg.type = mtDevCtl;
	msg.i.data = (void *)&data;
	msg.i.size = sizeof(pwm_data_t);
	msg.o.data = NULL;
	msg.o.size = 0;

	msg.i.io.oid = ctx->oid;

	err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	return EOK;
}


int pwmmsg_close(pwm_ctx_t *ctx)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	return EOK;
}


int pwmmsg_open(pwm_ctx_t *ctx, const char *path)
{
	unsigned int ntries = 10;
	oid_t oid;

	if (ctx == NULL || path == NULL) {
		return -EINVAL;
	}

	while (lookup(path, NULL, &oid) < 0) {
		ntries--;
		if (ntries == 0) {
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	ctx->oid = oid;

	return EOK;
}
