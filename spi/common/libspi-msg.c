/*
 * Phoenix-RTOS
 *
 * SPI message interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <spi-msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


int spimsg_xfer(const spimsg_ctx_t *ctx, const void *out, size_t olen, void *in, size_t ilen, size_t iskip)
{
	msg_t msg;
	spi_devctl_t *idevctl = (spi_devctl_t *)msg.i.raw;
	spi_devctl_t *odevctl = (spi_devctl_t *)msg.o.raw;
	int err;

	if (ctx == NULL) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	msg.i.data = (void *)out;
	msg.i.size = olen;
	msg.o.data = in;
	msg.o.size = ilen;
	idevctl->i.type = spi_devctl_xfer;
	idevctl->i.ctx = *ctx;
	idevctl->i.iskip = iskip;

	err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	return odevctl->o.err;
}


int spimsg_close(const spimsg_ctx_t *ctx)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	return EOK;
}


int spimsg_open(unsigned int dev, unsigned int ss, spimsg_ctx_t *ctx)
{
	unsigned int ntries = 10;
	char devName[16];
	int err;

	if (ctx == NULL) {
		return -EINVAL;
	}

	err = snprintf(devName, sizeof(devName), "/dev/spi%u.%u", dev, ss);
	if (err >= sizeof(devName)) {
		return -EINVAL;
	}

	while (lookup(devName, NULL, &ctx->oid) < 0) {
		ntries--;
		if (ntries == 0) {
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	return EOK;
}
