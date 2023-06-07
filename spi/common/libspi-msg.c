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
#include <spi.h>
#include <spi-msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


int spimsg_xfer(const spimsg_ctx_t *ctx, const void *out, size_t olen, void *in, size_t ilen, size_t iskip)
{
	msg_t msg;
	spi_devctl_t *idevctl = (spi_devctl_t *)msg.i.raw;
	spi_devctl_t *odevctl = (spi_devctl_t *)msg.o.raw;
	int err, needscopy = 0;

	if (ctx == NULL) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	idevctl->u.i.type = spi_devctl_xfer;
	idevctl->u.i.ctx = *ctx;

	/* Pack msg to the raw fields */
	if (olen <= (sizeof(msg.i.raw) - sizeof(spi_devctl_t))) {
		memcpy(idevctl->payload, out, olen);
	}
	else {
		msg.i.data = (void *)out; /* FIXME msg.i.data should be const */
		msg.i.size = olen;
	}
	idevctl->u.i.xfer.isize = olen;

	if (ilen <= (sizeof(msg.o.raw) - sizeof(spi_devctl_t))) {
		needscopy = 1;
	}
	else {
		msg.o.data = in;
		msg.o.size = ilen;
	}
	idevctl->u.i.xfer.osize = ilen;

	idevctl->u.i.xfer.iskip = iskip;

	err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	if (needscopy != 0) {
		memcpy(in, odevctl->payload, ilen);
	}

	return odevctl->u.o.err;
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

	if (ss == SPI_SS_EXTERNAL) {
		err = snprintf(devName, sizeof(devName), "/dev/spi%u", dev);
	}
	else {
		err = snprintf(devName, sizeof(devName), "/dev/spi%u.%u", dev, ss);
	}

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
