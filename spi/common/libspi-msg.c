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
	msg_t msg = { 0 };
	spi_devctl_t *idevctl = (spi_devctl_t *)msg.i.raw;
	spi_devctl_t *odevctl = (spi_devctl_t *)msg.o.raw;
	int err;
	unsigned int rawMode = 0;

	if (ctx == NULL) {
		return -EINVAL;
	}

	if (olen < 256 && ilen < 256 && (olen + sizeof(spi_devctl_t) + 2) <= sizeof(msg.i.raw) && (ilen + sizeof(spi_devctl_t)) < sizeof(msg.o.raw)) {
		/*
		 * Raw transmission mode stores data in msg.i.raw/msg.o.raw in the following form:
		 * msg.i.raw: [spi_devctl_t], [1-byte olen], [1-byte ilen], [out data]
		 * msg.o.raw: [spi_devctl_t], [in data]
		 */
		rawMode = 1;
	}

	msg.type = mtDevCtl;
	idevctl->i.type = spi_devctl_xfer;
	idevctl->i.ctx = *ctx;
	idevctl->i.iskip = iskip;

	if (rawMode) {
		msg.i.raw[sizeof(spi_devctl_t)] = (unsigned char)olen;
		msg.i.raw[sizeof(spi_devctl_t) + 1] = (unsigned char)ilen;
		memcpy(&msg.i.raw[sizeof(spi_devctl_t) + 2], out, olen);
	}
	else {
		msg.i.data = (void *)out;
		msg.i.size = olen;
		msg.o.data = in;
		msg.o.size = ilen;
	}

	err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	if (rawMode) {
		memcpy(in, &msg.o.raw[sizeof(spi_devctl_t)], ilen);
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
