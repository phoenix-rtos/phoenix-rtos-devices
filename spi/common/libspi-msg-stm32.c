/*
 * Phoenix-RTOS
 *
 * SPI message interface (stm32-multi flavor)
 *
 * Copyright 2025 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <spi.h>
#include <spi-msg.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stm32l4-multi.h>


/* Quirks:
 * - SPI is configured only on spimsg_open open,
 * - iskip field is ignored - no real way to handle it,
 * - mutli's SPI features (cmd, address, etc) are unused,
 * - SPI number is kept in a hacky way in oid.id,
 * - there's no way to pass the SPI speed, only bdiv is accepted instead,
 * - slave select is ignored, hardwired by GPIO config */


int spimsg_xfer(const spimsg_ctx_t *ctx, const void *out, size_t olen, void *in, size_t ilen, size_t iskip)
{
	(void)iskip;

	msg_t msg = { .type = mtDevCtl };
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	msg.i.data = out;
	msg.i.size = olen;
	msg.o.data = in;
	msg.o.size = ilen;

	i->type = spi_rw;
	i->spi_rw.spi = ctx->oid.id;
	i->spi_rw.addr = 0;
	i->spi_rw.flags = 0;
	i->spi_rw.cmd = 0;

	int err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	return msg.o.err;
}


int spimsg_close(const spimsg_ctx_t *ctx)
{
	return 0;
}


int spimsg_open(unsigned int dev, unsigned int ss, spimsg_ctx_t *ctx)
{
	(void)ss;

	unsigned int ntries = 10;
	while (lookup("/dev/multi", NULL, &ctx->oid) < 0) {
		if (--ntries == 0) {
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	/* Hacky - need a way to keep spi no */
	ctx->oid.id = dev;

	/* Configure SPI */
	msg_t msg = { .type = mtDevCtl };
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	i->type = spi_def;
	i->spi_def.spi = dev;
	i->spi_def.enable = 1;
	i->spi_def.mode = ctx->mode;
	i->spi_def.bdiv = ctx->speed; /* Hacky, API specifies clock speed, not div rate */

	int err = msgSend(ctx->oid.port, &msg);
	if (err < 0) {
		return err;
	}

	return msg.o.err;
}
