/*
 * Phoenix-RTOS
 *
 * Sensors SPI communication interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <libgen.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/threads.h>

#include <zynq7000-gpio-msg.h>

#include "sensors-spi.h"


static struct {
	handle_t lock; /* SPI bus lock */
} sensorsspi_common;


int sensorsspi_xfer(const spimsg_ctx_t *ctx, oid_t *ss, const void *out, size_t olen, void *in, size_t ilen, size_t iskip)
{
	int ret;

	mutexLock(sensorsspi_common.lock);

	/* Manually assert SS line */
	if (ctx->oid.id == SPI_SS_EXTERNAL) {
		gpiomsg_writePin(ss, 0);
	}

	ret = spimsg_xfer(ctx, out, olen, in, ilen, iskip);

	/* Manually deassert SS line */
	if (ctx->oid.id == SPI_SS_EXTERNAL) {
		gpiomsg_writePin(ss, 1);
	}

	mutexUnlock(sensorsspi_common.lock);

	return ret;
}


int sensorsspi_open(const char *devSPI, const char *devSS, oid_t *spi, oid_t *ss)
{
	char *path, *dir, *base;
	unsigned int pin;
	int err, ntries;
	oid_t oid;

	if ((devSPI == NULL) || (spi == NULL)) {
		return -EINVAL;
	}

	/* Get SPI device */
	ntries = 10;
	while (lookup(devSPI, NULL, spi) < 0) {
		ntries--;
		if (ntries == 0) {
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	if (spi->id != SPI_SS_EXTERNAL) {
		return EOK;
	}

	if ((devSS == NULL) || (ss == NULL)) {
		return -EINVAL;
	}

	/* Get external SS device */
	ntries = 10;
	while (lookup(devSS, NULL, ss) < 0) {
		ntries--;
		if (ntries == 0) {
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	path = strdup(devSS);
	if (path == NULL) {
		return -ENOMEM;
	}

	/* Get pin number */
	base = basename(path);
	if (strncmp(base, "pin", 3)) {
		free(path);
		return -EINVAL;
	}
	pin = strtoul(base + 3, NULL, 0);

	/* Configure pin as output */
	dir = dirname(path);
	sprintf(dir, "%s/dir", dir);

	ntries = 10;
	while (lookup(dir, NULL, &oid) < 0) {
		ntries--;
		if (ntries == 0) {
			free(path);
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	err = gpiomsg_writeDir(&oid, 1 << pin, 1 << pin);
	if (err < 0) {
		free(path);
		return err;
	}

	/* Raise SS pin high */
	dir = dirname(path);
	sprintf(dir, "%s/port", dir);

	ntries = 10;
	while (lookup(dir, NULL, &oid) < 0) {
		ntries--;
		if (ntries == 0) {
			free(path);
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	err = gpiomsg_writePort(&oid, 1 << pin, 1 << pin);
	if (err < 0) {
		free(path);
		return err;
	}

	free(path);

	return EOK;
}


int sensorsspi_init(void)
{
	int err;

	err = mutexCreate(&sensorsspi_common.lock);
	if (err < 0) {
		return err;
	}

	return EOK;
}
