/*
 * Phoenix-RTOS
 *
 * armv7a9-zynq7000 platform support package (PSP)
 *
 * bus access
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <libgen.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/threads.h>

#include <spi.h>
#include <spi-msg.h>
#include <zynq7000-gpio-msg.h>

#include <libsensors/sensor.h>
#include <libsensors/bus.h>


struct sensor_busctx_spi_t {
	spimsg_ctx_t ctx;
	oid_t ss;
};


struct {
	handle_t lock; /* SPI bus lock */
} psp_common;


static int psp_spiXfer(const struct _sensor_bus_t *bus, const void *out, size_t olen, void *in, size_t ilen, size_t iskip)
{
	const spimsg_ctx_t *ctx = &bus->ctx.spi->ctx;
	oid_t *ss = &bus->ctx.spi->ss;
	int ret;

	mutexLock(psp_common.lock);

	/* Manually assert SS line */
	if (ctx->oid.id == SPI_SS_EXTERNAL) {
		gpiomsg_writePin(ss, 0);
	}

	ret = spimsg_xfer(ctx, out, olen, in, ilen, iskip);

	/* Manually deassert SS line */
	if (ctx->oid.id == SPI_SS_EXTERNAL) {
		gpiomsg_writePin(ss, 1);
	}

	mutexUnlock(psp_common.lock);

	return ret;
}


static int psp_spiCfg(struct _sensor_bus_t *bus, int speed, char mode)
{
	bus->ctx.spi->ctx.mode = mode;
	bus->ctx.spi->ctx.speed = speed;

	return 0;
}


static int psp_spiClose(struct _sensor_bus_t *bus)
{
	free(bus->ctx.spi);

	return 0;
}


static int psp_openOid(const char *devSPI, const char *devSS, oid_t *spi, oid_t *ss)
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
	strcat(dir, "/dir");

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
	strcat(dir, "/port");

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


static int psp_spiOpen(struct _sensor_bus_t *bus, const char *devSPI, const char *devSS)
{
	struct sensor_busctx_spi_t *spi = bus->ctx.spi;

	return psp_openOid(devSPI, devSS, &spi->ctx.oid, &spi->ss);
}


void bsp_busDealloc(struct _sensor_bus_t *bus)
{
	switch (bus->bustype) {
		case bus_spi:
			free(bus->ctx.spi);
			break;
		case bus_i2c:
			free(bus->ctx.i2c);
			break;
		default:
			/* erroneous state, do nothing */
			break;
	}
}


int psp_busAlloc(sensor_bus_t *bus, enum sensor_bus_type type)
{
	const sensor_bus_t spi = {
		.bustype = bus_spi,
		.ops.bus_xfer = psp_spiXfer,
		.ops.bus_cfg = psp_spiCfg,
		.ops.bus_close = psp_spiClose,
		.ops.bus_open = psp_spiOpen,
	};

	int size;
	const sensor_bus_t *proto;

	switch (type) {
		case bus_spi:
			proto = &spi;
			size = sizeof(struct sensor_busctx_spi_t);
			break;
		case bus_i2c:
			/* #TODO: add different buses setup */
		default:
			return -ENODEV;
	}

	/* malloc at 'spi' even for different buses. Does not matter after all it is only memory space */
	*bus = *proto;
	bus->ctx.spi = malloc(size);

	if (bus->ctx.spi == NULL) {
		return -ENOMEM;
	}

	return 0;
}


void psp_busDone(void)
{
	resourceDestroy(psp_common.lock);
}


int psp_busInit(void)
{
	int err = mutexCreate(&psp_common.lock);
	if (err < 0) {
		return err;
	}

	return 0;
}
