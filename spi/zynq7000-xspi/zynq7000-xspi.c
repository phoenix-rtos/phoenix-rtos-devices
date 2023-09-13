/*
 * Phoenix-RTOS
 *
 * Zynq-7000 AXI SPI server
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski, Aleksander Kaminski
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
#include <string.h>
#include <unistd.h>
#include <sys/threads.h>

#include <posix/utils.h>
#include <board_config.h>

#include "libzynq7000-xspi.h"


/* Number of SPI controllers */
#define SPI_COUNT 1

#define PRIORITY 2

static struct {
	unsigned int dev;
} spisrv_common;


static void spisrv_devctl(msg_t *msg)
{
	spi_devctl_t *in = (spi_devctl_t *)msg->i.raw;
	spi_devctl_t *out = (spi_devctl_t *)msg->o.raw;
	void *idata = msg->i.data;
	void *odata = msg->o.data;

	switch (in->u.i.type) {
		case spi_devctl_xfer:
			out->u.o.err = spi_setMode(spisrv_common.dev, in->u.i.ctx.mode);
			if (out->u.o.err < 0) {
				break;
			}

			out->u.o.err = spi_setSpeed(spisrv_common.dev, in->u.i.ctx.speed);
			if (out->u.o.err < 0) {
				break;
			}

			/* Unpack msg */
			if ((in->u.i.xfer.isize != 0) && (msg->i.data == NULL)) {
				idata = in->payload;
			}

			if ((in->u.i.xfer.osize != 0) && (msg->o.data == NULL)) {
				odata = out->payload;
			}

			out->u.o.err = spi_xfer(spisrv_common.dev, in->u.i.ctx.oid.id, idata, in->u.i.xfer.isize, odata, in->u.i.xfer.osize, in->u.i.xfer.iskip);
			break;

		default:
			out->u.o.err = -EINVAL;
			break;
	}
}


static void spisrv_msgthr(void *arg)
{
	unsigned int port = (unsigned int)arg;
	msg_rid_t rid;
	msg_t msg;
	int err;

	for (;;) {
		err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			printf("zynq7000-xspi: failed to receive message for SPI%u controller, err: %s\n", spisrv_common.dev, strerror(err));
			break;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			case mtDevCtl:
				spisrv_devctl(&msg);
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void spisrv_help(const char *progname)
{
	printf("Usage: %s [spi_bus <0..%u>]\n", progname, SPI_COUNT - 1);
}


int main(int argc, char *argv[])
{
	unsigned int i, dev;
	char devName[16];
	oid_t oid;
	int err;

	if (argc == 1) {
		dev = 0;
	}
	else if (argc != 2) {
		spisrv_help(argv[0]);
		return EXIT_FAILURE;
	}
	else {
		dev = strtoul(argv[1], NULL, 0);
		if (dev >= SPI_COUNT) {
			spisrv_help(argv[0]);
			return EXIT_FAILURE;
		}
	}

	priority(PRIORITY);

	err = spi_init(dev);
	if (err < 0) {
		printf("zynq7000-xspi: failed to initialize SPI%u controller, err: %s\n", dev, strerror(err));
		return EXIT_FAILURE;
	}

	/* Wait for rootfs before creating device files */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(10000);
	}

	err = portCreate(&oid.port);
	if (err < 0) {
		printf("zynq7000-xspi: failed to create port for SPI%u controller, err: %s\n", dev, strerror(err));
		return EXIT_FAILURE;
	}

	/* Create device for use with external SS lines */
	sprintf(devName, "spi%u", dev);
	oid.id = SPI_SS_EXTERNAL;
	err = create_dev(&oid, devName);
	if (err < 0) {
		printf("zynq7000-xspi: failed to create device file %s, err: %s\n", devName, strerror(err));
		return EXIT_FAILURE;
	}

	for (i = 0; i < SPI_SS_COUNT; i++) {
		sprintf(devName, "spi%u.%u", dev, i);
		oid.id = i;
		err = create_dev(&oid, devName);
		if (err < 0) {
			printf("zynq7000-xspi: failed to create device file %s, err: %s\n", devName, strerror(err));
			return EXIT_FAILURE;
		}
	}

	spisrv_common.dev = dev;
	spisrv_msgthr((void *)oid.port);

	return EXIT_SUCCESS;
}
