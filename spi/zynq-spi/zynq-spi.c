/*
 * Phoenix-RTOS
 *
 * Zynq7000 / ZynqMP SPI server
 *
 * Copyright 2025 Phoenix Systems
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
#include <string.h>
#include <unistd.h>

#include <posix/utils.h>
#include <board_config.h>


/* Number of SPI controllers */
#define SPI_COUNT 2


/* SPI slave devices */
static const int devs[][SPI_COUNT] = {
	{ SPI0_SS0, SPI1_SS0 },
	{ SPI0_SS1, SPI1_SS1 },
	{ SPI0_SS2, SPI1_SS2 },
};


static struct {
	unsigned int dev;
} spisrv_common;


static void spisrv_devctl(msg_t *msg)
{
	spi_devctl_t *in = (spi_devctl_t *)msg->i.raw;
	spi_devctl_t *out = (spi_devctl_t *)msg->o.raw;
	const void *idata = msg->i.data;
	void *odata = msg->o.data;

	switch (in->i.type) {
		case spi_devctl_xfer:
			msg->o.err = spi_setMode(spisrv_common.dev, in->i.ctx.mode);
			if (msg->o.err < 0) {
				break;
			}

			msg->o.err = spi_setSpeed(spisrv_common.dev, in->i.ctx.speed);
			if (msg->o.err < 0) {
				break;
			}

			/* Unpack msg */
			if ((in->i.xfer.isize != 0) && (msg->i.data == NULL)) {
				idata = in->payload;
			}

			if ((in->i.xfer.osize != 0) && (msg->o.data == NULL)) {
				odata = out->payload;
			}

			msg->o.err = spi_xfer(spisrv_common.dev, in->i.ctx.oid.id, idata, in->i.xfer.isize, odata, in->i.xfer.osize, in->i.xfer.iskip);
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


static void spisrv_msgthr(void *arg)
{
	unsigned int port = (unsigned int)(intptr_t)arg;
	msg_rid_t rid;
	msg_t msg;
	int err;

	for (;;) {
		err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			printf("zynq-spi: failed to receive message for SPI%u controller, err: %s\n", spisrv_common.dev, strerror(err));
			break;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtDevCtl:
				spisrv_devctl(&msg);
				break;

			default:
				msg.o.err = -ENOSYS;
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

	if (argc != 2) {
		spisrv_help(argv[0]);
		return EXIT_FAILURE;
	}

	dev = strtoul(argv[1], NULL, 0);
	if (dev >= SPI_COUNT) {
		spisrv_help(argv[0]);
		return EXIT_FAILURE;
	}

	err = spi_init(dev);
	if (err < 0) {
		printf("zynq-spi: failed to initialize SPI%u controller, err: %s\n", dev, strerror(err));
		return EXIT_FAILURE;
	}

	/* Wait for rootfs before creating device files */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(10000);
	}

	err = portCreate(&oid.port);
	if (err < 0) {
		printf("zynq-spi: failed to create port for SPI%u controller, err: %s\n", dev, strerror(err));
		return EXIT_FAILURE;
	}

	/* Create device for use with external SS lines */
	sprintf(devName, "spi%u", dev);
	oid.id = SPI_SS_EXTERNAL;
	err = create_dev(&oid, devName);
	if (err < 0) {
		printf("zynq-spi: failed to create device file %s, err: %s\n", devName, strerror(err));
		return EXIT_FAILURE;
	}

	for (i = 0; i < sizeof(devs) / sizeof(devs[0]); i++) {
		/* Skip not configured slave devices */
		if (devs[i][dev] < 0) {
			continue;
		}

		sprintf(devName, "spi%u.%u", dev, i);
		oid.id = i;
		err = create_dev(&oid, devName);
		if (err < 0) {
			printf("zynq-spi: failed to create device file %s, err: %s\n", devName, strerror(err));
			return EXIT_FAILURE;
		}
	}

	spisrv_common.dev = dev;
	spisrv_msgthr((void *)(intptr_t)oid.port);

	return EXIT_SUCCESS;
}
