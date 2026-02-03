/*
 * Phoenix-RTOS
 *
 * GRLIB SPI CTRL driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <sys/msg.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <spi-mst.h>
#include <spi-mst-msg.h>

#include "libgrspictrl.h"


#define SPI_PRIO 3


static struct {
	unsigned int dev;
	spi_ctx_t ctx;
} common;


static int devctl(msg_t *msg)
{
	spi_devctl_t *in = (spi_devctl_t *)msg->i.raw;

	int err = spi_setBitOrder(&common.ctx, in->cfg.bitOrder);
	if (err < 0) {
		return err;
	}

	err = spi_setMode(&common.ctx, in->cfg.mode);
	if (err < 0) {
		return err;
	}

	err = spi_setSpeed(&common.ctx, in->cfg.speed);
	if (err < 0) {
		return err;
	}

	const spi_transferDesc_t *descriptors;
	if (msg->i.data == NULL) {
		descriptors = (void *)in->payload;
	}
	else {
		descriptors = msg->i.data;
	}
	const void *txBuf = &descriptors[in->cfg.descCnt];
	void *rxBuf = (msg->o.data != NULL) ? msg->o.data : msg->o.raw;

	return spi_transaction(&common.ctx, msg->oid.id, descriptors, in->cfg.descCnt, txBuf, rxBuf);
}


static void thread(void *arg)
{
	uint32_t port = (uint32_t)((uintptr_t)arg);
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		int err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			else {
				printf("spi: msgRecv error: %s\n", strerror(-err));
				break;
			}
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = 0;
				break;

			case mtDevCtl:
				msg.o.err = devctl(&msg);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void usage(const char *progname)
{
	printf("Usage: %s <spi core>\n", progname);
}


int main(int argc, char **argv)
{
	if (argc != 2) {
		usage(argv[0]);
		return 1;
	}

	char *end;
	common.dev = strtoul(argv[1], &end, 10);
	if ((*end != '\0') || (end == argv[1])) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	if (spi_init(common.dev, &common.ctx) < 0) {
		return EXIT_FAILURE;
	}

	oid_t oid;
	/* Wait for rootfs before creating device files */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(10000);
	}

	if (portCreate(&oid.port) != EOK) {
		return EXIT_FAILURE;
	}

	char devname[16];
	if (snprintf(devname, sizeof(devname), "spi%u", common.dev) >= sizeof(devname)) {
		return EXIT_FAILURE;
	}

	oid.id = SPI_SS_EXTERNAL;
	if (create_dev(&oid, devname) < 0) {
		return EXIT_FAILURE;
	}

	for (unsigned int i = 0; i < common.ctx.ssCount; i++) {
		if (snprintf(devname, sizeof(devname), "spi%u.%u", common.dev, i) >= sizeof(devname)) {
			return EXIT_FAILURE;
		}
		oid.id = i;
		if (create_dev(&oid, devname) < 0) {
			return EXIT_FAILURE;
		}
	}

	puts("spi: initialized");

	priority(SPI_PRIO);

	thread((void *)((uintptr_t)oid.port));

	puts("spi: exiting");

	return 0;
}
