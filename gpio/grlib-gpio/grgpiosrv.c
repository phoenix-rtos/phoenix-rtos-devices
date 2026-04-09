/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO server
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/debug.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <libgrgpio.h>

/* clang-format off */
#define LOG(fmt, ...)       printf("grgpiosrv: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "grgpiosrv: " fmt "\n", ##__VA_ARGS__)
/* clang-format on */

#define GRGPIO_PRIO 3


static struct {
	oid_t oid;
	grgpio_ctx_t ctx;
} grgpio_common;


static void msgHandler(void)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(grgpio_common.oid.port, &msg, &rid) < 0) { }

		switch (msg.type) {
			case mtDevCtl:
			case mtOpen:
			case mtClose:
				grgpio_handleMsg(&grgpio_common.ctx, &msg);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(grgpio_common.oid.port, &msg, rid);
	}
}


int main(int argc, char *argv[])
{
	oid_t oid;
	char *endptr;
	unsigned long portNum;

	if (argc != 2) {
		LOG_ERROR("Usage: %s <port_number>", argv[0]);
		return EXIT_FAILURE;
	}

	portNum = strtoul(argv[1], &endptr, 10);
	if ((*endptr != '\0') || (argv[1][0] == '\0')) {
		LOG_ERROR("Invalid port number: %s", argv[1]);
		return EXIT_FAILURE;
	}

	if (portCreate(&grgpio_common.oid.port) < 0) {
		LOG_ERROR("Failed to create port");
		return EXIT_FAILURE;
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(100000);
	}

	if (grgpio_init(&grgpio_common.ctx, (unsigned int)portNum) < 0) {
		portDestroy(grgpio_common.oid.port);
		LOG_ERROR("Failed to initialize GPIO port %lu", portNum);
		return EXIT_FAILURE;
	}

	grgpio_common.oid.id = 0;
	if (grgpio_createDev(&grgpio_common.oid, (unsigned int)portNum) < 0) {
		portDestroy(grgpio_common.oid.port);
		LOG_ERROR("Failed to create GPIO device");
		return EXIT_FAILURE;
	}

	LOG("gpio%lu initialized", portNum);
	priority(GRGPIO_PRIO);
	msgHandler();

	return 0;
}
