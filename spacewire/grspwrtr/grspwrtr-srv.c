/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Andrzej Tlomak, Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <posix/utils.h>
#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/platform.h>

#include <grspwrtr-srv.h>
#include <libgrspwrtr.h>


/* clang-format off */
#define TRACE(fmt, ...) do { if (0) { printf("%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
#define LOG(fmt, ...)       printf("spacewire-router: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "spacewire-router: " fmt "\n", ##__VA_ARGS__)
/* clang-format on */

#define SPWRTR_PRIO 3


static struct {
	oid_t oid;
	spwrtr_dev_t dev;
} spwrtr_common;


/* Message handling */


static void spwrtr_handleDevCtl(msg_t *msg)
{
	spwrtr_i_t *ictl = (spwrtr_i_t *)msg->i.raw;
	spwrtr_o_t *octl = (spwrtr_o_t *)msg->o.raw;

	int err;
	switch (ictl->type) {
		case spwrtr_pmap_set:
			err = spwrtr_setPortMapping(&spwrtr_common.dev, ictl->task.mapping.port, ictl->task.mapping.enPorts);
			break;

		case spwrtr_pmap_get: {
			uint32_t map = 0;
			err = spwrtr_getPortMapping(&spwrtr_common.dev, ictl->task.mapping.port, &map);
			octl->val = map;
			break;
		}

		case spwrtr_clkdiv_set:
			err = spwrtr_setClockDiv(&spwrtr_common.dev, ictl->task.clkdiv.port, ictl->task.clkdiv.div);
			break;

		case spwrtr_clkdiv_get: {
			uint32_t div = 0;
			err = spwrtr_getClockDiv(&spwrtr_common.dev, ictl->task.clkdiv.port, &div);
			octl->val = div;
			break;
		}

		case spwrtr_reset:
			err = spwrtr_resetDevice(&spwrtr_common.dev);
			break;

		case spwrtr_route_set:
			err = spwrtr_setRoute(&spwrtr_common.dev, ictl->task.route.addr, ictl->task.route.enPorts, ictl->task.route.ctrl);
			break;

		case spwrtr_route_get: {
			uint32_t map = 0;
			uint32_t ctrl = 0;
			err = spwrtr_getPortMapping(&spwrtr_common.dev, ictl->task.route.addr, &map);
			if (err == 0) {
				err = spwrtr_getAddrCtrl(&spwrtr_common.dev, ictl->task.route.addr, &ctrl);
			}
			octl->route.map = map;
			octl->route.ctrl = ctrl;
			break;
		}

		case spwrtr_route_clear:
			err = spwrtr_clearRoute(&spwrtr_common.dev, ictl->task.route.addr);
			break;

		case spwrtr_port_status: {
			uint32_t sts = 0;
			err = spwrtr_getPortStatus(&spwrtr_common.dev, ictl->task.port.port, &sts);
			octl->val = sts;
			break;
		}

		case spwrtr_port_errclr: {
			uint32_t errs = 0;
			err = spwrtr_clearPortErrors(&spwrtr_common.dev, ictl->task.port.port, &errs);
			octl->val = errs;
			break;
		}

		default:
			err = -EINVAL;
			break;
	}
	msg->o.err = err;
}


static void spwrtr_msgLoop(void)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(spwrtr_common.oid.port, &msg, &rid) < 0) {
		}
		switch (msg.type) {
			case mtDevCtl:
				spwrtr_handleDevCtl(&msg);
				break;

			case mtRead:
			case mtWrite:
			case mtOpen:
			case mtClose:
				msg.o.err = 0;
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(spwrtr_common.oid.port, &msg, rid);
	}
}


/* Initialization */


static int spwrtr_createDevs(oid_t *oid, int n)
{
	char buf[11];
	if (snprintf(buf, sizeof(buf), "spwrtr%d", n) >= sizeof(buf)) {
		return -1;
	}

	if (create_dev(oid, buf) < 0) {
		return -1;
	}
	return 0;
}


static void spwrtr_usage(const char *progname)
{
	printf("Usage: %s [options]\n", progname);
	printf("Options:\n");
	printf("\t-n <id> - spwrtr core id\n");
}


int main(int argc, char **argv)
{
	oid_t oid;
	int c;
	int spwrtrn = 0;

	if (argc > 1) {
		do {
			c = getopt(argc, argv, "n:");
			switch (c) {
				case 'n':
					spwrtrn = atoi(optarg);
					break;

				case -1:
					break;

				default:
					spwrtr_usage(argv[0]);
					return EXIT_FAILURE;
			}
		} while (c != -1);
	}

	if (portCreate(&spwrtr_common.oid.port) < 0) {
		LOG_ERROR("Failed to create port");
		return EXIT_FAILURE;
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(100000);
	}

	if (spwrtr_init(&spwrtr_common.dev, spwrtrn) < 0) {
		portDestroy(spwrtr_common.oid.port);
		LOG_ERROR("Failed to initialize SpaceWire-router");
		return EXIT_FAILURE;
	}

	if (spwrtr_createDevs(&spwrtr_common.oid, spwrtrn) < 0) {
		portDestroy(spwrtr_common.oid.port);
		LOG_ERROR("Failed to create SpaceWire-router device");
		return EXIT_FAILURE;
	}

	LOG("initialized");
	(void)setPriority(SPWRTR_PRIO);
	spwrtr_msgLoop();

	return 0;
}
