/*
 * Phoenix-RTOS
 *
 * GRLIB I2C MST driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <i2c.h>
#include <i2c-msg.h>


#define I2C_PRIO 3


static int devctl(msg_t *msg)
{
	i2c_devctl_t *in = (i2c_devctl_t *)msg->i.raw;

	switch (in->i.type) {
		case i2c_devctl_bus_write:
			return i2c_busWrite(in->i.dev_addr, msg->i.data, msg->i.size);

		case i2c_devctl_bus_read:
			return i2c_busRead(in->i.dev_addr, msg->o.data, msg->o.size);

		case i2c_devctl_reg_read:
			return i2c_regRead(in->i.dev_addr, in->i.reg_addr, msg->o.data, msg->o.size);

		default:
			return -EINVAL;
	}
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
				printf("i2c: msgRecv error: %s\n", strerror(-err));
				break;
			}
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = 0;
				break;

			case mtRead:
			case mtWrite:
				msg.o.err = -ENOSYS;
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
	printf("Usage: %s <i2c core>\n", progname);
}


int main(int argc, char **argv)
{
	oid_t oid;
	unsigned int dev;
	uint32_t port;
	char devname[16];

	if (argc != 2) {
		usage(argv[0]);
		return 1;
	}

	char *end;
	dev = strtoul(argv[1], &end, 10);
	if ((*end != '\0') || (end == argv[1])) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	if (i2c_init(dev) < 0) {
		return EXIT_FAILURE;
	}

	if (portCreate(&port) != EOK) {
		return EXIT_FAILURE;
	}

	oid.port = port;
	oid.id = 0;

	snprintf(devname, sizeof(devname), "i2c%u", dev);
	if (create_dev(&oid, devname) < 0) {
		return EXIT_FAILURE;
	}

	puts("i2c: initialized");

	priority(I2C_PRIO);

	thread((void *)((uintptr_t)port));

	puts("i2c: exiting");

	return 0;
}
