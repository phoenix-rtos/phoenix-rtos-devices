/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL I2C BUS driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <posix/utils.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <i2c.h>
#include <i2c-msg.h>


static int dev_ctl(msg_t *msg)
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
	uint32_t port = (uint32_t)arg;
	msg_t msg;
	unsigned long int rid;
	int err;

	for (;;) {
		if ((err = msgRecv(port, &msg, &rid)) < 0) {
			printf("i2c: msgRecv returned error: %s\n", strerror(-err));
			if (err == -EINTR)
				continue;
			else
				break; /* EINVAL (invalid/closed port) or ENOMEM - fatal error */
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;
			case mtRead:
			case mtWrite:
				msg.o.io.err = -EINVAL;
				break;
			case mtDevCtl: {
				i2c_devctl_t *out = (i2c_devctl_t *)msg.o.raw;
				out->o.err = dev_ctl(&msg);
				break;
			}
			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [i2c_bus_no <1,4>]\n", progname);
}


int main(int argc, char **argv)
{
	oid_t dev;
	unsigned int dev_no;
	uint32_t port;
	char devname[sizeof("i2cX")];

	if (argc != 2) {
		print_usage(argv[0]);
		return 1;
	}

	dev_no = atoi(argv[1]);
	if (i2c_init(dev_no) < 0) {
		printf("i2c: initialization failed\n");
		return 1;
	}

	if (portCreate(&port) != EOK) {
		printf("i2c: port allocation failed\n");
		return 2;
	}

	dev.port = port;
	dev.id = 0;

	snprintf(devname, sizeof(devname), "i2c%u", dev_no % 10);
	if (create_dev(&dev, devname) < 0) {
		printf("i2c: could not create device\n");
		return 3;
	}

	puts("i2c: initialized");
	thread((void *)port);

	printf("i2c: exiting\n");
	return 0;
}
