/*
 * Phoenix-RTOS
 *
 * Zynq-7000 GPIO server
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <getopt.h>
#include <stdlib.h>
#include <string.h>

#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <posix/utils.h>

#include "zynq7000-gpio-msg.h"
#include "gpio.h"


/* Default server thread priority */
#define PRIORITY 2


/* GPIO oid.id encoding */
#define GPIO_BANK_BIT 7 /* Bank */
#define GPIO_PORT_BIT 6 /* Port */
#define GPIO_DIR_BIT  5 /* Dir */

#define GPIO_BANK (1 << GPIO_BANK_BIT)
#define GPIO_PORT (1 << GPIO_PORT_BIT)
#define GPIO_DIR  (1 << GPIO_DIR_BIT)
#define GPIO_PIN  (31 << 0)


static void gpiosrv_devctl(msg_t *msg)
{
	gpio_devctl_t *in = (gpio_devctl_t *)msg->i.raw;
	gpio_devctl_t *out = (gpio_devctl_t *)msg->o.raw;
	unsigned int bank = (in->i.oid.id & GPIO_BANK) ? 1 : 0;
	unsigned int pin = in->i.oid.id & GPIO_PIN;
	uint32_t val;

	switch (in->i.type) {
		case gpio_devctl_read_pin:
			out->o.err = gpio_readPin(bank, pin, &val);
			out->o.val = val;
			break;

		case gpio_devctl_write_pin:
			out->o.err = gpio_writePin(bank, pin, in->i.val);
			break;

		case gpio_devctl_read_port:
			out->o.err = gpio_readPort(bank, &val);
			out->o.val = val;
			break;

		case gpio_devctl_write_port:
			out->o.err = gpio_writePort(bank, in->i.val, in->i.mask);
			break;

		case gpio_devctl_read_dir:
			out->o.err = gpio_readDir(bank, &val);
			out->o.val = val;
			break;

		case gpio_devctl_write_dir:
			out->o.err = gpio_writeDir(bank, in->i.val, in->i.mask);
			break;

		default:
			out->o.err = -ENOSYS;
	}
}


static void gpiosrv_msgthr(void *arg)
{
	unsigned int bank, pin, port = (unsigned int)arg;
	msg_rid_t rid;
	char buff[16];
	uint32_t val;
	msg_t msg;
	int err;

	for (;;) {
		err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			printf("zynq7000-gpio: failed to receive message, err: %s\n", strerror(err));
			break;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			case mtDevCtl:
				gpiosrv_devctl(&msg);
				break;

			case mtRead:
				bank = (msg.i.io.oid.id & GPIO_BANK) ? 1 : 0;
				pin = msg.i.io.oid.id & GPIO_PIN;

				if (msg.i.io.oid.id & GPIO_PORT) {
					err = gpio_readPort(bank, &val);
				}
				else if (msg.i.io.oid.id & GPIO_DIR) {
					err = gpio_readDir(bank, &val);
				}
				else {
					err = gpio_readPin(bank, pin, &val);
				}

				if (err < 0) {
					msg.o.io.err = err;
				}
				else if ((msg.o.data == NULL) || (msg.o.size == 0)) {
					msg.o.io.err = 0;
				}
				else {
					err = sprintf(buff, "%u\n", val) + 1;
					if (err <= msg.i.io.offs) {
						msg.o.io.err = 0;
					}
					else {
						err -= msg.i.io.offs;
						if (err > msg.o.size) {
							err = msg.o.size;
						}
						buff[msg.i.io.offs + err - 1] = '\0';
						strncpy(msg.o.data, buff + msg.i.io.offs, err);
						msg.o.io.err = err;
					}
				}
				break;

			case mtWrite:
				bank = (msg.i.io.oid.id & GPIO_BANK) ? 1 : 0;
				pin = msg.i.io.oid.id & GPIO_PIN;

				if ((msg.i.data == NULL) || (msg.i.size == 0)) {
					msg.o.io.err = 0;
				}
				else {
					err = sizeof(buff);
					if (msg.i.size < err) {
						err = msg.i.size;
					}
					strncpy(buff, msg.i.data, err);
					buff[err - 1] = '\0';
					val = strtoul(buff, NULL, 0);

					if (msg.i.io.oid.id & GPIO_PORT) {
						err = gpio_writePort(bank, val, 0xffffffff);
					}
					else if (msg.i.io.oid.id & GPIO_DIR) {
						err = gpio_writeDir(bank, val, 0xffffffff);
					}
					else {
						err = gpio_writePin(bank, pin, val);
					}
					msg.o.io.err = (err < 0) ? err : msg.i.size;
				}
				break;

			case mtGetAttr:
			case mtSetAttr:
				msg.o.attr.err = -ENOSYS;
				break;

			default:
				msg.o.io.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void gpiosrv_usage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("Options:\n");
	printf("\t-p priority - server thread priority (default %u)\n", PRIORITY);
	printf("\t-h          - this help message\n");
}


int main(int argc, char *argv[])
{
	int err, c, prio = PRIORITY;
	unsigned int i, j;
	char devName[16];
	oid_t oid;

	while ((c = getopt(argc, argv, "p:h")) != -1) {
		switch (c) {
			case 'p':
				prio = atoi(optarg);
				break;

			case 'h':
				gpiosrv_usage(argv[0]);
				return EXIT_SUCCESS;

			default:
				gpiosrv_usage(argv[0]);
				return EXIT_FAILURE;
		}
	}

	err = gpio_init();
	if (err < 0) {
		printf("zynq7000-gpio: failed to initialize controller, err: %s\n", strerror(err));
		return EXIT_FAILURE;
	}

	/* Wait for rootfs before creating device files */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(10000);
	}

	err = portCreate(&oid.port);
	if (err < 0) {
		printf("zynq7000-gpio: failed to create server port, err: %s\n", strerror(err));
		return EXIT_FAILURE;
	}

	for (i = 0; i < GPIO_BANKS; i++) {
		oid.id = (i << GPIO_BANK_BIT) | GPIO_PORT;
		snprintf(devName, sizeof(devName), "gpio%u/port", i);
		err = create_dev(&oid, devName);
		if (err < 0) {
			printf("zynq7000-gpio: failed to create device file %s, err: %s\n", devName, strerror(err));
			return EXIT_FAILURE;
		}

		oid.id = (i << GPIO_BANK_BIT) | GPIO_DIR;
		snprintf(devName, sizeof(devName), "gpio%u/dir", i);
		err = create_dev(&oid, devName);
		if (err < 0) {
			printf("zynq7000-gpio: failed to create device file %s, err: %s\n", devName, strerror(err));
			return EXIT_FAILURE;
		}

		for (j = 0; j < GPIO_PINS; j++) {
			/* Skip not configured pins */
			if (gpioPins[i][j] < 0) {
				continue;
			}

			oid.id = (i << GPIO_BANK_BIT) | j;
			snprintf(devName, sizeof(devName), "gpio%u/pin%u", i, j);
			err = create_dev(&oid, devName);
			if (err < 0) {
				printf("zynq7000-gpio: failed to create device file %s, err: %s\n", devName, strerror(err));
				return EXIT_FAILURE;
			}
		}
	}

	priority(prio);
	gpiosrv_msgthr((void *)oid.port);

	return EXIT_SUCCESS;
}
