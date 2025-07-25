/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL GPIO driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/platform.h>
#include <posix/utils.h>
#include <phoenix/arch/armv7a/imx6ull/imx6ull.h>

#include "imx6ull-gpio.h"


/* WARN: currently, only a single thread is supported */
#define GPIO_THREADS_NO 1


enum { gpio1 = 0, gpio2, gpio3, gpio4, gpio5, dir1, dir2, dir3, dir4, dir5 };


enum { dr = 0, gdir, psr, icr1, icr2, imr, isr, edge };


struct {
	struct {
		volatile uint32_t *base;
#if GPIO_THREADS_NO > 1
		handle_t lock;
#endif
	} gpio[5];

	uint32_t port;
} common;


static const addr_t paddr[] = { 0x0209c000, 0x020a0000, 0x020a4000, 0x020a8000, 0x020ac000 };
static const int clocks[] = { pctl_clk_gpio1, pctl_clk_gpio2, pctl_clk_gpio3, pctl_clk_gpio4, pctl_clk_gpio5 };


int init(void)
{
	int i, err;
	char devpath[sizeof("gpioX/port")];
	platformctl_t pctl;
	oid_t dev;

	for (i = 0; i < sizeof(common.gpio) / sizeof(common.gpio[0]); ++i) {
		if ((common.gpio[i].base = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, paddr[i])) == MAP_FAILED) {
			printf("gpiodrv: Could not map gpio%d paddr %p\n", i + 1, (void*) paddr[i]);
			return -1;
		}
	}

	if (portCreate(&common.port) != EOK) {
		printf("gpiodrv: Could not create port\n");
		return -1;
	}

	for (i = 0; i < sizeof(common.gpio) / sizeof(common.gpio[0]); ++i) {

		snprintf(devpath, sizeof(devpath), "gpio%u/port", (i + 1) % 10);

		dev.port = common.port;
		dev.id = gpio1 + i;

		if ((err = create_dev(&dev, devpath)) != EOK) {
			printf("gpiodrv: Could not create port file #%d (err %d)\n", i + 1, err);
			return - 1;
		}

		snprintf(devpath, sizeof(devpath), "gpio%u/dir", (i + 1) % 10);

		dev.port = common.port;
		dev.id = dir1 + i;

		if ((err = create_dev(&dev, devpath)) != EOK) {
			printf("gpiodrv: Could not create direction file #%d (err %d)\n", i + 1, err);
			return - 1;
		}

		pctl.action = pctl_set;
		pctl.type = pctl_devclock;
		pctl.devclock.state = 3;
		pctl.devclock.dev = clocks[i];

		if (platformctl(&pctl) != EOK) {
			printf("gpiodrv: Could not enable clock for gpio%d\n", i + 1);
			return -1;
		}

#if GPIO_THREADS_NO > 1
		if (mutexCreate(&common.gpio[i].lock) < 0) {
			printf("gpiodrv: Could not create mutex for gpio%d\n", i + 1);
			return -1;
		}
#endif
	}

	return 0;
}


int gpioread(int d, uint32_t *val)
{
	if (d < gpio1 || d > gpio5)
		return -EINVAL;

#if GPIO_THREADS_NO > 1
	mutexLock(common.gpio[d - gpio1].lock);
#endif

	(*val) = *(common.gpio[d - gpio1].base + dr);

#if GPIO_THREADS_NO > 1
	mutexUnlock(common.gpio[d - gpio1].lock);
#endif

	return EOK;
}


int gpiowrite(int d, uint32_t val, uint32_t mask)
{
	uint32_t t;

	if (d < gpio1 || d > gpio5)
		return -EINVAL;

#if GPIO_THREADS_NO > 1
	mutexLock(common.gpio[d - gpio1].lock);
#endif

	t = *(common.gpio[d - gpio1].base + dr) & ~(mask);
	*(common.gpio[d - gpio1].base + dr) = t | (val & mask);

#if GPIO_THREADS_NO > 1
	mutexUnlock(common.gpio[d - gpio1].lock);
#endif

	return EOK;
}


int gpiogetdir(int d, uint32_t *dir)
{
	if (d < dir1 || d > dir5)
		return -EINVAL;

#if GPIO_THREADS_NO > 1
	mutexLock(common.gpio[d - dir1].lock);
#endif

	*(dir) = *(common.gpio[d - dir1].base + gdir);

#if GPIO_THREADS_NO > 1
	mutexUnlock(common.gpio[d - dir1].lock);
#endif

	return EOK;
}


int gpiosetdir(int d, uint32_t dir, uint32_t mask)
{
	uint32_t t;

	if (d < dir1 || d > dir5)
		return -EINVAL;

#if GPIO_THREADS_NO > 1
	mutexLock(common.gpio[d - dir1].lock);
#endif

	t = *(common.gpio[d - dir1].base + gdir) & ~(mask);
	*(common.gpio[d - dir1].base + gdir) = t | (dir & mask);

#if GPIO_THREADS_NO > 1
	mutexUnlock(common.gpio[d - dir1].lock);
#endif

	return EOK;
}


void thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	int d;
	uint32_t val, mask;

	while (1) {
		if (msgRecv(common.port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = (msg.oid.id < gpio1) || (msg.oid.id > dir5) ? -ENOENT : EOK;
				break;

			case mtRead:
				if (msg.o.data != NULL && msg.o.size >= sizeof(uint32_t)) {
					d = msg.oid.id;
					/* NOTE: ignoring msg.i.io.offs to allow repetetive reads without seek / reopen */

					if (d >= gpio1 && d <= gpio5) {
						msg.o.err = gpioread(d, &val);
						memcpy(msg.o.data, &val, sizeof(uint32_t));
					}
					else if (d >= dir1 && d <= dir5) {
						msg.o.err = gpiogetdir(d, &val);
						memcpy(msg.o.data, &val, sizeof(uint32_t));
					}

					if (msg.o.err == EOK)
						msg.o.err = sizeof(uint32_t);
				}
				break;

			case mtWrite:
				if (msg.i.data != NULL && msg.i.size >= 2) {
					const uint8_t *data = msg.i.data;

					/* NOTE: ignoring msg.i.io.offs to allow repetetive writes without seek / reopen */

					if ((msg.i.size % sizeof(uint32_t) != 0) && ((data[0] == '+') || (data[0] == '-'))) {
						/* text mode [+/-][pin] -> 1/0 as out/in or value */
						uint32_t pin = 0;
						for (d = 1; d < msg.i.size && isdigit(data[d]); ++d)
							pin = pin * 10 + (data[d] - '0');

						if (pin > 31)
							break; /* -EINVAL */

						mask = 1 << pin;
						val = ((data[0] == '+') ? 1 : 0) << pin;
					}
					else { /* binary mode */
						if (msg.i.size < sizeof(uint32_t))
							break; /* -EINVAL */

						memcpy(&val, msg.i.data, sizeof(uint32_t));

						if (msg.i.size >= sizeof(uint32_t) * 2)
							memcpy(&mask, msg.i.data + sizeof(uint32_t), sizeof(uint32_t));
						else
							mask = (uint32_t)-1;
					}

					d = msg.oid.id;

					if (d >= gpio1 && d <= gpio5)
						msg.o.err = gpiowrite(d, val, mask);
					else if (d >= dir1 && d <= dir5)
						msg.o.err = gpiosetdir(d, val, mask);

					if (msg.o.err == EOK)
						msg.o.err = msg.i.size;
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(common.port, &msg, rid);
	}
}


static int parse_args(int argc, char *argv[])
{
	int opt;
	unsigned long prio;
	char *endptr;

	while ((opt = getopt(argc, argv, "p:")) != -1) {
		switch (opt) {
			case 'p':
				prio = strtoul(optarg, &endptr, 0);
				if (*endptr != '\0') {
					printf("%s: incorrect priority value (%s)\n", argv[0], optarg);
					return -1;
				}

				if (prio > 7) {
					printf("%s: incorrect priority value (%lu). It must be in [0,7] range\n",
						argv[0], prio);
					return -1;
				}

				priority(prio);
				break;

			default:
				printf("gpio driver for imx6ull. Usage:\n");
				printf("%s [-p priority]\n", argv[0]);
				printf("\t-p priority\t\tSelect priority for the gpio driver\n");
				return -1;
		}
	}

	return 0;
}


int main(int argc, char *argv[])
{
	oid_t root;

	if (parse_args(argc, argv) < 0)
		return 1;

	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0)
		usleep(10000);

	/* Wait for the console */
	while (write(1, "", 0) < 0)
		usleep(10000);

	if (init())
		return -EIO;

	thread(NULL);

	return 0;
}
