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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/platform.h>
#include <posix/utils.h>
#include <phoenix/arch/imx6ull.h>

#include "imx6ull-gpio.h"


enum { gpio1 = 0, gpio2, gpio3, gpio4, gpio5, dir1, dir2, dir3, dir4, dir5 };


enum { dr = 0, gdir, psr, icr1, icr2, imr, isr, edge };


struct {
	struct {
		volatile uint32_t *base;
		id_t port;
		id_t dir;
		handle_t lock;
	} gpio[5];

	int port;
} common;


static const addr_t paddr[] = { 0x0209c000, 0x020a0000, 0x020a4000, 0x020a8000, 0x020ac000 };
static const int clocks[] = { pctl_clk_gpio1, pctl_clk_gpio2, pctl_clk_gpio3, pctl_clk_gpio4, pctl_clk_gpio5 };


int init(void)
{
	int i, err;
	char devpath[16];
	platformctl_t pctl;

	for (i = 0; i < sizeof(common.gpio) / sizeof(common.gpio[0]); ++i) {
		if ((common.gpio[i].base = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_UNCACHED, FD_PHYSMEM, paddr[i])) == MAP_FAILED) {
			printf("gpiodrv: Could not map gpio%d paddr %p\n", i + 1, (void*) paddr[i]);
			return -1;
		}
	}

	if (portEvent(3, 0, 0) < 0) {
		if ((common.port = portCreate(0)) < 0) {
			printf("gpiodrv: Could not create port\n");
			return -1;
		}
	}
	else {
		common.port = 3;
	}

	for (i = 0; i < sizeof(common.gpio) / sizeof(common.gpio[0]); ++i) {
		sprintf(devpath, "/dev/gpio%d", i + 1);
		mkdir(devpath, 0750);
	
		sprintf(devpath, "/dev/gpio%d/port", i + 1);

		if ((err = create_dev(common.port, gpio1 + i, devpath, S_IFCHR)) != EOK) {
			printf("gpiodrv: Could not create port file #%d (err %d)\n", i + 1, err);
			return - 1;
		}

		common.gpio[i].port = gpio1 + i;

		sprintf(devpath, "/dev/gpio%d/dir", i + 1);

		if ((err = create_dev(common.port, dir1 + i, devpath, S_IFCHR)) != EOK) {
			printf("gpiodrv: Could not create direction file #%d (err %d)\n", i + 1, err);
			return - 1;
		}

		common.gpio[i].dir = dir1 + i;

		pctl.action = pctl_set;
		pctl.type = pctl_devclock;
		pctl.devclock.state = 3;
		pctl.devclock.dev = clocks[i];

		if (platformctl(&pctl) != EOK) {
			printf("gpiodrv: Could not enable clock for gpio%d\n", i + 1);
			return -1;
		}

		if (mutexCreate(&common.gpio[i].lock) < 0) {
			printf("gpiodrv: Could not create mutex for gpio%d\n", i + 1);
			return -1;
		}
	}

	return 0;
}


int gpioread(int d, uint32_t *val)
{
	if (d < gpio1 || d > gpio5)
		return -EINVAL;

	mutexLock(common.gpio[d - gpio1].lock);
	(*val) = *(common.gpio[d - gpio1].base + dr);
	mutexUnlock(common.gpio[d - gpio1].lock);

	return EOK;
}


int gpiowrite(int d, uint32_t val, uint32_t mask)
{
	uint32_t t;

	if (d < gpio1 || d > gpio5)
		return -EINVAL;

	mutexLock(common.gpio[d - gpio1].lock);
	t = *(common.gpio[d - gpio1].base + dr) & ~(mask);
	*(common.gpio[d - gpio1].base + dr) = t | (val & mask);
	mutexUnlock(common.gpio[d - gpio1].lock);

	return EOK;
}


int gpiogetdir(int d, uint32_t *dir)
{
	if (d < dir1 || d > dir5)
		return -EINVAL;

	mutexLock(common.gpio[d - dir1].lock);
	*(dir) = *(common.gpio[d - dir1].base + gdir);
	mutexUnlock(common.gpio[d - dir1].lock);

	return EOK;
}


int gpiosetdir(int d, uint32_t dir, uint32_t mask)
{
	uint32_t t;

	if (d < dir1 || d > dir5)
		return -EINVAL;

	mutexLock(common.gpio[d - dir1].lock);
	t = *(common.gpio[d - dir1].base + gdir) & ~(mask);
	*(common.gpio[d - dir1].base + gdir) = t | (dir & mask);
	mutexUnlock(common.gpio[d - dir1].lock);

	return EOK;
}


void thread(void *arg)
{
	msg_t msg;
	unsigned int rid;
	int d;
	uint32_t val, mask;
	int error;

	while (1) {
		if (msgRecv(common.port, &msg, &rid) < 0)
			continue;

		error = -EINVAL;

		switch (msg.type) {
			case mtOpen:
				msg.o.open = msg.object;
				/* fallthrough */
			case mtClose:
				error = msg.object < gpio1 || msg.object > dir5 ? -ENXIO : EOK;
				break;

			case mtRead:
				if (msg.o.data != NULL && msg.o.size >= sizeof(uint32_t)) {
					d = msg.object;

					if (d >= gpio1 && d <= gpio5) {
						error = gpioread(d, &val);
						memcpy(msg.o.data, &val, sizeof(uint32_t));
					}
					else if (d >= dir1 && d <= dir5) {
						error = gpiogetdir(d, &val);
						memcpy(msg.o.data, &val, sizeof(uint32_t));
					}

					if (error == EOK)
						msg.o.io = sizeof(uint32_t);
					else
						msg.o.io = 0;
				}
				break;

			case mtWrite:
				if (msg.i.data != NULL && msg.i.size >= sizeof(uint32_t)) {
					memcpy(&val, msg.i.data, sizeof(uint32_t));

					if (msg.i.size >= sizeof(uint32_t) << 1)
						memcpy(&mask, msg.i.data + sizeof(uint32_t), sizeof(uint32_t));
					else
						mask = (uint32_t)-1;

					d = msg.object;

					if (d >= gpio1 && d <= gpio5)
						error = gpiowrite(d, val, mask);
					else if (d >= dir1 && d <= dir5)
						error = gpiosetdir(d, val, mask);

					if (error == EOK)
						msg.o.io = (msg.i.size >= (sizeof(uint32_t) << 1)) ? sizeof(uint32_t) << 1 : sizeof(uint32_t);
					else
						msg.o.io = 0;
				}
				break;
		}

		msgRespond(common.port, error, &msg, rid);
	}
}


int main(void)
{
	oid_t root;

	if (init())
		return -EIO;

	if (fork())
		_exit(0);
	setsid();

	thread(NULL);

	return 0;
}
