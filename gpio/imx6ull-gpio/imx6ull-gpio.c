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
#include <phoenix/arch/imx6ull.h>

#include "imx6ull-gpio.h"


enum { gpio1 = 0, gpio2, gpio3, gpio4, gpio5, dir1, dir2, dir3, dir4, dir5 };


enum { dr = 0, gdir, psr, icr1, icr2, imr, isr, edge };


struct {
	struct {
		volatile u32 *base;
		id_t port;
		id_t dir;
		handle_t lock;
	} gpio[5];

	u32 port;
} common;


static const addr_t paddr[] = { 0x0209c000, 0x020a0000, 0x020a4000, 0x020a8000, 0x020ac000 };
static const int clocks[] = { pctl_clk_gpio1, pctl_clk_gpio2, pctl_clk_gpio3, pctl_clk_gpio4, pctl_clk_gpio5 };


int init(void)
{
	int i, err;
	char dirname[11];
	platformctl_t pctl;
	oid_t dir;
	msg_t msg;

	for (i = 0; i < sizeof(common.gpio) / sizeof(common.gpio[0]); ++i) {
		if ((common.gpio[i].base = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_UNCACHED, OID_PHYSMEM, paddr[i])) == MAP_FAILED) {
			printf("gpiodrv: Could not map gpio%d paddr %p\n", i + 1, (void*) paddr[i]);
			return -1;
		}
	}

	if (portCreate(&common.port) != EOK) {
		printf("gpiodrv: Could not create port\n");
		return -1;
	}

	err = mkdir("/dev", 0);

	if (err < 0 && err != -EEXIST) {
		printf("gpiodrv: mkdir /dev failed\n");
		return -1;
	}

	strcpy(dirname, "/dev/gpiox");

	for (i = 0; i < sizeof(common.gpio) / sizeof(common.gpio[0]); ++i) {
		dirname[9] = '1' + i;

		err = mkdir(dirname, 0);

		if (err < 0 && err != -EEXIST) {
			printf("gpiodrv: Could not create %s\n", dirname);
			return -1;
		}

		if (lookup(dirname, NULL, &dir) < 0) {
			printf("gpiodrv: %s lookup failed\n", dirname);
			return -1;
		}

		msg.type = mtCreate;
		msg.i.create.dir = dir;
		msg.i.create.type = otDev;
		msg.i.create.mode = 0;
		msg.i.create.dev.port = common.port;
		msg.i.create.dev.id = gpio1 + i;
		msg.i.data = "port";
		msg.i.size = sizeof("port");
		msg.o.data = NULL;
		msg.o.size = 0;

		if (msgSend(dir.port, &msg) < 0 || msg.o.create.err != EOK) {
			printf("gpiodrv: Could not create port file #%d (err %d)\n", i + 1, msg.o.create.err);
			return - 1;
		}

		common.gpio[i].port = msg.o.create.oid.id;

		msg.type = mtCreate;
		msg.i.create.dir = dir;
		msg.i.create.type = otDev;
		msg.i.create.mode = 0;
		msg.i.create.dev.port = common.port;
		msg.i.create.dev.id = dir1 + i;
		msg.i.data = "dir";
		msg.i.size = sizeof("dir");
		msg.o.data = NULL;
		msg.o.size = 0;

		if (msgSend(dir.port, &msg) < 0 || msg.o.create.err != EOK) {
			printf("gpiodrv: Could not create direction file #%d\n", i + 1);
			return - 1;
		}

		common.gpio[i].dir = msg.o.create.oid.id;

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


int gpioread(int d, u32 *val)
{
	if (d < gpio1 || d > gpio5)
		return -EINVAL;

	mutexLock(common.gpio[d - gpio1].lock);
	(*val) = *(common.gpio[d - gpio1].base + dr);
	mutexUnlock(common.gpio[d - gpio1].lock);

	return EOK;
}


int gpiowrite(int d, u32 val, u32 mask)
{
	u32 t;

	if (d < gpio1 || d > gpio5)
		return -EINVAL;

	mutexLock(common.gpio[d - gpio1].lock);
	t = *(common.gpio[d - gpio1].base + dr) & ~(mask);
	*(common.gpio[d - gpio1].base + dr) = t | (val & mask);
	mutexUnlock(common.gpio[d - gpio1].lock);

	return EOK;
}


int gpiogetdir(int d, u32 *dir)
{
	if (d < dir1 || d > dir5)
		return -EINVAL;

	mutexLock(common.gpio[d - dir1].lock);
	*(dir) = *(common.gpio[d - dir1].base + gdir);
	mutexUnlock(common.gpio[d - dir1].lock);

	return EOK;
}


int gpiosetdir(int d, u32 dir, u32 mask)
{
	u32 t;

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
	u32 val, mask;

	while (1) {
		if (msgRecv(common.port, &msg, &rid) < 0)
			continue;

		msg.o.io.err = -EINVAL;

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = msg.i.openclose.oid.id < gpio1 || msg.i.openclose.oid.id > dir5 ? -ENOENT : EOK;
				break;

			case mtRead:
				if (msg.o.data != NULL && msg.o.size >= sizeof(u32)) {
					d = msg.i.io.oid.id;

					if (d >= gpio1 && d <= gpio5) {
						msg.o.io.err = gpioread(d, &val);
						memcpy(msg.o.data, &val, sizeof(u32));
					}
					else if (d >= dir1 && d <= dir5) {
						msg.o.io.err = gpiogetdir(d, &val);
						memcpy(msg.o.data, &val, sizeof(u32));
					}

					if (msg.o.io.err == EOK)
						msg.o.io.err = sizeof(u32);
				}
				break;

			case mtWrite:
				if (msg.i.data != NULL && msg.i.size >= sizeof(u32)) {
					memcpy(&val, msg.i.data, sizeof(u32));

					if (msg.i.size >= sizeof(u32) << 1)
						memcpy(&mask, msg.i.data + sizeof(u32), sizeof(u32));
					else
						mask = (u32)-1;

					d = msg.i.io.oid.id;

					if (d >= gpio1 && d <= gpio5)
						msg.o.io.err = gpiowrite(d, val, mask);
					else if (d >= dir1 && d <= dir5)
						msg.o.io.err = gpiosetdir(d, val, mask);

					if (msg.o.io.err == EOK)
						msg.o.io.err = (msg.i.size >= (sizeof(u32) << 1)) ? sizeof(u32) << 1 : sizeof(u32);
				}
				break;
		}

		msgRespond(common.port, &msg, rid);
	}
}


int main(void)
{
	oid_t root;

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
