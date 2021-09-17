/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL watchdog driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Jakub Sarzyński
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <phoenix/arch/imx6ull.h>
#include <posix/utils.h>

#include "watchdog.h"

const size_t wdog_addr = 0x20bc000; /* WDOG1 address */
const int default_timeout = 60;

struct {
	volatile uint16_t *base;
	uint32_t port;
} wdog;

enum { wcr = 0, wsr, wrsr, wicr, wmcr };


static void wdog_kick(void)
{
	/* Write magic sequence */
	*(wdog.base + wsr) = 0x5555;
	*(wdog.base + wsr) = 0xAAAA;
}

static void wdog_setup(void)
{
	uint16_t val = *(wdog.base + wcr);

	/* Clear timeout value */
	val &= ~0xff00;
	/* Set default timeout */
	/* 0x00 - 0.5 sec, 0x01 - 1 sec and so on... */
	val |= (default_timeout * 2 - 1) << 8;

	/* Enable watchdog, enable suspend in Debug mode */
	val |= (1 << 2) | (1 << 1);
	*(wdog.base + wcr) = val;
}

static int wdog_settimeout(int timeout)
{
	uint16_t val;

	val = *(wdog.base + wcr);
	/* Clear timeout value */
	val &= ~0xff00;
	/* Set new timeout */
	/* 0x00 - 0.5 sec, 0x01 - 1 sec and so on... */
	val |= (timeout * 2 - 1) << 8;
	*(wdog.base + wcr) = val;

	/* new timeout will be used only after kicking the watchdog */
	wdog_kick();
	return timeout;
}

static void op_ioctl(msg_t *msg)
{
	const int *data;
	unsigned long request;
	int timeout = 0, err = EOK;
	id_t id;

	data = ioctl_unpack(msg, &request, &id);

	switch (request) {
		case WDIOC_KEEPALIVE:
			wdog_kick();
			break;
		case WDIOC_SETTIMEOUT:
			if (!data) {
				err = -EINVAL;
				break;
			}

			timeout = *data;

			if (timeout <= 0) {
				err = -EINVAL;
				break;
			}

			/* Max timeout */
			if (timeout >= 128)
				timeout = 128;

			timeout = wdog_settimeout(timeout);
			data = &timeout;
			break;
	}

	ioctl_setResponse(msg, request, err, data);
}


void wdog_handle(void)
{
	msg_t msg;
	unsigned long int rid;

	while (1) {
		if (msgRecv(wdog.port, &msg, &rid) < 0)
			continue;

		msg.o.io.err = -ENOSYS;

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;
			case mtWrite:
				wdog_kick();
				msg.o.io.err = msg.i.size;
				break;
			case mtDevCtl:
				op_ioctl(&msg);
				break;
		}

		msgRespond(wdog.port, &msg, rid);
	}
}


int wdog_init(void)
{
	int err;
	oid_t dev;
	platformctl_t pctl;

	wdog.base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, wdog_addr);
	if (wdog.base == MAP_FAILED) {
		puts("watchdog: mmap failed");
		return -1;
	}

	err = portCreate(&wdog.port);
	if (err) {
		puts("watchdog: could not create port");
		return -1;
	}

	dev.port = wdog.port;
	dev.id = 0;

	err = create_dev(&dev, "watchdog");
	if (err) {
		puts("watchdog: could not create port file /dev/watchdog");
		return -1;
	}

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.state = 3;
	pctl.devclock.dev = pctl_clk_wdog1;

	err = platformctl(&pctl);
	if (err) {
		puts("watchdog: could not enable clock");
		return -1;
	}

	wdog_setup();

	return EOK;
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

	if (wdog_init())
		return -EIO;

	wdog_handle();

	return 0;
}
