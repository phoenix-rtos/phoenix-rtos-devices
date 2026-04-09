/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO message interface
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <unistd.h>

#include <libgrgpio.h>
#include "grgpio-msg.h"


int gpiomsg_readPort(const oid_t *oid, uint32_t *val)
{
	if ((oid == NULL) || (val == NULL)) {
		return -EINVAL;
	}

	msg_t msg = { 0 };
	gpio_i_t *idevctl = (gpio_i_t *)msg.i.raw;
	gpio_o_t *odevctl = (gpio_o_t *)msg.o.raw;

	msg.type = mtDevCtl;
	msg.oid = *oid;
	idevctl->type = gpio_getPort;

	int err = msgSend(oid->port, &msg);
	if (err < 0) {
		return err;
	}
	*val = odevctl->val;

	return msg.o.err;
}


int gpiomsg_writePort(const oid_t *oid, uint32_t val, uint32_t mask)
{
	if (oid == NULL) {
		return -EINVAL;
	}

	msg_t msg = { 0 };
	gpio_i_t *idevctl = (gpio_i_t *)msg.i.raw;

	msg.type = mtDevCtl;
	msg.oid = *oid;
	idevctl->type = gpio_setPort;
	idevctl->val = val;
	idevctl->mask = mask;

	int err = msgSend(oid->port, &msg);
	if (err < 0) {
		return err;
	}

	return msg.o.err;
}


int gpiomsg_readDir(const oid_t *oid, uint32_t *val)
{
	if ((oid == NULL) || (val == NULL)) {
		return -EINVAL;
	}

	msg_t msg = { 0 };
	gpio_i_t *idevctl = (gpio_i_t *)msg.i.raw;
	gpio_o_t *odevctl = (gpio_o_t *)msg.o.raw;

	msg.type = mtDevCtl;
	msg.oid = *oid;
	idevctl->type = gpio_getDir;

	int err = msgSend(oid->port, &msg);
	if (err < 0) {
		return err;
	}
	*val = odevctl->val;

	return msg.o.err;
}


int gpiomsg_writeDir(const oid_t *oid, uint32_t val, uint32_t mask)
{
	if (oid == NULL) {
		return -EINVAL;
	}

	msg_t msg = { 0 };
	gpio_i_t *idevctl = (gpio_i_t *)msg.i.raw;

	msg.type = mtDevCtl;
	msg.oid = *oid;
	idevctl->type = gpio_setDir;
	idevctl->val = val;
	idevctl->mask = mask;

	int err = msgSend(oid->port, &msg);
	if (err < 0) {
		return err;
	}

	return msg.o.err;
}


int gpiomsg_readPin(const oid_t *oid, uint32_t pin, uint32_t *val)
{
	if ((oid == NULL) || (val == NULL) || (pin > 31U)) {
		return -EINVAL;
	}

	uint32_t portVal;

	int err = gpiomsg_readPort(oid, &portVal);
	if (err < 0) {
		return err;
	}

	*val = (portVal >> pin) & 1U;

	return 0;
}


int gpiomsg_writePin(const oid_t *oid, uint32_t pin, uint32_t val)
{
	if ((oid == NULL) || (pin > 31U)) {
		return -EINVAL;
	}

	return gpiomsg_writePort(oid, (val != 0U) ? (1U << pin) : 0U, 1U << pin);
}


int gpiomsg_readPinDir(const oid_t *oid, uint32_t pin, uint32_t *val)
{
	if ((oid == NULL) || (val == NULL) || (pin > 31U)) {
		return -EINVAL;
	}

	uint32_t dirVal;

	int err = gpiomsg_readDir(oid, &dirVal);
	if (err < 0) {
		return err;
	}

	*val = (dirVal >> pin) & 1U;

	return 0;
}


int gpiomsg_writePinDir(const oid_t *oid, uint32_t pin, uint32_t val)
{
	if ((oid == NULL) || (pin > 31U)) {
		return -EINVAL;
	}

	return gpiomsg_writeDir(oid, (val != 0U) ? (1U << pin) : 0U, 1U << pin);
}


int gpiomsg_open(uint32_t port, oid_t *oid)
{
	unsigned int ntries = 10;
	char devName[16];

	int err = snprintf(devName, sizeof(devName), "/dev/gpio%" PRIu32, port);
	if (err >= sizeof(devName)) {
		return -EINVAL;
	}

	while (lookup(devName, NULL, oid) < 0) {
		if (--ntries == 0) {
			return -ENOENT;
		}
		usleep(10 * 1000);
	}

	return 0;
}
