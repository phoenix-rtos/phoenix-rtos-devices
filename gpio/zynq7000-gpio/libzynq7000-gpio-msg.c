/*
 * Phoenix-RTOS
 *
 * Zynq-7000 GPIO message interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>

#include "zynq7000-gpio-msg.h"


int gpiomsg_readPin(oid_t *pin, uint32_t *val)
{
	msg_t msg = { 0 };
	gpio_devctl_t *idevctl = (gpio_devctl_t *)msg.i.raw;
	gpio_devctl_t *odevctl = (gpio_devctl_t *)msg.o.raw;
	int err;

	if ((pin == NULL) || (val == NULL)) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	msg.oid = *pin;
	idevctl->i.type = gpio_devctl_read_pin;

	err = msgSend(pin->port, &msg);
	if (err < 0) {
		return err;
	}
	*val = odevctl->o.val;

	return msg.o.err;
}


int gpiomsg_writePin(oid_t *pin, uint32_t val)
{
	msg_t msg = { 0 };
	gpio_devctl_t *idevctl = (gpio_devctl_t *)msg.i.raw;
	int err;

	if (pin == NULL) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	idevctl->i.type = gpio_devctl_write_pin;
	msg.oid = *pin;
	idevctl->i.val = val;

	err = msgSend(pin->port, &msg);
	if (err < 0) {
		return err;
	}

	return msg.o.err;
}


int gpiomsg_readPort(oid_t *port, uint32_t *val)
{
	msg_t msg = { 0 };
	gpio_devctl_t *idevctl = (gpio_devctl_t *)msg.i.raw;
	gpio_devctl_t *odevctl = (gpio_devctl_t *)msg.o.raw;
	int err;

	if ((port == NULL) || (val == NULL)) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	idevctl->i.type = gpio_devctl_read_port;
	msg.oid = *port;

	err = msgSend(port->port, &msg);
	if (err < 0) {
		return err;
	}
	*val = odevctl->o.val;

	return msg.o.err;
}


int gpiomsg_writePort(oid_t *port, uint32_t val, uint32_t mask)
{
	msg_t msg = { 0 };
	gpio_devctl_t *idevctl = (gpio_devctl_t *)msg.i.raw;
	int err;

	if (port == NULL) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	idevctl->i.type = gpio_devctl_write_port;
	msg.oid = *port;
	idevctl->i.val = val;
	idevctl->i.mask = mask;

	err = msgSend(port->port, &msg);
	if (err < 0) {
		return err;
	}

	return msg.o.err;
}


int gpiomsg_readDir(oid_t *dir, uint32_t *val)
{
	msg_t msg = { 0 };
	gpio_devctl_t *idevctl = (gpio_devctl_t *)msg.i.raw;
	gpio_devctl_t *odevctl = (gpio_devctl_t *)msg.o.raw;
	int err;

	if ((dir == NULL) || (val == NULL)) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	idevctl->i.type = gpio_devctl_read_dir;
	msg.oid = *dir;

	err = msgSend(dir->port, &msg);
	if (err < 0) {
		return err;
	}
	*val = odevctl->o.val;

	return msg.o.err;
}


int gpiomsg_writeDir(oid_t *dir, uint32_t val, uint32_t mask)
{
	msg_t msg = { 0 };
	gpio_devctl_t *idevctl = (gpio_devctl_t *)msg.i.raw;
	int err;

	if (dir == NULL) {
		return -EINVAL;
	}

	msg.type = mtDevCtl;
	idevctl->i.type = gpio_devctl_write_dir;
	msg.oid = *dir;
	idevctl->i.val = val;
	idevctl->i.mask = mask;

	err = msgSend(dir->port, &msg);
	if (err < 0) {
		return err;
	}

	return msg.o.err;
}
