/*
 * Phoenix-RTOS
 *
 * i2c msg client library
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <sys/types.h>

#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include <i2c.h>
#include <i2c-msg.h>

static struct {
	oid_t i2c_oid;
	int initialized;
} common;

/* opens the device */
int i2c_init(unsigned int dev_no)
{
	char devname[] = "/dev/i2cX";
	int timeout = 10; /* x 10 ms = 100 ms */

	devname[sizeof(devname) - 2] = '0' + (dev_no % 10);
	while (lookup(devname, NULL, &common.i2c_oid) < 0) {
		if (timeout-- <= 0)
			return -ETIMEDOUT;

		usleep(10 * 1000);
	}

	common.initialized = 1;
	return 0;
}


/* Performs i2c generic write operation to the given slave device. */
int i2c_busWrite(uint8_t dev_addr, const uint8_t *data, uint32_t len)
{
	msg_t msg;
	int res;
	i2c_devctl_t *in = (i2c_devctl_t *)msg.i.raw;
	i2c_devctl_t *out = (i2c_devctl_t *)msg.o.raw;

	if (!common.initialized)
		return -EIO;

	msg.type = mtDevCtl;

	msg.i.data = (uint8_t *)data; /* FIXME: dropping const because of broken msg_t declaration */
	msg.i.size = len;
	msg.o.data = NULL;
	msg.o.size = 0;

	in->i.type = i2c_devctl_bus_write;
	in->i.dev_addr = dev_addr;

	if ((res = msgSend(common.i2c_oid.port, &msg)) < 0)
		return res;

	return out->o.err;
}


/* Performs i2c generic read operation from the given slave device. */
int i2c_busRead(uint8_t dev_addr, uint8_t *data_out, uint32_t len)
{
	msg_t msg;
	int res;
	i2c_devctl_t *in = (i2c_devctl_t *)msg.i.raw;
	i2c_devctl_t *out = (i2c_devctl_t *)msg.o.raw;

	if (!common.initialized)
		return -EIO;

	msg.type = mtDevCtl;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = data_out;
	msg.o.size = len;

	in->i.type = i2c_devctl_bus_read;
	in->i.dev_addr = dev_addr;

	if ((res = msgSend(common.i2c_oid.port, &msg)) < 0)
		return res;

	return out->o.err;
}


/* Performs i2c regiester read operation from the given slave device */
int i2c_regRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
{
	msg_t msg;
	int res;
	i2c_devctl_t *in = (i2c_devctl_t *)msg.i.raw;
	i2c_devctl_t *out = (i2c_devctl_t *)msg.o.raw;

	if (!common.initialized)
		return -EIO;

	msg.type = mtDevCtl;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = data_out;
	msg.o.size = len;

	in->i.type = i2c_devctl_reg_read;
	in->i.dev_addr = dev_addr;
	in->i.reg_addr = reg_addr;

	if ((res = msgSend(common.i2c_oid.port, &msg)) < 0)
		return res;

	return out->o.err;
}
