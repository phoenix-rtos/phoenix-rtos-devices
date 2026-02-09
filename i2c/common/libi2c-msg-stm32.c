/*
 * Phoenix-RTOS
 *
 * I2C message interface (stm32-multi flavor)
 *
 * Copyright 2026 Phoenix Systems
 * Author: Radoslaw Szewczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <i2c.h>
#include <i2c-msg.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stm32l4-multi.h>


static struct {
	oid_t oid; /* I2C slave oid, initialized by spimsg_open() */
	int dev;   /* I2C device number */
} common;


int i2c_init(unsigned int dev_no)
{
	unsigned int ntries = 10;
	while (lookup("/dev/multi", NULL, &common.oid) < 0) {
		if (--ntries == 0) {
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	common.dev = dev_no;

	return 0;
}


int i2c_busRead(uint8_t dev_addr, uint8_t *data, uint32_t len)
{
	msg_t msg = { 0 };
	int res;
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	msg.type = mtDevCtl;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = data;
	msg.o.size = len;

	i->type = i2c_get;
	i->i2c_msg.i2c = common.dev;
	i->i2c_msg.addr = dev_addr;

	if ((res = msgSend(common.oid.port, &msg)) < 0)
		return res;

	return msg.o.err;
}


int i2c_regRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
{
	msg_t msg;
	int res;
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	msg.type = mtDevCtl;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = data_out;
	msg.o.size = len;

	i->type = i2c_getwreg;
	i->i2c_msg.i2c = common.dev;
	i->i2c_msg.addr = dev_addr;
	i->i2c_msg.reg = reg_addr;

	if ((res = msgSend(common.oid.port, &msg)) < 0)
		return res;

	return msg.o.err;
}


int i2c_busWrite(uint8_t dev_addr, const uint8_t *data, uint32_t len)
{
	msg_t msg = { 0 };
	int res;
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	msg.type = mtDevCtl;

	msg.i.data = data;
	msg.i.size = len;
	msg.o.data = NULL;
	msg.o.size = 0;

	i->type = i2c_set;
	i->i2c_msg.i2c = common.dev;
	i->i2c_msg.addr = dev_addr;

	if ((res = msgSend(common.oid.port, &msg)) < 0)
		return res;

	return msg.o.err;
}


int i2c_regWrite(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data_out, uint32_t len)
{
	msg_t msg;
	int res;
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	msg.type = mtDevCtl;

	msg.i.data = data_out;
	msg.i.size = len;
	msg.o.data = NULL;
	msg.o.size = 0;

	i->type = i2c_setwreg;
	i->i2c_msg.i2c = common.oid.id;
	i->i2c_msg.addr = dev_addr;
	i->i2c_msg.reg = reg_addr;

	if ((res = msgSend(common.oid.port, &msg)) < 0)
		return res;

	return msg.o.err;
}
