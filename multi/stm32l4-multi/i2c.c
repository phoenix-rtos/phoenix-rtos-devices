/*
 * Phoenix-RTOS
 *
 * STM32L4 I2C driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/pwman.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "libmulti/libi2c.h"

#include "stm32l4-multi.h"
#include "common.h"
#include "config.h"
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"

#define MAX_I2C i2c4

#define I2C1_POS 0
#define I2C2_POS (I2C1_POS + I2C1)
#define I2C3_POS (I2C2_POS + I2C2)
#define I2C4_POS (I2C3_POS + I2C3)

#define N_I2C_ACTIVE (I2C1 + I2C2 + I2C3 + I2C4)

libi2c_ctx_t i2c_ctx[N_I2C_ACTIVE];
handle_t i2c_lock[N_I2C_ACTIVE];


static const int i2cConfig[MAX_I2C + 1] = { I2C1, I2C2, I2C3, I2C4 };


static const int i2cPos[MAX_I2C + 1] = { I2C1_POS, I2C2_POS, I2C3_POS, I2C4_POS };


ssize_t i2c_read(int i2c, unsigned char addr, void *buff, size_t len)
{
	if ((N_I2C_ACTIVE == 0) || (i2c < i2c1) || (i2c > MAX_I2C) || (i2cConfig[i2c] == 0)) {
		return -1;
	}

	mutexLock(i2c_lock[i2cPos[i2c]]);
	ssize_t ret = libi2c_read(&i2c_ctx[i2cPos[i2c]], addr, buff, len);
	mutexUnlock(i2c_lock[i2cPos[i2c]]);

	return ret;
}


ssize_t i2c_readReg(int i2c, unsigned char addr, unsigned char reg, void *buff, size_t len)
{
	if ((N_I2C_ACTIVE == 0) || (i2c < i2c1) || (i2c > MAX_I2C) || (i2cConfig[i2c] == 0)) {
		return -EINVAL;
	}

	mutexLock(i2c_lock[i2cPos[i2c]]);
	ssize_t ret = libi2c_readReg(&i2c_ctx[i2cPos[i2c]], addr, reg, buff, len);
	mutexUnlock(i2c_lock[i2cPos[i2c]]);

	return ret;
}


ssize_t i2c_write(int i2c, unsigned char addr, const void *buff, size_t len)
{
	if ((N_I2C_ACTIVE == 0) || (i2c < i2c1) || (i2c > MAX_I2C) || (i2cConfig[i2c] == 0)) {
		return -EINVAL;
	}

	mutexLock(i2c_lock[i2cPos[i2c]]);
	ssize_t ret = libi2c_write(&i2c_ctx[i2cPos[i2c]], addr, buff, len);
	mutexUnlock(i2c_lock[i2cPos[i2c]]);

	return ret;
}


ssize_t i2c_writeReg(int i2c, unsigned char addr, unsigned char reg, const void *buff, size_t len)
{
	if ((N_I2C_ACTIVE == 0) || (i2c < i2c1) || (i2c > MAX_I2C) || (i2cConfig[i2c] == 0)) {
		return -EINVAL;
	}

	mutexLock(i2c_lock[i2cPos[i2c]]);
	ssize_t ret = libi2c_writeReg(&i2c_ctx[i2cPos[i2c]], addr, reg, buff, len);
	mutexUnlock(i2c_lock[i2cPos[i2c]]);

	return ret;
}


int i2c_setSpeed(int i2c, uint32_t speed, int rise_time)
{
	if ((N_I2C_ACTIVE == 0) || (i2c < i2c1) || (i2c > MAX_I2C) || (i2cConfig[i2c] == 0)) {
		return -EINVAL;
	}

	mutexLock(i2c_lock[i2cPos[i2c]]);
	int ret = libi2c_setSpeed(&i2c_ctx[i2cPos[i2c]], speed, rise_time);
	mutexUnlock(i2c_lock[i2cPos[i2c]]);

	return ret;
}


void i2c_init(void)
{
	int i2c;
	if (N_I2C_ACTIVE == 0) {
		return;
	}

	for (i2c = i2c1; i2c <= MAX_I2C; ++i2c) {
		if (!i2cConfig[i2c]) {
			continue;
		}

		mutexCreate(&i2c_lock[i2cPos[i2c]]);
		libi2c_init(&i2c_ctx[i2cPos[i2c]], i2c);
	}
}
