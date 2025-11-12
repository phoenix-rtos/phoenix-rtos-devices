/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 I2C driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef LIBI2C_H_
#define LIBI2C_H_

#include <stdint.h>
#include <sys/types.h>


typedef struct {
	volatile unsigned int *base;
	volatile int err;
	int clk;
	uint32_t refclk_freq;

	handle_t irqlock;
	handle_t irqcond;
} libi2c_ctx_t;


enum { i2c1 = 0, i2c2, i2c3, i2c4 };


enum libi2c_speed {
	libi2c_speed_standard = 0, /* Standard mode, 100 kHz */
	libi2c_speed_fast,         /* Fast mode, 400 kHz */
	libi2c_speed_fastplus,     /* Fast Plus mode, 1 MHz */
};


ssize_t libi2c_read(libi2c_ctx_t *ctx, unsigned char addr, void *buff, size_t len);


ssize_t libi2c_readReg(libi2c_ctx_t *ctx, unsigned char addr, unsigned char reg, void *buff, size_t len);


ssize_t libi2c_write(libi2c_ctx_t *ctx, unsigned char addr, const void *buff, size_t len);


ssize_t libi2c_writeReg(libi2c_ctx_t *ctx, unsigned char addr, unsigned char reg, const void *buff, size_t len);


int libi2c_setSpeed(libi2c_ctx_t *ctx, enum libi2c_speed speed, int rise_time);


int libi2c_init(libi2c_ctx_t *ctx, int i2c);


#endif
