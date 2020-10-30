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


#ifndef I2C_H_
#define I2C_H_


#include <sys/types.h>


ssize_t i2c_read(int i2c, unsigned char addr, void *buff, size_t len);


ssize_t i2c_readReg(int i2c, unsigned char addr, unsigned char reg, void *buff, size_t len);


ssize_t i2c_write(int i2c, unsigned char addr, void *buff, size_t len);


ssize_t i2c_writeReg(int i2c, unsigned char addr, unsigned char reg, void *buff, size_t len);


void i2c_init(void);


#endif
