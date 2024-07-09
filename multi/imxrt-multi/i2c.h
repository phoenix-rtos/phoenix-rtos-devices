/*
 * Phoenix-RTOS
 *
 * i.MX RT i2c driver
 *
 * Copyright 2019, 2024 Phoenix Systems
 * Author: Andrzej Glowinski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _I2C_H_
#define _I2C_H_

#include <sys/msg.h>


void i2c_handleMsg(msg_t *msg, int dev);


int i2c_init(void);


int multi_i2c_busWrite(unsigned bus, uint8_t dev_addr, const uint8_t *data, uint32_t len);


int multi_i2c_busRead(unsigned bus, uint8_t dev_addr, uint8_t *data_out, uint32_t len);


int multi_i2c_regRead(unsigned bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len);


#endif
