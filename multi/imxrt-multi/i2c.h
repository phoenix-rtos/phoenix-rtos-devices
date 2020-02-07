/*
 * Phoenix-RTOS
 *
 * i.MX RT i2c driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _I2C_H_
#define _I2C_H_


int i2c_handleMsg(msg_t *msg, int dev);


int i2c_init(void);

#endif
