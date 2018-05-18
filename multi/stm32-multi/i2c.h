/*
 * Phoenix-RTOS
 *
 * STM32L1 I2C driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _I2C_H_
#define _I2C_H_


extern unsigned int i2c_transaction(char op, char addr, char reg, void *buff, unsigned int count);


extern int i2c_init(void);


#endif
