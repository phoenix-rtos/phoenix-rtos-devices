/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 I2C driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _I2CDRV_H_
#define _I2CDRV_H_


enum { I2CDRV_DEF = 0, I2CDRV_GET, I2CDRV_SET };


typedef struct {
	unsigned int off;
	char buff[];
} __attribute__((packed)) i2cdrv_data_t;


typedef struct {
	char type;
	char addr;
	char reg;
	char buff[];
} __attribute__((packed)) i2cdrv_devctl_t;


/* Temporary solution */
extern unsigned int i2cdrv_id;


extern void i2cdrv_init(void);


#endif
