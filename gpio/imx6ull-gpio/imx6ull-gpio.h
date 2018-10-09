/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL GPIO driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GPIODRV_H_
#define _GPIODRV_H_

typedef union {
	unsigned int val;
	struct {
		unsigned int val;
		unsigned int mask;
	} __attribute__((packed)) w;
} gpiodata_t;

#endif
