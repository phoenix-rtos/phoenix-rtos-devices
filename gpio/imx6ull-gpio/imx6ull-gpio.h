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

typedef struct {
	union {
		unsigned int val;
		struct {
			unsigned int val;
			unsigned int mask;
		} w;
	};
} __attribute__((packed)) gpiodata_t;

#endif
