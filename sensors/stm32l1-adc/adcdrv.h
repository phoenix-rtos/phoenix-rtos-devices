/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 ADC driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _ADCDRV_H_
#define _ADCDRV_H_


enum { ADCDRV_DEF = 0, ADCDRV_GET, ADCDRV_SET };


typedef struct {
	unsigned int off;
	char buff[];
} __attribute__((packed)) adcdrv_data_t;


typedef struct {
	char type;
	char channel;
} __attribute__((packed)) adcdrv_devctl_t;


#endif
