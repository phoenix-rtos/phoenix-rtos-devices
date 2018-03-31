/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * iMXRT UART driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef UARTDRV_H_
#define UARTDRV_H_


enum { UARTDRV_DEF = 0 };


enum { UARTDRV_PARNONE = 0, UARTDRV_PAREVEN, UARTDRV_PARODD };


typedef struct {
	unsigned int off;
	char buff[];
} __attribute__((packed)) uartdrv_data_t;


typedef struct {
	char type;

	union {
		struct {
			char enable;
			char bits;
			char parity;
			unsigned int baud;
		} def;

		struct {
			char mode;
			unsigned int timeout;
		} get;

		struct {
			int state;
		} enable;
	};
} __attribute__((packed)) uartdrv_devctl_t;


extern void uartdrv_init(void);


#endif
