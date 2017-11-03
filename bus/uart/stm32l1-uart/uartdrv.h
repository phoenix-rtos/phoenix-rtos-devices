/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 UART driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski, Adrian Kepka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef UARTDRV_H_
#define UARTDRV_H_


enum { UART1_BIT = 1 << 0,
       UART2_BIT = 1 << 1,
       UART3_BIT = 1 << 2,
       UART4_BIT = 1 << 3,
       UART5_BIT = 1 << 4 };


enum { UARTDRV_DEF = 0, UARTDRV_GET, UARTDRV_ENABLE };


enum { UARTDRV_MNORMAL = 0, UARTDRV_MNBLOCK };


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


extern int uartdrv_enable(char *uart, int state);


extern void uartdrv_init(unsigned uarts);


#endif
