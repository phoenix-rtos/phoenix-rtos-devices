/*
 * Phoenix-RTOS
 *
 * i.MX RT multidriver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMXRT_MULTI_H_
#define _IMXRT_MULTI_H_


enum { uart_def = 0, uart_get, uart_set };


/* UART */


enum { lpuart1 = 0, lpuart2, lpuart3, lpuart4, lpuart5, lpuart6, lpuart7, lpuart8 };


enum { uart_mnormal = 0, uart_mnblock };


enum { uart_parnone = 0, uart_pareven, uart_parodd };


typedef struct {
	int uart;
	int mode;
	unsigned int timeout;
} __attribute__((packed)) uartget_t;


typedef struct {
	int uart;
} __attribute__((packed)) uartset_t;


typedef struct {
	int uart;
	unsigned int baud;
	char enable;
	char bits;
	char parity;
} __attribute__((packed)) uartdef_t;


/* MULTI */


typedef struct {
	int type;

	union {
		uartget_t uart_get;
		uartset_t uart_set;
		uartdef_t uart_def;
	};
} __attribute__((packed)) multi_i_t;


typedef struct {
	int err;
} __attribute__((packed)) multi_o_t;


#endif
