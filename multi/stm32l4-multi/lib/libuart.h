/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 UART driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef LIBUART_H_
#define LIBUART_H_

#include <sys/threads.h>
#include <libtty.h>

typedef struct {
	char stack[256] __attribute__ ((aligned(8)));

	volatile unsigned int *base;
	volatile int enabled;

	int bits;
	int parity;
	int baud;

	handle_t cond;
	handle_t inth;
	handle_t irqlock;

	libtty_common_t tty_common;

	volatile unsigned char rxbuff;
	volatile int rxready;
} libuart_ctx_t;


enum { usart1 = 0, usart2, usart3, uart4, uart5 };


ssize_t libuart_write(libuart_ctx_t *ctx, const void *buff, size_t bufflen, unsigned int mode);


ssize_t libuart_read(libuart_ctx_t *ctx, void *buff, size_t bufflen, unsigned int mode);


int libuart_getAttr(libuart_ctx_t *ctx, int type);


int libuart_devCtl(libuart_ctx_t *ctx, pid_t pid, unsigned int request, const void* inData, const void** outData);


int libuart_init(libuart_ctx_t *ctx, unsigned int uart);


#endif
