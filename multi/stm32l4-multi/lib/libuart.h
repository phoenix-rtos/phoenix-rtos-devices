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

#ifndef _LIBUART_H_
#define _LIBUART_H_

#include <sys/threads.h>
#include <libtty.h>

typedef struct {
	char stack[256] __attribute__ ((aligned(8)));

	volatile unsigned int *base;
	volatile int enabled;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	handle_t irqlock;

	libtty_common_t tty_common;

	volatile unsigned char rxbuff;
	volatile int rxready;
} libuart_ctx;


enum { usart1 = 0, usart2, usart3, uart4, uart5 };


ssize_t libuart_write(libuart_ctx *ctx, const void *buff, size_t bufflen, unsigned int mode);


ssize_t libuart_read(libuart_ctx *ctx, void *buff, size_t bufflen, unsigned int mode);


int libuart_getAttr(libuart_ctx *ctx, int type);


int libuart_devCtl(libuart_ctx *ctx, pid_t pid, unsigned int request, const void* inData, const void** outData);


int libuart_init(libuart_ctx *ctx, unsigned int uart);


#endif
