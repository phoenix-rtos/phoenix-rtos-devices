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

#include "libmulti/libdma.h"

typedef struct {
	volatile unsigned int *base;
	unsigned int port;
	unsigned int baud;
	volatile int enabled;
	int bits;

	enum {
		uart_dma,
		uart_irq,
	} type;
	union {
		struct {
			volatile char *volatile txbeg;
			volatile char *volatile txend;

			char *volatile rxdfifo;
			size_t rxdfifosz;
			volatile unsigned int rxnblock; /* set to 1 to read imdiately available bytes only */
			volatile unsigned int rxdr;
			volatile unsigned int rxdw;
			volatile char *volatile rxbeg;
			volatile char *volatile rxend;
			volatile unsigned int *volatile read;

			handle_t rxlock;
			handle_t rxcond;
			handle_t txlock;
			handle_t txcond;
			handle_t lock;
		} irq;
		struct {
			handle_t txlock;
			handle_t rxlock;
			handle_t rxcond;
			handle_t rxcondlock;
			const struct libdma_per *per;

			volatile char *rxbuf;
			volatile size_t rxbufsz;
			volatile size_t read;
			volatile size_t rxfifopos;
			volatile size_t rxfifoprevend;
			volatile int rxfifofull;
			char *rxfifo;
			size_t rxfifosz;
		} dma;
	} data;
} libuart_ctx;


/* clang-format off */
enum { usart1 = 0, usart2, usart3, uart4, uart5 };


enum { uart_mnormal = 0, uart_mnblock };


enum { uart_parnone = 0, uart_pareven, uart_parodd };
/* clang-format on */


int libuart_configure(libuart_ctx *ctx, char bits, char parity, unsigned int baud, char enable);


int libuart_write(libuart_ctx *ctx, const void *buff, unsigned int bufflen);


/* timeout in milliseconds - zero disables timeout */
int libuart_read(libuart_ctx *ctx, void *buff, unsigned int count, char mode, unsigned int timeout);


int libuart_init(libuart_ctx *ctx, unsigned int uart, int dma);


#endif
