/*
 * Phoenix-RTOS
 *
 * STM32L1 UART driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include ARCH
#include <errno.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "rcc.h"


typedef struct {
	volatile unsigned int *base;
	unsigned int port, baud;

	volatile char *txbeg, *txend;

	volatile char rxdfifo[32];
	volatile unsigned int rxdr;
	volatile unsigned int rxdw;
	volatile char *rxbeg, *rxend;
	volatile unsigned int read;

	handle_t cond;
	handle_t mutex;
	handle_t inth;
} uart_t;


enum { sr = 0, dr, brr, cr1, cr2, cr3, gtpr };


static int uart_irq(unsigned int n, void *arg)
{
	uart_t *uart = arg;
	handle_t release = 0;

	if ((*(uart->base + sr) & (1 << 7))) {
		/* Txd buffer empty */
		if (uart->txbeg != uart->txend) {
			*(uart->base + dr) = *(uart->txbeg++);
		}
		else {
			*(uart->base + cr1) &= ~(1 << 7);
			release = 1;
		}
	}

	if (*(uart->base + sr) & ((1 << 5) | (1 << 3))) {
		/* Rxd buffer not empty */
		uart->rxdfifo[uart->rxdw++] = *(uart->base + dr);
		uart->rxdw %= sizeof(uart->rxdfifo);

		if (uart->rxdr == uart->rxdw)
			uart->rxdr = (uart->rxdr + 1) % sizeof(uart->rxdfifo);
	}

	if (uart->rxbeg != NULL) {
		while (uart->rxdr != uart->rxdw && uart->rxbeg != uart->rxend) {
			*(uart->rxbeg++) = uart->rxdfifo[uart->rxdr++];
			uart->rxdr %= sizeof(uart->rxdfifo);
			uart->read++;
		}

		release = 1;
	}

	return release;
}


static int uart_write(void* buff, unsigned int bufflen, uart_t *uart)
{
	mutexLock(uart->mutex);
	keepidle(1);

	uart->txbeg = buff;
	uart->txend = buff + bufflen;

	*(uart->base + cr1) |= 1 << 7;

	while (uart->txbeg != uart->txend)
		condWait(uart->cond, uart->mutex, 0);

	uart->txbeg = NULL;
	uart->txend = NULL;

	keepidle(0);
	mutexUnlock(uart->mutex);

	return bufflen;
}


static int uart_read(void* buff, unsigned int count, uart_t *uart, char mode, unsigned int timeout)
{
	int i, err, read;

	mutexLock(uart->mutex);
	keepidle(1);

	uart->read = 0;
	uart->rxend = buff + count;
	uart->rxbeg = buff;

	/* Provoke UART exception to fire so that existing data from
	 * rxdfifo is copied into buff. The handler will clear this
	 * bit. */

	*(uart->base + cr1) |= 1 << 7;

	while (mode != UARTDRV_MNBLOCK && uart->rxbeg != uart->rxend) {
		err = condWait(uart->cond, uart->mutex, timeout);

		if (timeout && err == -ETIME)
			break;
	}

	uart->rxbeg = NULL;
	dataBarier();
	uart->rxend = NULL;

	read = uart->read;
	if (!(*(uart->base + cr1) & (1 << 12)) && (*(uart->base + cr1) & (1 << 10))) {
		for (i = 0; i < read; ++i)
			((char *)buff)[i] &= 0x7f;
	}

	keepidle(0);
	mutexUnlock(uart->mutex);

	return read;
}


static int uart_adjustBaud(uart_t *uart, int cpufreq)
{
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_cpuclk;
	platformctl(&pctl);

	if (cpufreq != pctl.cpuclk.hz) {
		/* Adjust to new clock frequency */

		cpufreq = pctl.cpuclk.hz;

		*(uart->base + cr1) &= ~(1 << 13);
		*(uart->base + brr) = cpufreq / uart->baud;
		*(uart->base + cr1) |= 1 << 13;
	}

	return cpufreq;
}


int uart_init(void)
{
	unsigned uarts = 0xb;
	int i;
//	char name[] = "/uartdrv0";

	const struct {
		volatile u32 *base;
		int dev;
		unsigned enableBit;
		unsigned irq;
	} info[] = {
		{ (void *)0x40013800, pctl_usart1, 37 + 16 },
		{ (void *)0x40004400, pctl_usart2, 38 + 16 },
		{ (void *)0x40004800, pctl_usart3, 39 + 16 },
		{ (void *)0x40004c00, pctl_uart4, 48 + 16 },
		{ (void *)0x40005000, pctl_uart5, 49 + 16 },
	};

	uart_t *uartptr;

	for (i = 0; i < 5; ++i) {
		if (uarts & (1 << i)) {
			rcc_devClk(info[i].dev, 3);

			uartptr = malloc(sizeof(uart_t));

			mutexCreate(&uartptr->mutex);
			condCreate(&uartptr->cond);

			uartptr->base = info[i].base;

			uartptr->txbeg = NULL;
			uartptr->txend = NULL;

			uartptr->rxbeg = NULL;
			uartptr->rxend = NULL;
			uartptr->rxdr = 0;
			uartptr->rxdw = 0;

			/* Set up UART to 9600,8,n,1 16-bit oversampling */

			/* disable UART */
			*(uartptr->base + cr1) &= ~(1 << 13);
			/* 1 start, 1 stop bit */
			*(uartptr->base + cr2) = 0;
			/* enable receiver, enable transmitter, enable rxd irq*/
			*(uartptr->base + cr1) = 0x2c;
			/* no aditional settings */
			*(uartptr->base + cr3) = 0;
			*(uartptr->base + cr1) |= 1 << 13;

			uartptr->baud = 9600;

//			name[8] = '1' + i;

			interrupt(info[i].irq, uart_irq, uartptr, uartptr->cond, &uartptr->inth);
		}
	}

	return EOK;
}
