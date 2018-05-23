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

#include "stm32-multi.h"
#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "rcc.h"


struct {
	volatile unsigned int *base;
	unsigned int port;
	unsigned int baud;

	volatile char *txbeg;
	volatile char *txend;

	volatile char rxdfifo[32];
	volatile unsigned int rxdr;
	volatile unsigned int rxdw;
	volatile char *rxbeg;
	volatile char *rxend;
	volatile unsigned int read;

	handle_t cond;
	handle_t mutex;
} uart_common[5];


static const int uart2pctl[] = { pctl_usart1, pctl_usart2, pctl_usart3, pctl_uart4, pctl_uart5 };


enum { sr = 0, dr, brr, cr1, cr2, cr3, gtpr };


static int uart_irq(unsigned int n, void *arg)
{
	int uart = (int)arg, release = -1;

	if ((*(uart_common[uart].base + sr) & (1 << 7))) {
		/* Txd buffer empty */
		if (uart_common[uart].txbeg != uart_common[uart].txend) {
			*(uart_common[uart].base + dr) = *(uart_common[uart].txbeg++);
		}
		else {
			*(uart_common[uart].base + cr1) &= ~(1 << 7);
			release = 1;
		}
	}

	if (*(uart_common[uart].base + sr) & ((1 << 5) | (1 << 3))) {
		/* Rxd buffer not empty */
		uart_common[uart].rxdfifo[uart_common[uart].rxdw++] = *(uart_common[uart].base + dr);
		uart_common[uart].rxdw %= sizeof(uart_common[uart].rxdfifo);

		if (uart_common[uart].rxdr == uart_common[uart].rxdw)
			uart_common[uart].rxdr = (uart_common[uart].rxdr + 1) % sizeof(uart_common[uart].rxdfifo);
	}

	if (uart_common[uart].rxbeg != NULL) {
		while (uart_common[uart].rxdr != uart_common[uart].rxdw && uart_common[uart].rxbeg != uart_common[uart].rxend) {
			*(uart_common[uart].rxbeg++) = uart_common[uart].rxdfifo[uart_common[uart].rxdr++];
			uart_common[uart].rxdr %= sizeof(uart_common[uart].rxdfifo);
			uart_common[uart].read++;
		}

		release = 1;
	}

	return release;
}


static int uart_isEnabled(int uart)
{
	platformctl_t pctl;

	if (uart < usart1 || uart > uart5)
		return -EINVAL;

	pctl.action = pctl_get;
	pctl.type = pctl_devclk;
	pctl.devclk.dev = uart2pctl[uart - usart1];
	platformctl(&pctl);

	return !!pctl.devclk.state;
}


int uart_configure(int uart, char bits, char parity, unsigned int baud, char enable)
{
	int en, err = EOK;

	if (uart < usart1 || uart > uart5)
		return -EINVAL;

	en = uart_isEnabled(uart);

	rcc_devClk(uart2pctl[uart], 1);

	mutexLock(uart_common[uart].mutex);

	*(uart_common[uart].base + cr1) &= ~(1 << 13);
	dataBarier();

	if (bits == 8 && parity != uart_parnone)
		*(uart_common[uart].base + cr1) |= 1 << 12;
	else if ((bits == 7 && parity != uart_parnone) || (bits == 8 && parity == uart_parnone))
		*(uart_common[uart].base + cr1) &= ~(1 << 12);
	else
		err = -EINVAL;

	if (err == EOK) {
		uart_common[uart].baud = baud;
		*(uart_common[uart].base + brr) = rcc_getCpufreq() / baud;

		if (parity != uart_parnone)
			*(uart_common[uart].base + cr1) |= 1 << 10;
		else
			*(uart_common[uart].base + cr1) &= ~(1 << 10);

		if (parity == uart_parodd)
			*(uart_common[uart].base + cr1) |= 1 << 9;
		else
			*(uart_common[uart].base + cr1) &= ~(1 << 9);

		*(uart_common[uart].base + cr1) |= (!!enable) << 13;

		en = enable;
	}

	dataBarier();
	*(uart_common[uart].base + cr1) |= 1 << 13;
	dataBarier();

	rcc_devClk(uart2pctl[uart], !!en);

	mutexUnlock(uart_common[uart].mutex);

	return err;
}


int uart_write(int uart, void* buff, unsigned int bufflen)
{
	if (uart < usart1 || uart > uart5)
		return -EINVAL;

	if (!uart_isEnabled(uart))
		return -EIO;

	mutexLock(uart_common[uart].mutex);
	keepidle(1);

	uart_common[uart].txbeg = buff;
	uart_common[uart].txend = buff + bufflen;

	*(uart_common[uart].base + cr1) |= 1 << 7;

	while (uart_common[uart].txbeg != uart_common[uart].txend)
		condWait(uart_common[uart].cond, uart_common[uart].mutex, 0);

	uart_common[uart].txbeg = NULL;
	uart_common[uart].txend = NULL;

	keepidle(0);
	mutexUnlock(uart_common[uart].mutex);

	return bufflen;
}


int uart_read(int uart, void* buff, unsigned int count, char mode, unsigned int timeout)
{
	int i, err, read;

	if (uart < usart1 || uart > uart5)
		return -EINVAL;

	if (!uart_isEnabled(uart))
		return -EIO;

	mutexLock(uart_common[uart].mutex);
	keepidle(1);

	uart_common[uart].read = 0;
	uart_common[uart].rxend = buff + count;
	uart_common[uart].rxbeg = buff;

	/* Provoke UART exception to fire so that existing data from
	 * rxdfifo is copied into buff. The handler will clear this
	 * bit. */

	*(uart_common[uart].base + cr1) |= 1 << 7;

	while (mode != uart_mnblock && uart_common[uart].rxbeg != uart_common[uart].rxend) {
		err = condWait(uart_common[uart].cond, uart_common[uart].mutex, timeout);

		if (timeout && err == -ETIME)
			break;
	}

	uart_common[uart].rxbeg = NULL;
	dataBarier();
	uart_common[uart].rxend = NULL;

	read = uart_common[uart].read;
	if (!(*(uart_common[uart].base + cr1) & (1 << 12)) && (*(uart_common[uart].base + cr1) & (1 << 10))) {
		for (i = 0; i < read; ++i)
			((char *)buff)[i] &= 0x7f;
	}

	keepidle(0);
	mutexUnlock(uart_common[uart].mutex);

	return read;
}


int uart_init(void)
{
	int i;
	platformctl_t pctl;
	const struct {
		volatile u32 *base;
		int dev;
		unsigned irq;
	} info[] = {
		{ (void *)0x40013800, pctl_usart1, 37 + 16 },
		{ (void *)0x40004400, pctl_usart2, 38 + 16 },
		{ (void *)0x40004800, pctl_usart3, 39 + 16 },
		{ (void *)0x40004c00, pctl_uart4, 48 + 16 },
		{ (void *)0x40005000, pctl_uart5, 49 + 16 },
	};

	pctl.action = pctl_get;
	pctl.type = pctl_cpuclk;
	platformctl(&pctl);

	for (i = 0; i < 5; ++i) {
		rcc_devClk(info[i].dev, 1);

		mutexCreate(&uart_common[i].mutex);
		condCreate(&uart_common[i].cond);

		uart_common[i].base = info[i].base;

		uart_common[i].txbeg = NULL;
		uart_common[i].txend = NULL;

		uart_common[i].rxbeg = NULL;
		uart_common[i].rxend = NULL;
		uart_common[i].rxdr = 0;
		uart_common[i].rxdw = 0;

		/* Set up UART to 9600,8,n,1 16-bit oversampling */

		/* disable UART */
		*(uart_common[i].base + cr1) &= ~(1 << 13);
		/* 9600 baudrate */
		*(uart_common[i].base + brr) = pctl.cpuclk.hz / 9600;
		uart_common[i].baud = 9600;
		/* 1 start, 1 stop bit */
		*(uart_common[i].base + cr2) = 0;
		/* enable receiver, enable transmitter, enable rxd irq*/
		*(uart_common[i].base + cr1) = 0x2c;
		/* no aditional settings */
		*(uart_common[i].base + cr3) = 0;
		/* UART enable */
		*(uart_common[i].base + cr1) |= 1 << 13;

		/* Left UART4 enabled only */
		if (i != 3)
			rcc_devClk(info[i].dev, 0);

		interrupt(info[i].irq, uart_irq, (void *)i, uart_common[i].cond, NULL);
	}

	return EOK;
}
