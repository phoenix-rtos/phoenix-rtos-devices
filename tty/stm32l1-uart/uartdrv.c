/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 UART driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/pwman.h>
#include <sys/platform.h>

#include "uartdrv.h"


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


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


typedef struct {
	enum { PLATCTL_SET = 0, PLATCTL_GET } action;
	enum { PLATCTL_DEVCLOCK = 0, PLATCTL_CPUCLOCK } type;

	union {
		struct {
			unsigned dev, state;
		} devclock;

		struct {
			unsigned hz;
		} cpuclock;
	};
} __attribute__((packed)) platformctl_t;


static int uartdrv_irqHandler(unsigned int n, void *arg)
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


static int uartdrv_write(void* buff, unsigned int bufflen, uart_t *uart)
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


static int uartdrv_read(void* buff, unsigned int count, uart_t *uart, char mode, unsigned int timeout)
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
	__asm__ volatile ("dmb");
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


static int uartdrv_adjustBaud(uart_t *uart, int cpufreq)
{
	platformctl_t pctl;
	pctl.action = PLATCTL_GET;
	pctl.type = PLATCTL_CPUCLOCK;

	platformctl(&pctl);

	if (cpufreq != pctl.cpuclock.hz) {
		/* Adjust to new clock frequency */

		cpufreq = pctl.cpuclock.hz;

		*(uart->base + cr1) &= ~(1 << 13);
		*(uart->base + brr) = cpufreq / uart->baud;
		*(uart->base + cr1) |= 1 << 13;
	}

	return cpufreq;
}


static void uartdrv_thread(void *arg)
{
	int err;
	unsigned int tmp;
	char buff[64];
	msghdr_t hdr;
	uartdrv_data_t *data = (uartdrv_data_t *)buff;
	uartdrv_devctl_t *devctl = (uartdrv_devctl_t *)buff;
	size_t size;
	uart_t *uart = arg;

	int cpufreq = 0;


	for (;;) {
		tmp = recv(uart->port, buff, sizeof(buff), &hdr);
		size = min(sizeof(buff), hdr.rsize);

		cpufreq = uartdrv_adjustBaud(uart, cpufreq);

		switch (hdr.op) {
		case READ:
			if (*(uart->base + cr1) & (1 << 13)) {
				tmp = uartdrv_read(buff, size, uart, UARTDRV_MNORMAL, 0);
				if (hdr.type == NORMAL)
					respond(uart->port, EOK, buff, tmp);
			}
			else if (hdr.type == NORMAL) {
				respond(uart->port, EINVAL, NULL, 0);
			}

			break;

		case WRITE:
			if (*(uart->base + cr1) & (1 << 13)) {
				tmp = uartdrv_write(data->buff, tmp - sizeof(data->off), uart);
				if (hdr.type == NORMAL)
					respond(uart->port, EOK, &tmp, sizeof(tmp));
			}
			else if (hdr.type == NORMAL) {
				respond(uart->port, EINVAL, NULL, 0);
			}

			break;

		case DEVCTL: {
			switch (devctl->type) {
			case UARTDRV_DEF:
				err = EOK;
				tmp = *(uart->base + cr1) & (1 << 13);
				*(uart->base + cr1) &= ~(1 << 13);

				if ((devctl->def.bits == 8 && devctl->def.parity != UARTDRV_PARNONE))
					*(uart->base + cr1) |= (1 << 12);
				else if ((devctl->def.bits == 7 && devctl->def.parity != UARTDRV_PARNONE) ||
					 (devctl->def.bits == 8 && devctl->def.parity == UARTDRV_PARNONE))
					*(uart->base + cr1) &= ~(1 << 12);
				else
					err = EINVAL;

				if (err == EOK) {
					uart->baud = devctl->def.baud;
					*(uart->base + brr) = cpufreq / uart->baud;

					if (devctl->def.parity != UARTDRV_PARNONE)
						*(uart->base + cr1) |= (1 << 10);
					else
						*(uart->base + cr1) &= ~(1 << 10);

					if (devctl->def.parity == UARTDRV_PARODD)
						*(uart->base + cr1) |= (1 << 9);
					else
						*(uart->base + cr1) &= ~(1 << 9);

					*(uart->base + cr1) |= (!!(devctl->def.enable) << 13);
				}
				else if (tmp) {
					*(uart->base + cr1) |= (1 << 13);
				}

				if (hdr.type == NORMAL)
					respond(uart->port, err, NULL, 0);

				break;

			case UARTDRV_GET:
				if (*(uart->base + cr1) & (1 << 13)) {
					tmp = uartdrv_read(buff, size, uart, devctl->get.mode, devctl->get.timeout);
					if (hdr.type == NORMAL)
						respond(uart->port, EOK, buff, tmp);
				}
				else if (hdr.type == NORMAL) {
					respond(uart->port, EINVAL, NULL, 0);
				}

				break;

			case UARTDRV_ENABLE:
				*(uart->base + cr1) &= ~(!devctl->enable.state << 13);
				*(uart->base + cr1) |= (!!devctl->enable.state << 13);

				if (hdr.type == NORMAL)
					respond(uart->port, EOK, NULL, 0);

				break;

			default:
				if (hdr.type == NORMAL)
					respond(uart->port, EINVAL, NULL, 0);

				break;
			}
			break;
		}

		default:
			if (hdr.type == NORMAL) {
				respond(uart->port, EINVAL, NULL, 0);
			}
			break;
		}
	}
}


void main(void)
{
	unsigned uarts = UARTS;

	uart_t *uartmain = NULL;
	int i, stacksz = 1024 - 32;
	char name[] = "/uartdrv0", *stack;

	const struct {
		volatile u32 *base;
		unsigned apbenr;
		unsigned enableBit;
		unsigned irq;
	} info[] = {
		{ (void *)0x40013800, rcc_apb2enr, 1 << 14, 37 + 16 }, /* USART 1 */
		{ (void *)0x40004400, rcc_apb1enr, 1 << 17, 38 + 16 }, /* USART 2 */
		{ (void *)0x40004800, rcc_apb1enr, 1 << 18, 39 + 16 }, /* USART 3 */
		{ (void *)0x40004c00, rcc_apb1enr, 1 << 19, 48 + 16 }, /* UART 4 */
		{ (void *)0x40005000, rcc_apb1enr, 1 << 20, 49 + 16 }, /* UART 5 */
	};

	volatile u32 *rcc = (void *)0x40023800;
	uart_t *uartptr;

	for (i = 0; i < 5; ++i) {
		if (uarts & (1 << i)) {
			*(rcc + info[i].apbenr) |= info[i].enableBit;

			__asm__ volatile ("dmb");

			/* Enable low power clock */
			*(rcc + info[i].apbenr + 3) |= info[i].enableBit;

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

			name[8] = '1' + i;

			portCreate(&uartptr->port);
			portRegister(uartptr->port, name);

			interrupt(info[i].irq, uartdrv_irqHandler, uartptr, uartptr->cond, &uartptr->inth);

			if (uartmain != NULL) {
				if ((stack = malloc(stacksz)) == NULL || beginthread(uartdrv_thread, 1, stack, stacksz, (void *)uartptr) == -ENOMEM) {
					printf("uartdrv: not enough memory to start uart%d!\n", i + 1);
					free(stack);
					free(uartptr);
				}
			}
			else {
				uartmain = uartptr;
			}
		}
	}

	uartdrv_thread(uartmain);
}
