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

#include HAL
#include "uartdrv.h"
#include "gpiodrv.h"
#include "proc/threads.h"
#include "proc/msg.h"
#include "../include/errno.h"

/* Need getter for CPU clock. Temporary solution */
#define F_OSC 2097152


/* Temporary solution */
unsigned int uartdrv_id[3];


struct {
	volatile unsigned int *base;
	volatile unsigned int *rcc;
	unsigned int *port;

	volatile char txdfifo[32];
	volatile unsigned int txdr;
	volatile unsigned int txdw;

	volatile char rxdfifo[32];
	volatile unsigned int rxdr;
	volatile unsigned int rxdw;

	thread_t *txdEv;
	thread_t *rxdEv;

	spinlock_t txdSp;
	spinlock_t rxdSp;

	intr_handler_t irqHandler;
} uartdrv_common[3];


enum { sr = 0, dr, brr, cr1, cr2, cr3, gtpr };


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


static int uartdrv_irqHandler(unsigned int n, cpu_context_t *context, void *arg)
{
	if ((*(uartdrv_common[(int)arg].base + sr) & (1 << 7))) { /* Txd buffer empty */
		if (uartdrv_common[(int)arg].txdr != uartdrv_common[(int)arg].txdw) {
			*(uartdrv_common[(int)arg].base + dr) = uartdrv_common[(int)arg].txdfifo[uartdrv_common[(int)arg].txdr];
			uartdrv_common[(int)arg].txdr = (uartdrv_common[(int)arg].txdr + 1) % sizeof(uartdrv_common[(int)arg].txdfifo);
		}
		else
			*(uartdrv_common[(int)arg].base + cr1) &= ~(1 << 7);

		proc_threadWakeup(&uartdrv_common[(int)arg].txdEv);
	}

	if (*(uartdrv_common[(int)arg].base + sr) & ((1 << 5) | (1 << 3))) { /* Rxd buffer not empty */
		char rxd = *(uartdrv_common[(int)arg].base + dr);

		if (uartdrv_common[(int)arg].rxdr != ((uartdrv_common[(int)arg].rxdw + 1) % sizeof(uartdrv_common[(int)arg].rxdfifo))) {
			uartdrv_common[(int)arg].rxdfifo[uartdrv_common[(int)arg].rxdw] = rxd;
			uartdrv_common[(int)arg].rxdw = ((uartdrv_common[(int)arg].rxdw + 1) % sizeof(uartdrv_common[(int)arg].rxdfifo));
		}

		proc_threadWakeup(&uartdrv_common[(int)arg].rxdEv);
	}

	return 0;
}


static int uartdrv_write(void* buff, unsigned int bufflen, int uart)
{
	int i;

	for (i = 0; i < bufflen; ++i) {
		hal_spinlockSet(&uartdrv_common[uart].txdSp);
		while (uartdrv_common[uart].txdr == ((uartdrv_common[uart].txdw + 1) % sizeof(uartdrv_common[uart].txdfifo)))
			proc_threadWait(&uartdrv_common[uart].txdEv, &uartdrv_common[uart].txdSp, 0);

		uartdrv_common[uart].txdfifo[uartdrv_common[uart].txdw] = ((char *)buff)[i];
		uartdrv_common[uart].txdw = (uartdrv_common[uart].txdw + 1) % sizeof(uartdrv_common[uart].txdfifo);
		*(uartdrv_common[uart].base + cr1) |= 1 << 7;
		hal_spinlockClear(&uartdrv_common[uart].txdSp);
	}

	return i;
}


static int uartdrv_read(void* buff, unsigned int count, int uart, char mode, unsigned int timeout)
{
	int i, err;

	if (mode == UARTDRV_MNBLOCK)
		timeout = 0;

	for (i = 0; i < count; ++i) {
		hal_spinlockSet(&uartdrv_common[uart].rxdSp);

		if (mode == UARTDRV_MNBLOCK && uartdrv_common[uart].rxdr == uartdrv_common[uart].rxdw) {
			hal_spinlockClear(&uartdrv_common[uart].rxdSp);
			break;
		}

		err = 0;
		while (uartdrv_common[uart].rxdr == uartdrv_common[uart].rxdw && !err)
			err = proc_threadWait(&uartdrv_common[uart].rxdEv, &uartdrv_common[uart].rxdSp, timeout);

		if (uartdrv_common[uart].rxdr == uartdrv_common[uart].rxdw && err == -ETIME) {
			hal_spinlockClear(&uartdrv_common[uart].rxdSp);
			break;
		}

		((char *)buff)[i] = uartdrv_common[uart].rxdfifo[uartdrv_common[uart].rxdr];

		if (!(*(uartdrv_common[uart].base + cr1) & (1 << 12)) && (*(uartdrv_common[uart].base + cr1) & (1 << 10)))
			((char *)buff)[i] &= 0x7f;

		uartdrv_common[uart].rxdr = (uartdrv_common[uart].rxdr + 1) % sizeof(uartdrv_common[uart].rxdfifo);

		hal_spinlockClear(&uartdrv_common[uart].rxdSp);
	}

	return i;
}


static void uartdrv_thread(void *arg)
{
	char buff[64];
	int err;
	unsigned int tmp;
	msghdr_t hdr;
	uartdrv_data_t *data = (uartdrv_data_t *)buff;
	uartdrv_devctl_t *devclt = (uartdrv_devctl_t *)buff;
	size_t size;

	for (;;) {
		tmp = proc_recv(*uartdrv_common[(int)arg].port, buff, sizeof(buff), &hdr);

		size = min(sizeof(buff), hdr.rsize);

		switch (hdr.op) {
		case MSG_READ:
			if (*(uartdrv_common[(int)arg].base + cr1) & (1 << 13)) {
				tmp = uartdrv_read(buff, size, (int)arg, UARTDRV_MNORMAL, 0);
				if (hdr.type == MSG_NORMAL)
					proc_respond(*uartdrv_common[(int)arg].port, EOK, buff, tmp);
			}
			else if (hdr.type == MSG_NORMAL) {
					proc_respond(*uartdrv_common[(int)arg].port, EINVAL, NULL, 0);
			}

			break;

		case MSG_WRITE:
			if (*(uartdrv_common[(int)arg].base + cr1) & (1 << 13)) {
				tmp = uartdrv_write(data->buff, tmp - sizeof(data->off), (int)arg);
				if (hdr.type == MSG_NORMAL)
					proc_respond(*uartdrv_common[(int)arg].port, EOK, &tmp, sizeof(tmp));
			}
			else if (hdr.type == MSG_NORMAL) {
				proc_respond(*uartdrv_common[(int)arg].port, EINVAL, NULL, 0);
			}

			break;

		case MSG_DEVCTL: {
			switch (devclt->type) {
			case UARTDRV_DEF:
				err = EOK;
				tmp = *(uartdrv_common[(int)arg].base + cr1) & (1 << 13);
				*(uartdrv_common[(int)arg].base + cr1) &= ~(1 << 13);

				if ((devclt->def.bits == 8 && devclt->def.parity != UARTDRV_PARNONE))
					*(uartdrv_common[(int)arg].base + cr1) |= (1 << 12);
				else if ((devclt->def.bits == 7 && devclt->def.parity != UARTDRV_PARNONE) ||
						(devclt->def.bits == 8 && devclt->def.parity == UARTDRV_PARNONE))
					*(uartdrv_common[(int)arg].base + cr1) &= ~(1 << 12);
				else
					err = EINVAL;

				if (err == EOK) {
					*(uartdrv_common[(int)arg].base + brr) = F_OSC / devclt->def.baud;

					if (devclt->def.parity != UARTDRV_PARNONE)
						*(uartdrv_common[(int)arg].base + cr1) |= (1 << 10);
					else
						*(uartdrv_common[(int)arg].base + cr1) &= ~(1 << 10);

					if (devclt->def.parity == UARTDRV_PARODD)
						*(uartdrv_common[(int)arg].base + cr1) |= (1 << 9);
					else
						*(uartdrv_common[(int)arg].base + cr1) &= ~(1 << 9);

					*(uartdrv_common[(int)arg].base + cr1) |= (!!(devclt->def.enable) << 13);
				}
				else if (tmp) {
					*(uartdrv_common[(int)arg].base + cr1) |= (1 << 13);
				}

				if (hdr.type == MSG_NORMAL)
					proc_respond(*uartdrv_common[(int)arg].port, err, NULL, 0);

				break;

			case UARTDRV_GET:
				if (*(uartdrv_common[(int)arg].base + cr1) & (1 << 13)) {
					tmp = uartdrv_read(buff, size, (int)arg, devclt->get.mode, devclt->get.timeout);
					if (hdr.type == MSG_NORMAL)
						proc_respond(*uartdrv_common[(int)arg].port, EOK, buff, tmp);
				}
				else if (hdr.type == MSG_NORMAL) {
						proc_respond(*uartdrv_common[(int)arg].port, EINVAL, NULL, 0);
				}

				break;

			case UARTDRV_ENABLE:
				*(uartdrv_common[(int)arg].base + cr1) &= ~(!devclt->enable.state << 13);
				*(uartdrv_common[(int)arg].base + cr1) |= (!!devclt->enable.state << 13);

				if (hdr.type == MSG_NORMAL)
					proc_respond(*uartdrv_common[(int)arg].port, EOK, NULL, 0);

				break;

			default:
				if (hdr.type == MSG_NORMAL)
					proc_respond(*uartdrv_common[(int)arg].port, EINVAL, NULL, 0);

				break;
			}
			break;
		}

		default:
			if (hdr.type == MSG_NORMAL) {
				proc_respond(*uartdrv_common[(int)arg].port, EINVAL, NULL, 0);
			}
			break;
		}
	}
}


int uartdrv_enable(int uart, int state)
{
	uartdrv_devctl_t devctl;

	devctl.type = UARTDRV_ENABLE;
	devctl.enable.state = state;

	return proc_send(uartdrv_id[uart & 1], MSG_DEVCTL, &devctl, sizeof(devctl), MSG_NORMAL, NULL, 0);
}


void uartdrv_init(void)
{
	int i;
	char name[] = "/uartdrv0";

	uartdrv_common[0].base = (void *)0x40013800;
	uartdrv_common[1].base = (void *)0x40004c00;
	uartdrv_common[2].base = (void *)0x40004400;
	uartdrv_common[0].rcc = (void *)0x40023800;
	uartdrv_common[1].rcc = (void *)0x40023800;
	uartdrv_common[2].rcc = (void *)0x40023800;

	/* TODO use gpiodrv message interface */

	/* USART 1 */
	/* Init tx pin - output, usart1, push-pull, high speed, no pull-up */
	gpiodrv_configPin(GPIOB, 6, 2, 7, 0, 2, 0);

	/* Init rxd pin - input, usart1, push-pull, high speed, no pull-up */
	gpiodrv_configPin(GPIOB, 7, 2, 7, 0, 2, 0);

	/* Infrared enable pin */
	gpiodrv_configPin(GPIOB, 5, 1, 0, 0, 2, 0);

	/* Enable usart1 clock */
	*(uartdrv_common[0].rcc + rcc_apb2enr) |= 1 << 14;

	hal_cpuDataBarrier();

	/* UART 4 */
	/* Init tx pin - output, uart4, push-pull, high speed, no pull-up */
	gpiodrv_configPin(GPIOC, 10, 2, 8, 0, 2, 0);

	/* Init rxd pin - input, uart4, push-pull, high speed, no pull-up */
	gpiodrv_configPin(GPIOC, 11, 2, 8, 0, 2, 0);

	/* Enable uart4 clock */
	*(uartdrv_common[1].rcc + rcc_apb1enr) |= 1 << 19;

	hal_cpuDataBarrier();

	/* USART 2 */
	/* Init tx pin - output, usart2, push-pull, high speed, no pull-up */
	gpiodrv_configPin(GPIOD, 5, 2, 7, 0, 2, 0);

	/* Init rxd pin - input, usart2, push-pull, high speed, no pull-up */
	gpiodrv_configPin(GPIOD, 6, 2, 7, 0, 2, 0);

	/* Enable usart1 clock */
	*(uartdrv_common[2].rcc + rcc_apb1enr) |= 1 << 17;

	hal_cpuDataBarrier();

	for (i = 0; i < sizeof(uartdrv_common) / sizeof(uartdrv_common[0]); ++i) {
		hal_spinlockCreate(&uartdrv_common[i].txdSp, "txd spinlock");
		hal_spinlockCreate(&uartdrv_common[i].rxdSp, "rxd spinlock");
		uartdrv_common[i].txdEv = NULL;
		uartdrv_common[i].rxdEv = NULL;
		uartdrv_common[i].txdr = 0;
		uartdrv_common[i].txdw = 0;
		uartdrv_common[i].rxdr = 0;
		uartdrv_common[i].rxdw = 0;

		/* Set up UART to 9600,8,n,1 16-bit oversampling */
		*(uartdrv_common[i].base + cr1) &= ~(1 << 13);	/* disable UART */
		*(uartdrv_common[i].base + cr2) = 0;			/* 1 start, 1 stop bit */
		*(uartdrv_common[i].base + cr1) = 0x2c;		/* enable receiver, enable transmitter, enable rxd irq*/
		*(uartdrv_common[i].base + cr3) = 0;			/* no aditional settings */
		*(uartdrv_common[i].base + brr) = F_OSC / 9600;
		*(uartdrv_common[i].base + cr1) |= 1 << 13;

		proc_portCreate(&uartdrv_id[i]);
		name[8] = '0' + i;
		proc_portRegister(uartdrv_id[i], name);

		uartdrv_common[i].port = &uartdrv_id[i];

		uartdrv_common[i].irqHandler.next = NULL;
		uartdrv_common[i].irqHandler.prev = NULL;
		uartdrv_common[i].irqHandler.f = uartdrv_irqHandler;
		uartdrv_common[i].irqHandler.data = (void *)i;
		uartdrv_common[i].irqHandler.pmap = NULL;
	}

	hal_interruptsSetHandler(53, &uartdrv_common[0].irqHandler);
	hal_interruptsSetHandler(64, &uartdrv_common[1].irqHandler);
	hal_interruptsSetHandler(54, &uartdrv_common[1].irqHandler);

	proc_threadCreate(0, uartdrv_thread, 2, 512, 0, (void *)0);
	proc_threadCreate(0, uartdrv_thread, 2, 512, 0, (void *)1);
	proc_threadCreate(0, uartdrv_thread, 2, 512, 0, (void *)2);
}
