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

#include HAL
#include "uartdrv-imxrt.h"
#include "proc/threads.h"
#include "proc/msg.h"
#include "../include/errno.h"

/* UART input frequency */
#define F_OSC 8000000000


struct {
	volatile unsigned int *base;
	volatile unsigned int *ccm;

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

	unsigned int port;

	intr_handler_t irqHandler;
} uartdrv_common;


enum { uart_verid = 0, uart_param, uart_global, uart_pincfg, uart_baud, uart_stat, uart_ctrl,
	uart_data, uart_match, uart_modir, uart_fifo, uart_water };


enum { ccm_cscdr1 = 9, ccm_ccgr5 = 31 };


static int uartdrv_irqHandler(unsigned int n, cpu_context_t *context, void *arg)
{
	unsigned char rxd;

	if ((*(uartdrv_common.base + uart_stat) & (1 << 23))) { /* Txd buffer empty */
		if (uartdrv_common.txdr != uartdrv_common.txdw) {
			*(uartdrv_common.base + uart_data) = uartdrv_common.txdfifo[uartdrv_common.txdr];
			uartdrv_common.txdr = (uartdrv_common.txdr + 1) % sizeof(uartdrv_common.txdfifo);
		}
		else
			*(uartdrv_common.base + uart_ctrl) &= ~(1 << 23);

		proc_threadWakeup(&uartdrv_common.txdEv);
	}

	if (*(uartdrv_common.base + uart_water) & (1 << 24)) { /* Rxd buffer not empty */
		if (*(uartdrv_common.base + uart_stat) & (0xf << 16))
			*(uartdrv_common.base + uart_stat) |= 0xf << 16;

		rxd = *(uartdrv_common.base + uart_data);

		if (uartdrv_common.rxdr != ((uartdrv_common.rxdw + 1) % sizeof(uartdrv_common.rxdfifo))) {
			uartdrv_common.rxdfifo[uartdrv_common.rxdw] = rxd;
			uartdrv_common.rxdw = ((uartdrv_common.rxdw + 1) % sizeof(uartdrv_common.rxdfifo));
		}

		proc_threadWakeup(&uartdrv_common.rxdEv);
	}

	return 0;
}


static int uartdrv_write(void* buff, unsigned int bufflen)
{
	int i;

	for (i = 0; i < bufflen; ++i) {
		hal_spinlockSet(&uartdrv_common.txdSp);
		while (uartdrv_common.txdr == ((uartdrv_common.txdw + 1) % sizeof(uartdrv_common.txdfifo)))
			proc_threadWait(&uartdrv_common.txdEv, &uartdrv_common.txdSp, 0);

		uartdrv_common.txdfifo[uartdrv_common.txdw] = ((char *)buff)[i];
		uartdrv_common.txdw = (uartdrv_common.txdw + 1) % sizeof(uartdrv_common.txdfifo);
		*(uartdrv_common.base + uart_ctrl) |= 1 << 23;
		hal_spinlockClear(&uartdrv_common.txdSp);
	}

	return i;
}


static int uartdrv_read(void* buff, unsigned int count)
{
	int i;

	for (i = 0; i < count; ++i) {
		hal_spinlockSet(&uartdrv_common.rxdSp);

		while (uartdrv_common.rxdr == uartdrv_common.rxdw)
			proc_threadWait(&uartdrv_common.rxdEv, &uartdrv_common.rxdSp, 0);

		((char *)buff)[i] = uartdrv_common.rxdfifo[uartdrv_common.rxdr];

		uartdrv_common.rxdr = (uartdrv_common.rxdr + 1) % sizeof(uartdrv_common.rxdfifo);

		hal_spinlockClear(&uartdrv_common.rxdSp);
	}

	return i;
}


static void uartdrv_thread(void *arg)
{
	char buff[64];
	unsigned int tmp;
	msghdr_t hdr;
	uartdrv_data_t *data = (uartdrv_data_t *)buff;
	uartdrv_devctl_t *devclt = (uartdrv_devctl_t *)buff;
	size_t size;

	for (;;) {
		tmp = proc_recv(uartdrv_common.port, buff, sizeof(buff), &hdr);

		size = min(sizeof(buff), hdr.rsize);

		switch (hdr.op) {
		case MSG_READ:
			tmp = uartdrv_read(buff, size);

			if (hdr.type == MSG_NORMAL)
				proc_respond(uartdrv_common.port, EOK, buff, tmp);

			break;

		case MSG_WRITE:
			uartdrv_write(data->buff, tmp - sizeof(data->off));

			if (hdr.type == MSG_NORMAL)
				proc_respond(uartdrv_common.port, EOK, NULL, 0);

			break;

		case MSG_DEVCTL: {
			switch (devclt->type) {
			case UARTDRV_DEF:
				/* TODO */

				if (hdr.type == MSG_NORMAL)
					proc_respond(uartdrv_common.port, EOK, NULL, 0);

				break;

			default:
				if (hdr.type == MSG_NORMAL)
					proc_respond(uartdrv_common.port, EINVAL, NULL, 0);

				break;
			}
			break;
		}

		default:
			if (hdr.type == MSG_NORMAL) {
				proc_respond(uartdrv_common.port, EINVAL, NULL, 0);
			}
			break;
		}
	}
}


void uartdrv_init(void)
{
	u32 t;

	uartdrv_common.base = (void *)0x40184000;
	uartdrv_common.ccm = (void *)0x400fc000;

	/* Set mux and div */
	*(uartdrv_common.ccm + ccm_cscdr1) &= ~((1 << 16) | 0x3f);

	/* Enable clock */
	*(uartdrv_common.ccm + ccm_ccgr5) |= 0x3 << 24;

	/* Reset all internal logic and registers, except the Global Register */
	*(uartdrv_common.base + uart_global) |= 1 << 1;
	hal_cpuDataBarrier();
	*(uartdrv_common.base + uart_global) &= ~(1 << 1);
	hal_cpuDataBarrier();

	/* Set 115200 baudrate */
	t = *(uartdrv_common.base + uart_baud);
	t = (t & ~(0x1f << 24)) | (0x4 << 24);
	*(uartdrv_common.base + uart_baud) = (t & ~0x1fff) | 0x8b;
	*(uartdrv_common.base + uart_baud) &= ~(1 << 29);

	/* Set 8 bit and no parity mode */
	*(uartdrv_common.base + uart_ctrl) &= ~0x117;

	/* One stop bit */
	*(uartdrv_common.base + uart_baud) &= ~(1 << 13);

	*(uartdrv_common.base + uart_water) = 0;

	/* Enable FIFO */
	*(uartdrv_common.base + uart_fifo) |= (1 << 7) | (1 << 3);
	*(uartdrv_common.base + uart_fifo) |= 0x3 << 14;

	/* Clear all status flags */
	*(uartdrv_common.base + uart_stat) |= 0xc01fc000;

	/* Enable TX and RX */
	*(uartdrv_common.base + uart_ctrl) |= (1 << 19) | (1 << 18);

	hal_spinlockCreate(&uartdrv_common.txdSp, "txd spinlock");
	hal_spinlockCreate(&uartdrv_common.rxdSp, "rxd spinlock");
	uartdrv_common.rxdEv = NULL;
	uartdrv_common.txdEv = NULL;
	uartdrv_common.txdr = 0;
	uartdrv_common.txdw = 0;
	uartdrv_common.rxdr = 0;
	uartdrv_common.rxdw = 0;

	/* Enable RXD IRQ */
	*(uartdrv_common.base + uart_ctrl) |= 1 << 21;

	proc_portCreate(&uartdrv_common.port);
	proc_portRegister(uartdrv_common.port, "/uartdrv0");

	uartdrv_common.irqHandler.next = NULL;
	uartdrv_common.irqHandler.prev = NULL;
	uartdrv_common.irqHandler.f = uartdrv_irqHandler;
	uartdrv_common.irqHandler.data = NULL;
	uartdrv_common.irqHandler.pmap = NULL;
	hal_interruptsSetHandler(lpuart1_irq, &uartdrv_common.irqHandler);

	proc_threadCreate(NULL, uartdrv_thread, 2, 1024, NULL, NULL);
}
