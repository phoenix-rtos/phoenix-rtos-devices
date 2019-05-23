/*
 * Phoenix-RTOS
 *
 * iMX RT UART driver
 *
 * Copyright 2017-2019 Phoenix Systems
 * Author: Kamil Amanowicz, Marek Bialowas, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/threads.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include <libtty.h>

#include "common.h"
#include "uart.h"

#define UART1_POS 0
#define UART2_POS (UART1_POS + UART1)
#define UART3_POS (UART2_POS + UART2)
#define UART4_POS (UART3_POS + UART3)
#define UART5_POS (UART4_POS + UART4)
#define UART6_POS (UART5_POS + UART5)
#define UART7_POS (UART6_POS + UART6)
#define UART8_POS (UART7_POS + UART7)

#define UART_CNT (UART1 + UART2 + UART3 + UART4 + UART5 + UART6 + UART7 + UART8)

#define MODULE_CLK 80000000

#define BUFSIZE 256


typedef struct uart_s {
	char stack[1024] __attribute__ ((aligned(8)));

	volatile u32 *base;
	u32 mode;
	u16 dev_no;

	handle_t cond;
	handle_t inth;
	handle_t lock;

	size_t rxFifoSz;
	size_t txFifoSz;

	libtty_common_t tty_common;
} uart_t;


struct {
	uart_t uarts[UART_CNT];
	unsigned int port;
} uart_common;


static const int uartConfig[] = { UART1, UART2, UART3, UART4, UART5, UART6, UART7, UART8 };


static const int uartPos[] = { UART1_POS, UART2_POS, UART3_POS, UART4_POS, UART5_POS, UART6_POS,
	UART7_POS, UART8_POS };


enum { veridr = 0, paramr, globalr, pincfgr, baudr, statr, ctrlr, datar, matchr, modirr, fifor, waterr };


static int uart_handleIntr(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	*(uart->base + ctrlr) &= ~((1 << 23) | (1 << 21));

	return uart->cond;
}


static inline int uart_getRXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 24) & 0xff;
}


static inline int uart_getTXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 8) & 0xff;
}


static void uart_intrThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;

	for (;;) {
		/* wait for character or transmit data */
		mutexLock(uart->lock);
		while (!uart_getRXcount(uart)) { /* nothing to RX */
			if (libtty_txready(&uart->tty_common)) { /* something to TX */
				if (uart_getTXcount(uart) < uart->txFifoSz) /* TX ready */
					break;
				else
					*(uart->base + ctrlr) |= 1 << 23;
			}

			*(uart->base + ctrlr) |= 1 << 21;

			condWait(uart->cond, uart->lock, 0);
		}

		mutexUnlock(uart->lock);

		/* RX */
		while (uart_getRXcount(uart))
			libtty_putchar(&uart->tty_common, *(uart->base + datar), NULL);

		/* TX */
		while (libtty_txready(&uart->tty_common) && uart_getTXcount(uart) < uart->txFifoSz)
			*(uart->base + datar) = libtty_getchar(&uart->tty_common, NULL);
	}
}


static void signal_txready(void *_uart)
{
	uart_t *uartptr = (uart_t *)_uart;

	mutexLock(uartptr->lock);
	condSignal(uartptr->cond);
	mutexUnlock(uartptr->lock);
}


static void set_cflag(void *_uart, tcflag_t* cflag)
{
	uart_t *uartptr = (uart_t *)_uart;
	u32 t;

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	/* CSIZE ony CS7 and CS8 (default) is supported */
	if ((*cflag & CSIZE) == CS7) {
		*(uartptr->base + ctrlr) |= 1 << 11;
	}
	else { /* CS8 */
		*cflag &= ~CSIZE;
		*cflag |= CS8;

		*(uartptr->base + ctrlr) &= ~(1 << 11);
	}

	/* parity */
	t = *(uartptr->base + ctrlr) & ~3;
	if (*cflag & PARENB) {
		t |= 1 << 1;
		if (*cflag & PARODD)
			t |= 1;
	}
	*(uartptr->base + ctrlr) = t;

	/* stop bits */
	if (*cflag & CSTOPB)
		*(uartptr->base + baudr) |= (1 << 13);
	else
		*(uartptr->base + baudr) &= ~(1 << 13);

	/* reenable TX and RX */
	*(uartptr->base + ctrlr) |= (1 << 19) | (1 << 18);
}


static u32 calculate_baudrate(speed_t baud)
{
	int osr, sbr, bestSbr, bestOsr, bestErr = 1000, t, baud_rate = libtty_baudrate_to_int(baud);

	if (!baud_rate)
		return 0;

	for (osr = 3; osr < 32; ++osr) {
		sbr = MODULE_CLK / (baud_rate * (osr + 1));
		sbr &= 0xfff;
		t = MODULE_CLK / (sbr * (osr + 1));

		if (t > baud_rate)
			t = ((t - baud_rate) * 1000) / baud_rate;
		else
			t = ((baud_rate - t) * 1000) / baud_rate;

		if (t < bestErr) {
			bestErr = t;
			bestOsr = osr;
			bestSbr = sbr;
		}

		/* Finish if error is < 1% */
		if (bestErr < 10)
			break;
	}

	return (bestOsr << 24) | ((bestOsr <= 6) << 17) | bestSbr;
}


static void set_baudrate(void *_uart, speed_t baud)
{
	u32 reg, t;
	uart_t *uartptr = (uart_t *)_uart;

	reg = calculate_baudrate(baud);

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	t = *(uartptr->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0xfff);
	*(uartptr->base + baudr) = t | reg;

	/* reenable TX and RX */
	*(uartptr->base + ctrlr) |= (1 << 19) | (1 << 18);
}


int uart_handleMsg(msg_t *msg, int dev)
{
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	uart_t *uart;

	dev -= id_uart1;

	if (dev > 7 || !uartConfig[dev])
		return -EINVAL;

	uart = &uart_common.uarts[uartPos[dev]];

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.io.err = EOK;
			break;

		case mtWrite:
			msg->o.io.err = libtty_write(&uart->tty_common, msg->i.data, msg->i.size, msg->i.io.mode);
			break;

		case mtRead:
			msg->o.io.err = libtty_read(&uart->tty_common, msg->o.data, msg->o.size, msg->i.io.mode);
			break;

		case mtGetAttr:
			if (msg->i.attr.type == atPollStatus)
				msg->o.attr.val = libtty_poll_status(&uart->tty_common);
			else
				msg->o.attr.val = -EINVAL;
			break;

		case mtDevCtl:
			in_data = ioctl_unpack(msg, &request, NULL);
			pid = ioctl_getSenderPid(msg);
			err = libtty_ioctl(&uart->tty_common, pid, request, in_data, &out_data);
			ioctl_setResponse(msg, request, err, out_data);
			break;
	}

	return EOK;
}


int uart_init(void)
{
	int i, dev;
	u32 t;
	uart_t *uart;
	libtty_callbacks_t callbacks;
	static const size_t fifoSzLut[] = { 1, 4, 8, 16, 32, 64, 128, 256 };
	static const struct {
		volatile u32 *base;
		int dev;
		unsigned irq;
	} info[] = {
		{ (void *)0x40184000, pctl_clk_lpuart1, 20 + 16 },
		{ (void *)0x40188000, pctl_clk_lpuart2, 21 + 16 },
		{ (void *)0x4018c000, pctl_clk_lpuart3, 22 + 16 },
		{ (void *)0x40190000, pctl_clk_lpuart4, 23 + 16 },
		{ (void *)0x40194000, pctl_clk_lpuart5, 24 + 16 },
		{ (void *)0x40198000, pctl_clk_lpuart6, 25 + 16 },
		{ (void *)0x4019c000, pctl_clk_lpuart7, 26 + 16 },
		{ (void *)0x401a0000, pctl_clk_lpuart8, 27 + 16 }
	};

	for (i = 0, dev = 0; dev < sizeof(uartConfig) / sizeof(uartConfig[0]); ++dev) {
		if (!uartConfig[dev])
			continue;

		uart = &uart_common.uarts[i++];
		uart->base = info[dev].base;
		common_setClock(info[dev].dev, clk_state_run);

		if (condCreate(&uart->cond) < 0 || mutexCreate(&uart->lock) < 0)
			return -1;

		callbacks.arg = uart;
		callbacks.set_baudrate = set_baudrate;
		callbacks.set_cflag = set_cflag;
		callbacks.signal_txready = signal_txready;

		if (libtty_init(&uart->tty_common, &callbacks, BUFSIZE) < 0)
			return -1;

		/* Wait for kernel to stop sending data over uart */
		while (*(uart->base + waterr) & 0x700)
			usleep(100);

		/* Disable TX and RX */
		*(uart->base + ctrlr) &= ~((1 << 19) | (1 << 18));

		/* Reset all internal logic and registers, except the Global Register */
		*(uart->base + globalr) |= 1 << 1;
		common_dataBarrier();
		*(uart->base + globalr) &= ~(1 << 1);
		common_dataBarrier();

		/* Disable input trigger */
		*(uart->base + pincfgr) &= ~3;

		/* Set 115200 default baudrate */
		t = *(uart->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0xfff);
		*(uart->base + baudr) = t | calculate_baudrate(B115200);

		/* Set 8 bit and no parity mode */
		*(uart->base + ctrlr) &= ~0x117;

		/* One stop bit */
		*(uart->base + baudr) &= ~(1 << 13);

		*(uart->base + waterr) = 0;

		/* Enable FIFO */
		*(uart->base + fifor) |= (1 << 7) | (1 << 3);
		*(uart->base + fifor) |= 0x3 << 14;

		/* Clear all status flags */
		*(uart->base + statr) |= 0xc01fc000;

		uart->rxFifoSz = fifoSzLut[*(uart->base) & 0x7];
		uart->txFifoSz = fifoSzLut[(*(uart->base) >> 4) & 0x7];

		/* Enable receiver interrupt */
		*(uart->base + ctrlr) |= 1 << 21;

		/* Enable TX and RX */
		*(uart->base + ctrlr) |= (1 << 19) | (1 << 18);

		interrupt(info[dev].irq, uart_handleIntr, (void *)uart, uart->cond, NULL);

		beginthread(uart_intrThread, 2, &uart->stack, sizeof(uart->stack), uart);
	}

	return 0;
}
