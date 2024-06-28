/*
 * Phoenix-RTOS
 *
 * MCX N94x UART driver
 *
 * Copyright 2017-2019, 2024 Phoenix Systems
 * Author: Kamil Amanowicz, Marek Bialowas, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <paths.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/ioctl.h>
#include <sys/msg.h>
#include <sys/threads.h>

#include <libtty.h>
#include <libtty-lf-fifo.h>
#include <libklog.h>

#include "common.h"
#include "uart.h"
#include "dev.h"


#define FLEXCOMM_CAT(x) FLEXCOMM##x##_SEL
#define UARTN_ACTIVE(x) (FLEXCOMM_CAT(x) == FLEXCOMM_UART)

#define UART1_POS 0
#define UART2_POS (UART1_POS + UARTN_ACTIVE(0))
#define UART3_POS (UART2_POS + UARTN_ACTIVE(1))
#define UART4_POS (UART3_POS + UARTN_ACTIVE(2))
#define UART5_POS (UART4_POS + UARTN_ACTIVE(3))
#define UART6_POS (UART5_POS + UARTN_ACTIVE(4))
#define UART7_POS (UART6_POS + UARTN_ACTIVE(5))
#define UART8_POS (UART7_POS + UARTN_ACTIVE(6))
#define UART9_POS (UART8_POS + UARTN_ACTIVE(7))

#define UART_MAX 10
#define UART_CNT (UARTN_ACTIVE(0) + UARTN_ACTIVE(1) + UARTN_ACTIVE(2) + \
	UARTN_ACTIVE(3) + UARTN_ACTIVE(4) + UARTN_ACTIVE(5) + UARTN_ACTIVE(6) + \
	UARTN_ACTIVE(7) + UARTN_ACTIVE(8))


#ifndef UART_RXFIFOSIZE
#define UART_RXFIFOSIZE 128
#endif


typedef struct uart_s {
	char stack[1024] __attribute__ ((aligned(8)));

	volatile uint32_t *base;
	uint32_t mode;

	handle_t cond;
	handle_t inth;
	handle_t lock;

	size_t rxFifoSz;
	size_t txFifoSz;

	libtty_common_t tty_common;

	/* fifo between irq and thread */
	lf_fifo_t rxFifoCtx;
	uint8_t rxFifoData[UART_RXFIFOSIZE];

	/* statistics */
	struct {
		/* NOTE: to read statistics use debugger */
		size_t hw_overrunCntr;
		size_t sw_overrunCntr;
	} stat;
} uart_t;


static struct {
	uart_t uarts[UART_CNT];
	unsigned int major;
	unsigned int ttyminor;
} uart_common;


static const int uartConfig[] = { UARTN_ACTIVE(0), UARTN_ACTIVE(1), UARTN_ACTIVE(2), UARTN_ACTIVE(3),
	UARTN_ACTIVE(4), UARTN_ACTIVE(5), UARTN_ACTIVE(6), UARTN_ACTIVE(7), UARTN_ACTIVE(8), UARTN_ACTIVE(9) };


static const int uartPos[] = { UART1_POS, UART2_POS, UART3_POS, UART4_POS, UART5_POS, UART6_POS,
	UART7_POS, UART8_POS, UART9_POS };


/* clang-format off */
enum { veridr = 0, paramr, globalr, pincfgr, baudr, statr, ctrlr, datar, matchr, modirr, fifor, waterr };
/* clang-format on */


static inline int uart_getRXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 24) & 0xff;
}


static inline int uart_getTXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 8) & 0xff;
}


static int uart_handleIntr(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint32_t status = *(uart->base + statr);

	/* Disable interrupts, enabled in uart_intrThread */
	*(uart->base + ctrlr) &= ~((1 << 27) | (1 << 26) | (1 << 25) | (1 << 23) | (1 << 21));

	/* check for RX overrun */
	if ((status & (1uL << 19u)) != 0u) {
		/* invalidate fifo */
		*(uart->base + fifor) |= 1uL << 14u;
		uart->stat.hw_overrunCntr++;
	}

	/* Process received data */
	while (uart_getRXcount(uart) != 0) {
		uint8_t data = *(uart->base + datar);
		if (lf_fifo_push(&uart->rxFifoCtx, data) == 0) {
			uart->stat.sw_overrunCntr++;
		}
	}

	/* Clear errors: parity, framing, noise, overrun and idle flag */
	*(uart->base + statr) = (status & (0x1fuL << 16));

	return 1;
}


static void uart_intrThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint8_t mask;
	uint8_t c;

	for (;;) {
		/* wait for character or transmit data */
		mutexLock(uart->lock);
		while (lf_fifo_empty(&uart->rxFifoCtx) != 0) {        /* nothing to RX */
			if (libtty_txready(&uart->tty_common)) {          /* something to TX */
				if (uart_getTXcount(uart) < uart->txFifoSz) { /* TX ready */
					break;
				}
				else {
					*(uart->base + ctrlr) |= 1 << 23;
				}
			}

			*(uart->base + ctrlr) |= (1 << 27) | (1 << 26) | (1 << 25) | (1 << 21);

			condWait(uart->cond, uart->lock, 0);
		}

		if ((uart->tty_common.term.c_cflag & CSIZE) == CS7) {
			mask = 0x7f;
		}
		else {
			mask = 0xff;
		}

		mutexUnlock(uart->lock);

		/* RX */
		while (lf_fifo_pop(&uart->rxFifoCtx, &c) != 0) {
			libtty_putchar(&uart->tty_common, c & mask, NULL);
		}

		/* TX */
		while (libtty_txready(&uart->tty_common) && uart_getTXcount(uart) < uart->txFifoSz) {
			*(uart->base + datar) = libtty_getchar(&uart->tty_common, NULL);
		}
	}
}


static void signal_txready(void *_uart)
{
	uart_t *uartptr = (uart_t *)_uart;
	condSignal(uartptr->cond);
}


static void set_cflag(void *_uart, tcflag_t *cflag)
{
	uart_t *uartptr = (uart_t *)_uart;
	uint32_t t;

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	/* CSIZE only CS7 and CS8 (default) is supported */
	if ((*cflag & CSIZE) != CS7) { /* CS8 */
		*cflag &= ~CSIZE;
		*cflag |= CS8;
	}

	/* If parity bit is enabled data character length must be incremented */
	t = *(uartptr->base + ctrlr) & ~(1 << 4 | 1 << 11);
	if ((*cflag & CSIZE) == CS7) {
		if ((*cflag & PARENB) == 0) {
			t |= 1 << 11;
		}
	}
	else if ((*cflag & PARENB) != 0) {
		t |= 1 << 4;
	}
	*(uartptr->base + ctrlr) = t;

	/* parity */
	t = *(uartptr->base + ctrlr) & ~3;
	if ((*cflag & PARENB) != 0) {
		t |= 1 << 1;
		if ((*cflag & PARODD) != 0) {
			t |= 1;
		}
	}
	*(uartptr->base + ctrlr) = t;

	/* stop bits */
	if ((*cflag & CSTOPB) != 0) {
		*(uartptr->base + baudr) |= (1 << 13);
	}
	else {
		*(uartptr->base + baudr) &= ~(1 << 13);
	}

	/* re-enable TX and RX */
	*(uartptr->base + ctrlr) |= (1 << 19) | (1 << 18);
}


static uint32_t calculate_baudrate(uint32_t baud)
{
	uint32_t osr, sbr, bestSbr = 0, bestOsr = 0, bestDiff, t, tDiff;

	if (baud == 0) {
		return 0;
	}

	bestDiff = baud;

	for (osr = 4; osr <= 32; ++osr) {
		/* find sbr value in range between 1 and 8191 */
		sbr = (UART_CLK / (baud * osr)) & 0x1fff;
		sbr = (sbr == 0) ? 1 : sbr;

		/* baud rate difference based on temporary osr and sbr */
		tDiff = UART_CLK / (osr * sbr) - baud;
		t = UART_CLK / (osr * (sbr + 1));

		/* select best values between sbr and sbr+1 */
		if (tDiff > baud - t) {
			tDiff = baud - t;
			sbr += (sbr < 0x1fff);
		}

		if (tDiff <= bestDiff) {
			bestDiff = tDiff;
			bestOsr = osr - 1;
			bestSbr = sbr;
		}
	}

	return (bestOsr << 24) | ((bestOsr <= 6) << 17) | (bestSbr & 0x1fff);
}


static void set_baudrate(void *_uart, speed_t baud)
{
	uint32_t reg = 0, t;
	uart_t *uartptr = (uart_t *)_uart;

	int b = libtty_baudrate_to_int(baud);
	if (b > 0) {
		reg = calculate_baudrate((uint32_t)b);
	}

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	t = *(uartptr->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0x1fff);
	*(uartptr->base + baudr) = t | reg;

	/* re-enable TX and RX */
	*(uartptr->base + ctrlr) |= (1 << 19) | (1 << 18);
}


static void uart_handleMsg(msg_t *msg, msg_rid_t rid, unsigned int major, unsigned int minor)
{
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	uart_t *uart;

	if ((minor >= NELEMS(uartConfig)) || (uartConfig[minor] == 0)) {
		msg->o.err = -EINVAL;
		msgRespond(msg->oid.port, msg, rid);
		return;
	}

	uart = &uart_common.uarts[uartPos[minor]];

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = 0;
			break;

		case mtWrite:
			msg->o.err = libtty_write(&uart->tty_common, msg->i.data, msg->i.size, msg->i.io.mode);
			break;

		case mtRead:
			msg->o.err = libtty_read(&uart->tty_common, msg->o.data, msg->o.size, msg->i.io.mode);
			break;

		case mtGetAttr:
			if (msg->i.attr.type != atPollStatus) {
				msg->o.err = -ENOSYS;
				break;
			}
			msg->o.attr.val = libtty_poll_status(&uart->tty_common);
			msg->o.err = 0;
			break;

		case mtDevCtl:
			in_data = ioctl_unpack(msg, &request, NULL);
			pid = ioctl_getSenderPid(msg);
			err = libtty_ioctl(&uart->tty_common, pid, request, in_data, &out_data);
			ioctl_setResponse(msg, request, err, out_data);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}

	msgRespond(msg->oid.port, msg, rid);
}


static void uart_klogCblk(const char *data, size_t size)
{
	libtty_write(&uart_common.uarts[uartPos[uart_common.ttyminor]].tty_common, data, size, 0);
}


int uart_ttyInit(int ttyno)
{
	char path[24];
	oid_t dev;

	if (ttyno < 0) {
		return -EINVAL;
	}

	snprintf(path, sizeof(path) - 1, "/dev/uart%d", ttyno);
	path[sizeof(path) - 1] = '\0';

	if (lookup(path, NULL, &dev) < 0) {
		return -ENOENT;
	}

	uart_common.ttyminor = ttyno;

	int err = symlink(path, _PATH_TTY);
	if (err < 0) {
		return err;
	}

	err = symlink(path, _PATH_CONSOLE);
	if (err < 0) {
		return err;
	}

	libklog_init(uart_klogCblk);

	return 0;
}


static int uart_init(unsigned int minor)
{
	uint32_t t;
	uart_t *uart;
	libtty_callbacks_t callbacks;
	static const size_t fifoSzLut[] = { 1, 4, 8, 16, 32, 64, 128, 256 };
	static const struct {
		volatile uint32_t *base;
		int tx;
		int rx;
		int txalt;
		int rxalt;
		int irq;
	} info[10] = {
		{ .base = FLEXCOMM0_BASE, .tx = UART0_TX_PIN, .rx = UART0_RX_PIN, .txalt = UART0_TX_ALT, .rxalt = UART0_RX_ALT, .irq = FLEXCOMM0_IRQ },
		{ .base = FLEXCOMM1_BASE, .tx = UART1_TX_PIN, .rx = UART1_RX_PIN, .txalt = UART1_TX_ALT, .rxalt = UART1_RX_ALT, .irq = FLEXCOMM1_IRQ },
		{ .base = FLEXCOMM2_BASE, .tx = UART2_TX_PIN, .rx = UART2_RX_PIN, .txalt = UART2_TX_ALT, .rxalt = UART2_RX_ALT, .irq = FLEXCOMM2_IRQ },
		{ .base = FLEXCOMM3_BASE, .tx = UART3_TX_PIN, .rx = UART3_RX_PIN, .txalt = UART3_TX_ALT, .rxalt = UART3_RX_ALT, .irq = FLEXCOMM3_IRQ },
		{ .base = FLEXCOMM4_BASE, .tx = UART4_TX_PIN, .rx = UART4_RX_PIN, .txalt = UART4_TX_ALT, .rxalt = UART4_RX_ALT, .irq = FLEXCOMM4_IRQ },
		{ .base = FLEXCOMM5_BASE, .tx = UART5_TX_PIN, .rx = UART5_RX_PIN, .txalt = UART5_TX_ALT, .rxalt = UART5_RX_ALT, .irq = FLEXCOMM5_IRQ },
		{ .base = FLEXCOMM6_BASE, .tx = UART6_TX_PIN, .rx = UART6_RX_PIN, .txalt = UART6_TX_ALT, .rxalt = UART6_RX_ALT, .irq = FLEXCOMM6_IRQ },
		{ .base = FLEXCOMM7_BASE, .tx = UART7_TX_PIN, .rx = UART7_RX_PIN, .txalt = UART7_TX_ALT, .rxalt = UART7_RX_ALT, .irq = FLEXCOMM7_IRQ },
		{ .base = FLEXCOMM8_BASE, .tx = UART8_TX_PIN, .rx = UART8_RX_PIN, .txalt = UART8_TX_ALT, .rxalt = UART8_RX_ALT, .irq = FLEXCOMM8_IRQ },
		{ .base = FLEXCOMM9_BASE, .tx = UART9_TX_PIN, .rx = UART9_RX_PIN, .txalt = UART9_TX_ALT, .rxalt = UART9_RX_ALT, .irq = FLEXCOMM9_IRQ },
	};

	static const uint32_t default_baud[] = { UART0_BAUDRATE, UART1_BAUDRATE, UART2_BAUDRATE, UART3_BAUDRATE,
		UART4_BAUDRATE, UART5_BAUDRATE, UART6_BAUDRATE, UART7_BAUDRATE, UART8_BAUDRATE, UART9_BAUDRATE };
	static const uint32_t tty_bufsz[] = { UART0_BUFFSZ, UART1_BUFFSZ, UART2_BUFFSZ, UART3_BUFFSZ,
		UART4_BUFFSZ, UART5_BUFFSZ, UART6_BUFFSZ, UART7_BUFFSZ, UART8_BUFFSZ, UART9_BUFFSZ };

	uart = &uart_common.uarts[uartPos[minor]];
	uart->base = info[minor].base;

	if (condCreate(&uart->cond) < 0) {
		return -1;
	}

	if (mutexCreate(&uart->lock) < 0) {
		resourceDestroy(uart->cond);
		return -1;
	}

	callbacks.arg = uart;
	callbacks.set_baudrate = set_baudrate;
	callbacks.set_cflag = set_cflag;
	callbacks.signal_txready = signal_txready;

	if (libtty_init(&uart->tty_common, &callbacks, tty_bufsz[minor], libtty_int_to_baudrate(default_baud[minor])) < 0) {
		resourceDestroy(uart->cond);
		resourceDestroy(uart->lock);
		return -1;
	}

	lf_fifo_init(&uart->rxFifoCtx, uart->rxFifoData, sizeof(uart->rxFifoData));

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
	t = *(uart->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0x1fff);
	*(uart->base + baudr) = t | calculate_baudrate(default_baud[minor]);

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

	uart->rxFifoSz = fifoSzLut[*(uart->base + fifor) & 0x7];
	uart->txFifoSz = fifoSzLut[(*(uart->base + fifor) >> 4) & 0x7];

	/* Enable overrun, noise, framing error and receiver interrupts */
	*(uart->base + ctrlr) |= (1 << 27) | (1 << 26) | (1 << 25) | (1 << 21);

	/* Enable TX and RX */
	*(uart->base + ctrlr) |= (1 << 19) | (1 << 18);

	beginthread(uart_intrThread, 1, &uart->stack, sizeof(uart->stack), uart);
	interrupt(info[minor].irq, uart_handleIntr, (void *)uart, uart->cond, NULL);

	return 0;
}


static void __attribute__((constructor)) uart_register(void)
{
	if (dev_allocMajor(&uart_common.major) < 0) {
		/* TODO - exit the driver? trigger the reset? report the error? */
		return;
	}

	dev_register(uart_handleMsg, uart_common.major);

	for (unsigned int minor = 0; minor < UART_MAX; ++minor) {
		char fname[8];

		if (uartConfig[minor] == 0) {
			continue;
		}

		if (uart_init(minor) < 0) {
			/* TODO - exit the driver? trigger the reset? report the error? */
			return;
		}

		/* FIXME: Perhaps numerate UNIX way, not by physical device id? */
		snprintf(fname, sizeof(fname) - 1, "uart%u", minor);
		fname[sizeof(fname) - 1] = '\0';

		if (dev_registerFile(fname, uart_common.major, minor) < 0) {
			/* TODO - exit the driver? trigger the reset? report the error? */
			return;
		}
	}
}
