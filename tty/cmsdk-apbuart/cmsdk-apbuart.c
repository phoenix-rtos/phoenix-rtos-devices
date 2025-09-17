/*
 * Phoenix-RTOS
 *
 * ARM CMSDK APB-UART driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <paths.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <board_config.h>
#include <libtty.h>
#include <libtty-lf-fifo.h>
#include <libklog.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <posix/utils.h>

#include <phoenix/ioctl.h>


#define UART_STACKSZ (1024)

/* UART state bits */
#define TX_BUF_FULL (1 << 0)
#define RX_BUF_FULL (1 << 1)

/* UART control bits */
#define TX_EN     (1 << 0)
#define RX_EN     (1 << 1)
#define TX_INT_EN (1 << 2)
#define RX_INT_EN (1 << 3)

/* UART intstatus bits */
#define TX_INT (1 << 0)
#define RX_INT (1 << 1)

#define UART_CLK SYSCLK_FREQ

#define KMSG_CTRL_ID 100

#ifndef UART_RXFIFOSIZE
#define UART_RXFIFOSIZE 128
#endif


/* UART registers */
/* clang-format off */
enum { data = 0, state, ctrl, intstatus, bauddiv };
/* clang-format on */


typedef struct {
	volatile uint32_t *base;
	oid_t oid;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	libtty_common_t tty;

	/* fifo between irq and thread */
	lf_fifo_t rxFifoCtx;
	uint8_t rxFifoData[UART_RXFIFOSIZE];

	uint8_t stack[UART_STACKSZ] __attribute__((aligned(8)));
} uart_t;


static const struct {
	volatile uint32_t *base;
	unsigned int rxirq;
} info[] = {
	{ .base = UART0_BASE, .rxirq = UART0_RX_IRQ },
	{ .base = UART1_BASE, .rxirq = UART1_RX_IRQ },
	{ .base = UART2_BASE, .rxirq = UART2_RX_IRQ },
	{ .base = UART3_BASE, .rxirq = UART3_RX_IRQ },
	{ .base = UART4_BASE, .rxirq = UART4_RX_IRQ },
	{ .base = UART5_BASE, .rxirq = UART5_RX_IRQ },
};


static struct {
	uart_t uart;
	uint8_t stack[UART_STACKSZ] __attribute__((aligned(8)));
} uart_common;


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint32_t status;
	uint8_t c;

	status = *(uart->base + state);

	if ((status & RX_INT) != 0) {
		while ((*(uart->base + state) & RX_BUF_FULL) != 0) {
			c = *(uart->base + data) & 0xff;
			lf_fifo_push(&uart->rxFifoCtx, c);
		}
		*(uart->base + intstatus) = RX_INT;

		return 1;
	}

	return -1;
}


static void uart_intThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;

	for (;;) {
		while (lf_fifo_empty(&uart->rxFifoCtx) != 0) { /* nothing to RX */
			if (libtty_txready(&uart->tty) != 0) {     /* something to TX */
				if ((*(uart->base + state) & TX_BUF_FULL) == 0) {
					break;
				}
			}
			mutexLock(uart->lock);
			condWait(uart->cond, uart->lock, 0);
			mutexUnlock(uart->lock);
		}

		/* RX */
		uint8_t c;
		while (lf_fifo_pop(&uart->rxFifoCtx, &c) != 0) {
			libtty_putchar(&uart->tty, c, NULL);
		}

		/* TX */
		bool wake = false;
		while ((libtty_txready(&uart->tty) != 0) && ((*(uart->base + state) & TX_BUF_FULL) == 0)) {
			*(uart->base + data) = libtty_popchar(&uart->tty);
			wake = true;
		}

		if (wake) {
			libtty_wake_writer(&uart->tty);
		}
	}

	__builtin_unreachable();
}


static void uart_setBaudrate(void *data, int speed)
{
	uart_t *uart = (uart_t *)data;
	uint32_t div = UART_CLK / speed;

	*(uart->base + bauddiv) = div;
}


static void uart_signalTXReady(void *data)
{
	uart_t *uart = (uart_t *)data;

	condSignal(uart->cond);
}


static void uart_ioctl(unsigned port, msg_t *msg)
{
	unsigned long req;
	const void *inData = ioctl_unpack(msg, &req, NULL);
	pid_t pid = ioctl_getSenderPid(msg);

	const void *outData = NULL;
	int err = libtty_ioctl(&uart_common.uart.tty, pid, req, inData, &outData);

	ioctl_setResponse(msg, req, err, outData);
}


static void uart_dispatchMsg(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	uint32_t port = uart_common.uart.oid.port;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0) {
			continue;
		}

		if (libklog_ctrlHandle(port, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.err = 0;
				break;

			case mtClose:
				msg.o.err = 0;
				break;

			case mtWrite:
				msg.o.err = libtty_write(&uart_common.uart.tty, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtRead:
				msg.o.err = libtty_read(&uart_common.uart.tty, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atPollStatus) {
					msg.o.attr.val = libtty_poll_status(&uart_common.uart.tty);
					msg.o.err = 0;
					break;
				}

				msg.o.err = -ENOSYS;
				break;

			case mtDevCtl:
				uart_ioctl(port, &msg);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void uart_klogClbk(const char *data, size_t size)
{
	libtty_write(&uart_common.uart.tty, data, size, 0);
}


static void uart_mkDev(unsigned int id)
{
	char path[7];

	snprintf(path, sizeof(path), "uart%u", id);
	if (create_dev(&uart_common.uart.oid, path) < 0) {
		debug("cmsdk-apbuart: Cannot create device file.\n");
		return;
	}

	if (id == UART_CONSOLE_USER) {
		oid_t kmsg = { .port = uart_common.uart.oid.port, .id = KMSG_CTRL_ID };

		libklog_init(uart_klogClbk);

		if (create_dev(&uart_common.uart.oid, _PATH_CONSOLE) < 0) {
			debug("cmsdk-apbuart: Cannot create device file.\n");
		}

		if (libklog_ctrlRegister(&kmsg) < 0) {
			debug("cmsdk-apbuart: Cannot create kmsg control device file.\n");
		}
	}
}


static int uart_init(unsigned int n, int baud, int raw)
{
	uart_t *uart = &uart_common.uart;
	uart->base = info[n].base;

	libtty_callbacks_t callbacks = {
		.arg = uart,
		.set_cflag = NULL, /* Not supported */
		.set_baudrate = uart_setBaudrate,
		.signal_txready = uart_signalTXReady,
	};
	if (libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, baud) < 0) {
		return -1;
	}

	if (condCreate(&uart->cond) != 0) {
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		return -1;
	}

	if (mutexCreate(&uart->lock) != 0) {
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		resourceDestroy(uart->cond);
		return -1;
	}

	lf_fifo_init(&uart->rxFifoCtx, uart->rxFifoData, sizeof(uart->rxFifoData));

	/* Set raw mode */
	if (raw == 1) {
		libtty_set_mode_raw(&uart->tty);
	}

	*(uart->base + ctrl) = 0;

	uart->tty.term.c_ispeed = uart->tty.term.c_ospeed = baud;
	uart_setBaudrate(uart, baud);
	*(uart->base + ctrl) = TX_EN | RX_EN | RX_INT_EN;

	beginthread(uart_intThread, 2, &uart->stack, sizeof(uart->stack), (void *)uart);
	interrupt(info[n].rxirq, uart_interrupt, uart, uart->cond, &uart->inth);

	return 0;
}


static void uart_usage(const char *progname)
{
	printf("Usage: %s [options]\n", progname);
	printf("Options:\n");
	printf("\t-b <baudrate>   - baudrate (default 115200)\n");
	printf("\t-n <id>         - uart controller id\n");
	printf("\t-r              - set raw mode (default cooked)\n");
	printf("\t-h              - print this message\n");
}


int main(int argc, char **argv)
{
	int uartn = UART_CONSOLE_USER;
	int baud = 115200;
	int raw = 0;

	if (argc > 1) {
		for (;;) {
			int c = getopt(argc, argv, "n:b:rh");
			if (c == -1) {
				break;
			}

			switch (c) {
				case 'b':
					baud = atoi(optarg);
					if (baud <= 0) {
						debug("cmsdk-apbuart: wrong baudrate value\n");
						return EXIT_FAILURE;
					}
					break;

				case 'n':
					uartn = atoi(optarg);
					if ((uartn >= UART_MAX_CNT) || (uartn < 0)) {
						debug("cmsdk-apbuart: wrong uart ID\n");
						return EXIT_FAILURE;
					}
					break;

				case 'r':
					raw = 1;
					break;

				case 'h':
					uart_usage(argv[0]);
					return EXIT_SUCCESS;

				default:
					uart_usage(argv[0]);
					return EXIT_FAILURE;
			}
		}
	}

	if (portCreate(&uart_common.uart.oid.port) < 0) {
		debug("cmsdk-apbuart: cannot create port\n");
		return EXIT_FAILURE;
	}

	if (uart_init(uartn, baud, raw) < 0) {
		debug("cmsdk-apbuart: cannot initialize uart\n");
		return EXIT_FAILURE;
	}

	uart_mkDev(uartn);
	beginthread(uart_dispatchMsg, 3, uart_common.stack, sizeof(uart_common.stack), NULL);
	uart_dispatchMsg(NULL);

	return EXIT_SUCCESS;
}
