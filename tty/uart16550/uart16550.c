/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Copyright 2012-2015, 2020-2022 Phoenix Systems
 * Copyright 2001, 2008 Pawel Pisarczyk
 * Authors: Pawel Pisarczyk, Pawel Kolodziej,
 *          Julia Kosowska, Lukasz Kosinski,
 *          Gerard Swiderski et al.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <paths.h>
#include <string.h>

#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libtty.h>
#include <libklog.h>
#include <board_config.h>
#include <posix/utils.h>

#include "uarthw.h"
#include "uart16550.h"

#define KMSG_CTRL_ID 100

#define SW_BUF_SIZE 64

typedef struct {
	uint8_t hwctx[64];
	uint8_t buf[SW_BUF_SIZE];
	volatile unsigned int buf_i;

	unsigned int init;
	unsigned int clk;

	handle_t mutex;
	handle_t intcond;
	handle_t inth;

	oid_t oid;
	libtty_common_t tty;

	char stack[1024] __attribute__((aligned(8)));
} uart_t;


static struct {
	uart_t uarts[4];
	char stack[1024] __attribute__((aligned(8)));
} uart_common;


static uart_t *uart_get(oid_t *oid)
{
	unsigned int i;

	for (i = 0; i < sizeof(uart_common.uarts) / sizeof(uart_common.uarts[0]); i++) {
		if ((uart_common.uarts[i].oid.port == oid->port) && (uart_common.uarts[i].oid.id == oid->id)) {
			return uart_common.uarts + i;
		}
	}

	return NULL;
}


static void set_baudrate(void *_uart, int baud_rate)
{
	uint8_t reg;
	uart_t *uart = (uart_t *)_uart;

	if (baud_rate <= 0) {
		return;
	}

	/* Baud divisor */
	baud_rate = uart->clk / (16 * baud_rate);

	if (baud_rate > UINT16_MAX) {
		return;
	}

	reg = uarthw_read(uart->hwctx, REG_LCR);

	/* Set baud rate */
	uarthw_write(uart->hwctx, REG_LCR, reg | LCR_DLAB);
	uarthw_write(uart->hwctx, REG_LSB, (uint8_t)((unsigned)baud_rate));
	uarthw_write(uart->hwctx, REG_MSB, (uint8_t)((unsigned)baud_rate >> 8));
	uarthw_write(uart->hwctx, REG_LCR, reg & ~LCR_DLAB);
}


static void set_cflag(void *_uart, tcflag_t *cflag)
{
	uart_t *uart = (uart_t *)_uart;
	uint8_t lcr = uarthw_read(uart->hwctx, REG_LCR);

	lcr &= ~((3 << 0) | (1 << 2) | (1 << 3) | (1 << 4));

	/* Character length */
	switch (*cflag & CSIZE) {
		case CS5: lcr |= 0; break;
		case CS6: lcr |= 1; break;
		case CS7: lcr |= 2; break;
		case CS8: /* fallthrough */
		default: lcr |= 3; break;
	}

	/* Parity */
	lcr |= (((*cflag & PARENB) != 0) << 3) | (((*cflag & PARODD) == 0) << 4);

	/* Stop bits */
	lcr |= ((*cflag & CSTOPB) != 0) << 2;

	uarthw_write(uart->hwctx, REG_LCR, lcr);
}


static void signal_txready(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	condSignal(uart->intcond);
}

#ifdef __TARGET_RISCV64
__attribute__((section(".interrupt"), aligned(0x1000))) static int uart_interrupt(unsigned int n, void *arg)
{
	/* RISC-V platform is very special in how it handles memory during interrupts.
	 * Due to this the uart_interrupt function cannot call uarthw_* functions.
	 * Fortunately the UART IRQ on this platform is edge-triggered so we can just
	 * exit interrupt, signal uart->intcond and main thread will handle the rest.
	 */
	uart_t *uart = (uart_t *)arg;
	return uart->intcond;
}
#else
static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	/* Caution: implementation-dependent behavior!
	 * On some UARTs masking interrupts after an interrupt has been asserted
	 * does not de-assert the IRQ line. In this case it is necessary to handle
	 * the interrupt's cause fully within the ISR (e.g. reading the whole FIFO).
	 * On other UARTs masking interrupts de-asserts IRQ and changes the value of IIR.
	 * To handle this case we read IIR before masking interrupts. Note that in this
	 * case the FIFO will not be fully read within the ISR.
	 */
	uint8_t iir = uarthw_read(uart->hwctx, REG_IIR);
	uarthw_write(uart->hwctx, REG_IMR, 0);
	unsigned int i = uart->buf_i;
	do {
		uint8_t intr_type = (iir >> 1) & 0x7;
		if ((intr_type == IIR_CODE_DR) || (intr_type == IIR_CODE_RTO)) {
			uint8_t c = uarthw_read(uart->hwctx, REG_RBR);
			if (i < SW_BUF_SIZE) {
				uart->buf[i] = c;
				i++;
			}
		}
		else if (intr_type == IIR_CODE_LS) {
			uarthw_read(uart->hwctx, REG_LSR);
		}
		else if (intr_type == IIR_CODE_MS) {
			uarthw_read(uart->hwctx, REG_MSR);
		}

		iir = uarthw_read(uart->hwctx, REG_IIR);
	} while ((iir & IIR_IRQPEND) == 0);

	uart->buf_i = i;
	return uart->intcond;
}
#endif


static void uart_intthr(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint8_t target_imr = IMR_DR;

	mutexLock(uart->mutex);
	for (;;) {
		uarthw_write(uart->hwctx, REG_IMR, target_imr);
		condWait(uart->intcond, uart->mutex, 0);
		/* For the following part the interrupt needs to be masked */
		uarthw_write(uart->hwctx, REG_IMR, 0);

		/* Empty received buffer */
		unsigned int buf_i = uart->buf_i;
		for (unsigned int i = 0; i < buf_i; i++) {
			libtty_putchar(&uart->tty, uart->buf[i], NULL);
		}

		uart->buf_i = 0;
		/* Depending on implementation we may have more characters in hardware FIFO */
		while ((uarthw_read(uart->hwctx, REG_LSR) & LSR_DR) != 0) {
			libtty_putchar(&uart->tty, uarthw_read(uart->hwctx, REG_RBR), NULL);
		}

		/* Check for transmit */
		while (1) {
			if (libtty_txready(&uart->tty) != 0) {
				if ((uarthw_read(uart->hwctx, REG_LSR) & LSR_THRE) != 0) {
					uarthw_write(uart->hwctx, REG_THR, libtty_getchar(&uart->tty, NULL));
				}
				else {
					target_imr |= IMR_THRE;
					break;
				}
			}
			else {
				target_imr &= ~IMR_THRE;
				break;
			}
		}
	}
}


static void uart_ioctl(unsigned int port, msg_t *msg)
{
	const void *idata, *odata = NULL;
	oid_t oid = { .port = port };
	uart_t *uart;
	unsigned long req;
	int err;

	idata = ioctl_unpack(msg, &req, &oid.id);

	uart = uart_get(&oid);
	if (uart == NULL) {
		err = -EINVAL;
	}
	else if (req == KIOEN) {
		if ((UART16550_CONSOLE_USER >= 0) && (uart == &uart_common.uarts[UART16550_CONSOLE_USER])) {
			libklog_enable((int)(intptr_t)idata);
			err = EOK;
		}
		else {
			err = -EINVAL;
		}
	}
	else {
		err = libtty_ioctl(&uart->tty, ioctl_getSenderPid(msg), req, idata, &odata);
	}

	ioctl_setResponse(msg, req, err, odata);
}


static void poolthr(void *arg)
{
	unsigned int port = (uintptr_t)arg;
	uart_t *uart;
	msg_rid_t rid;
	msg_t msg;

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
			case mtClose:
				uart = uart_get(&msg.oid);
				if (uart == NULL) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.err = EOK;
				break;

			case mtRead:
				uart = uart_get(&msg.oid);
				if (uart == NULL) {
					msg.o.err = -EINVAL;
				}
				else {
					msg.o.err = libtty_read(&uart->tty, msg.o.data, msg.o.size, msg.i.io.mode);
				}
				break;

			case mtWrite:
				uart = uart_get(&msg.oid);
				if (uart == NULL) {
					msg.o.err = -EINVAL;
				}
				else {
					msg.o.err = libtty_write(&uart->tty, msg.i.data, msg.i.size, msg.i.io.mode);
				}
				break;

			case mtGetAttr:
				uart = uart_get(&msg.oid);
				if ((msg.i.attr.type != atPollStatus) || (uart == NULL)) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.attr.val = libtty_poll_status(&uart->tty);
				msg.o.err = EOK;
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
	libtty_write(&uart_common.uarts[UART16550_CONSOLE_USER].tty, data, size, 0);
}


static void _uart_mkDev(uint32_t port, int isconsole)
{
	char path[12];
	unsigned int i;

	for (i = 0; i < sizeof(uart_common.uarts) / sizeof(uart_common.uarts[0]); i++) {
		if (uart_common.uarts[i].init == 1) {
			uart_common.uarts[i].oid.port = port;
			uart_common.uarts[i].oid.id = (i == UART16550_CONSOLE_USER) ? 0 : i + 1;
			snprintf(path, sizeof(path), "/dev/ttyS%u", i);

			if (create_dev(&uart_common.uarts[i].oid, path) < 0) {
				fprintf(stderr, "uart16550: failed to register %s\n", path);
				return;
			}

			if (i == UART16550_CONSOLE_USER) {
				if (isconsole != 0) {
					libklog_init(uart_klogClbk);
					if (create_dev(&uart_common.uarts[i].oid, _PATH_CONSOLE) < 0) {
						fprintf(stderr, "uart16550: failed to register %s\n", _PATH_CONSOLE);
						return;
					}

					oid_t kmsgctrl = { .port = port, .id = KMSG_CTRL_ID };
					libklog_ctrlRegister(&kmsgctrl);
				}
				else {
					libklog_initNoDev(uart_klogClbk);
				}
			}
		}
	}
}


static int _uart_init(uart_t *uart, unsigned int uartn, unsigned int speed)
{
	unsigned int divisor;
	libtty_callbacks_t callbacks;

	int err = uarthw_init(uartn, uart->hwctx, sizeof(uart->hwctx), &uart->clk);
	if (err < 0) {
		return err;
	}

	divisor = uart->clk / (16 * speed);

	callbacks.arg = uart;
	callbacks.set_baudrate = set_baudrate;
	callbacks.set_cflag = set_cflag;
	callbacks.signal_txready = signal_txready;

	err = libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, speed);
	if (err < 0) {
		return err;
	}

	uart->buf_i = 0;
	condCreate(&uart->intcond);
	mutexCreate(&uart->mutex);

	beginthread(uart_intthr, 1, uart->stack, sizeof(uart->stack), uart);
	interrupt(uarthw_irq(uart->hwctx), uart_interrupt, uart, uart->intcond, &uart->inth);

	/* Set speed (MOD) */
	uarthw_write(uart->hwctx, REG_LCR, LCR_DLAB);
	uarthw_write(uart->hwctx, REG_LSB, divisor);
	uarthw_write(uart->hwctx, REG_MSB, divisor >> 8);

	/* Set data format (MOD) */
	uarthw_write(uart->hwctx, REG_LCR, LCR_D8N1);

	/* Enable and configure FIFOs */
	uarthw_write(uart->hwctx, REG_FCR, 0xa7);

	/* Enable hardware interrupts */
	uarthw_write(uart->hwctx, REG_MCR, MCR_OUT2);

	/* Set interrupt mask */
	uarthw_write(uart->hwctx, REG_IMR, 0);

	return EOK;
}


int main(int argc, char **argv)
{
	unsigned int i;
	uint32_t port;
	int err;

	int isconsole = 1;
	if ((argc == 2) && (strcmp(argv[1], "-n") == 0)) {
		isconsole = 0;
	}
	else if (argc != 1) {
		return -1;
	}

	portCreate(&port);

	for (i = 0; i < sizeof(uart_common.uarts) / sizeof(uart_common.uarts[0]); i++) {
		err = _uart_init(&uart_common.uarts[i], i, UART16550_BAUDRATE);
		if (err < 0) {
			if (err != -ENODEV) {
				fprintf(stderr, "uart16550: failed to init ttyS%u, err: %d\n", i, err);
			}
		}
		else {
			uart_common.uarts[i].init = 1;
		}
	}

	beginthread(poolthr, 4, uart_common.stack, sizeof(uart_common.stack), (void *)(uintptr_t)port);
	_uart_mkDev(port, isconsole);
	poolthr((void *)(uintptr_t)port);

	return EXIT_SUCCESS;
}
