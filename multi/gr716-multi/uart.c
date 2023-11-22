/*
 * Phoenix-RTOS
 *
 * GR716 UART driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>

#include <board_config.h>
#include <libtty.h>
#include <libklog.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <phoenix/ioctl.h>
#include <phoenix/arch/gr716.h>

#include "uart.h"
#include "common.h"
#include "gr716-multi.h"

/* UART registers */
#define UART_DATA   0
#define UART_STATUS 1
#define UART_CTRL   2
#define UART_SCALER 3
#define UART_DEBUG  4

/* UART control bits */
#define STOP_BITS   (1 << 15)
#define RX_FIFO_INT (1 << 10)
#define TX_FIFO_INT (1 << 9)
#define FLOW_CTRL   (1 << 6)
#define PARITY_EN   (1 << 5)
#define PARITY_ODD  (1 << 4)
#define TX_INT      (1 << 3)
#define RX_INT      (1 << 2)
#define TX_EN       (1 << 1)
#define RX_EN       (1 << 0)

/* UART status bits */
#define RX_FIFO_FULL  (1 << 10)
#define TX_FIFO_FULL  (1 << 9)
#define TX_FIFO_EMPTY (1 << 2)
#define TX_SR_EMPTY   (1 << 1)
#define DATA_READY    (1 << 0)

#define UART_CLK SYSCLK_FREQ


typedef struct {
	volatile uint32_t *base;
	oid_t oid;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	libtty_common_t tty;

	uint8_t stack[8 * _PAGE_SIZE] __attribute__((aligned(8)));
} uart_t;


static const struct {
	uint32_t *base;
	unsigned int irq;
	unsigned int cgudev;
	uint8_t txPin;
	uint8_t rxPin;
	uint8_t active;
} info[UART_MAX_CNT] = {
	{ UART0_BASE, UART0_IRQ, cgudev_apbuart0, UART0_TX, UART0_RX, UART0_ACTIVE },
	{ UART1_BASE, UART1_IRQ, cgudev_apbuart1, UART1_TX, UART1_RX, UART1_ACTIVE },
	{ UART2_BASE, UART2_IRQ, cgudev_apbuart2, UART2_TX, UART2_RX, UART2_ACTIVE },
	{ UART3_BASE, UART3_IRQ, cgudev_apbuart3, UART3_TX, UART3_RX, UART3_ACTIVE },
	{ UART4_BASE, UART4_IRQ, cgudev_apbuart4, UART4_TX, UART4_RX, UART4_ACTIVE },
	{ UART5_BASE, UART5_IRQ, cgudev_apbuart5, UART5_TX, UART5_RX, UART5_ACTIVE }
};


static struct {
	uart_t uart[UART_MAX_CNT];
	uint8_t stack[16 * _PAGE_SIZE] __attribute__((aligned(8)));
} uart_common;


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	(void)n;

	/* Check if we have data to receive */
	if ((*(uart->base + UART_CTRL) & RX_INT) != 0 && (*(uart->base + UART_STATUS) & DATA_READY) != 0) {
		/* Disable irq */
		*(uart->base + UART_CTRL) &= ~RX_INT;
	}

	return 1;
}


static void uart_intThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	int wake;

	mutexLock(uart->lock);

	for (;;) {
		while (libtty_txready(&uart->tty) == 0 && (*(uart->base + UART_STATUS) & DATA_READY) == 0) {
			condWait(uart->cond, uart->lock, 0);
		}

		/* Receive data until RX FIFO is not empty */
		while ((*(uart->base + UART_STATUS) & DATA_READY) != 0) {
			libtty_putchar(&uart->tty, (*(uart->base + UART_DATA) & 0xff), NULL);
		}

		/* Transmit data until TX TTY buffer is empty or TX FIFO is full */
		wake = 0;
		while (libtty_txready(&uart->tty) != 0 && (*(uart->base + UART_STATUS) & TX_FIFO_FULL) == 0) {
			*(uart->base + UART_DATA) = libtty_popchar(&uart->tty);
			wake = 1;
		}

		if (wake == 1) {
			libtty_wake_writer(&uart->tty);
		}

		/* If RX int is disabled */
		if ((*(uart->base + UART_CTRL) & RX_INT) == 0) {
			/* Enable RX int */
			*(uart->base + UART_CTRL) |= RX_INT;
		}
	}

	mutexUnlock(uart->lock);
}


static void uart_setBaudrate(void *data, speed_t speed)
{
	uart_t *uart = (uart_t *)data;
	uint32_t scaler = (UART_CLK / (libtty_baudrate_to_int(speed) * 8 + 7));

	*(uart->base + UART_SCALER) = scaler;
}


static void uart_setCFlag(void *data, tcflag_t *cflag)
{
	uart_t *uart = (uart_t *)data;

	/* Parity */
	if ((*cflag & PARENB) != 0) {
		*(uart->base + UART_CTRL) |= (PARITY_EN | PARITY_ODD);
		if ((*cflag & PARODD) == 0) {
			*(uart->base + UART_CTRL) &= ~PARITY_ODD;
		}
	}
	else {
		*(uart->base + UART_CTRL) &= ~PARITY_EN;
	}

	/* Stop bits */
	if ((*cflag & CSTOPB) != 0) {
		*(uart->base + UART_CTRL) |= STOP_BITS;
	}
	else {
		*(uart->base + UART_CTRL) &= ~STOP_BITS;
	}
}


static void uart_signalTXReady(void *data)
{
	uart_t *uart = (uart_t *)data;

	condSignal(uart->cond);
}


static void uart_ioctl(msg_t *msg, int dev)
{
	int err;
	pid_t pid;
	unsigned long req;
	const void *inData, *outData = NULL;

	inData = ioctl_unpack(msg, &req, NULL);
	pid = ioctl_getSenderPid(msg);

	err = libtty_ioctl(&uart_common.uart[dev].tty, pid, req, inData, &outData);

	ioctl_setResponse(msg, req, err, outData);
}


static int uart_cguInit(unsigned int cgudev)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_cguctrl;

	ctl.cguctrl.state = enable;
	ctl.cguctrl.cgu = cgu_primary;
	ctl.cguctrl.cgudev = cgudev;

	return platformctl(&ctl);
}


static int uart_setup(unsigned int n, speed_t baud, int raw)
{
	libtty_callbacks_t callbacks;
	uart_t *uart = &uart_common.uart[n];
	platformctl_t ctl = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iocfg = {
			.opt = 0x1,
			.pin = info[n].rxPin,
			.pullup = 0,
			.pulldn = 0,
		}
	};

	if (platformctl(&ctl) < 0) {
		return -1;
	}

	ctl.iocfg.pin = info[n].txPin;

	if (platformctl(&ctl) < 0) {
		return -1;
	}

	if (uart_cguInit(info[n].cgudev) < 0) {
		return -1;
	}

	uart->base = info[n].base;

	callbacks.arg = uart;
	callbacks.set_cflag = uart_setCFlag;
	callbacks.set_baudrate = uart_setBaudrate;
	callbacks.signal_txready = uart_signalTXReady;

	if (libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, baud) < 0) {
		return -1;
	}

	if (condCreate(&uart->cond) != EOK) {
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		return -1;
	}

	if (mutexCreate(&uart->lock) != EOK) {
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		resourceDestroy(uart->cond);
		return -1;
	}

	/* Set raw mode */
	if (raw == 1) {
		libtty_set_mode_raw(&uart->tty);
	}

	/* normal mode, 1 stop bit, no parity, 8 bits */
	uart_setCFlag(uart, &uart->tty.term.c_cflag);

	uart->tty.term.c_ispeed = uart->tty.term.c_ospeed = baud;
	uart_setBaudrate(uart, baud);
	*(uart->base + UART_CTRL) = RX_INT | RX_EN | TX_EN;

	beginthread(uart_intThread, 2, &uart->stack, sizeof(uart->stack), (void *)uart);
	interrupt(info[n].irq, uart_interrupt, uart, uart->cond, &uart->inth);

	return EOK;
}


void uart_handleMsg(msg_t *msg, int dev)
{
	dev -= id_uart0;

	if (info[dev].active == 0) {
		msg->o.io.err = -EINVAL;
		return;
	}

	switch (msg->type) {
		case mtOpen:
			msg->o.io.err = EOK;
			break;

		case mtClose:
			msg->o.io.err = EOK;
			break;

		case mtWrite:
			msg->o.io.err = libtty_write(&uart_common.uart[dev].tty, msg->i.data, msg->i.size, msg->i.io.mode);
			break;

		case mtRead:
			msg->o.io.err = libtty_read(&uart_common.uart[dev].tty, msg->o.data, msg->o.size, msg->i.io.mode);
			break;

		case mtGetAttr:
			if (msg->i.attr.type == atPollStatus) {
				msg->o.attr.val = libtty_poll_status(&uart_common.uart[dev].tty);
				msg->o.attr.err = EOK;
				break;
			}

			msg->o.attr.err = -EINVAL;
			break;

		case mtDevCtl:
			uart_ioctl(msg, dev);
			break;
	}
}


void uart_klogClbk(const char *data, size_t size)
{
	libtty_write(&uart_common.uart[UART_CONSOLE_USER].tty, data, size, 0);
}


int uart_init(void)
{
	int raw = 0;
	speed_t baud = B115200;

	for (int n = 0; n < UART_MAX_CNT; n++) {
		if (info[n].active == 0) {
			continue;
		}

		if (uart_setup(n, baud, raw) < 0) {
			return -1;
		}
	}

	return 0;
}
