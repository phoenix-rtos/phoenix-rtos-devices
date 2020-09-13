/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Copyright 2012-2015, 2020 Phoenix Systems
 * Copyright 2001, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej, Julia Kosowska
 *
 * %LICENSE%
 */

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/file.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/mman.h>

#include <libtty.h>
#include "uart16550.h"
#include "uarthw.h"


#define DRIVER "uart16550"


typedef struct {
	uint8_t hwctx[64];

	handle_t mutex;
	handle_t intcond;
	handle_t inth;

	oid_t oid;
	libtty_common_t tty;
} uart_t;


static uart_t *uarts[4];


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	return uart->intcond;
}


static void uart_setbaudrate(void *_uart, speed_t baud)
{
	/* TODO */
}


static void uart_setcflag(void *_uart, tcflag_t *cflag)
{
	/* TODO */
}


static void uart_signaltxready(void *_uart)
{
	uart_t *uart = _uart;

	uarthw_write(uart->hwctx, REG_IMR, IMR_THRE | IMR_DR);
	condSignal(uart->intcond);
}


void uart_intthr(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint64_t iir, lsr;
	char c;

	mutexLock(uart->mutex);
	for (;;) {

		while ((iir = uarthw_read(uart->hwctx, REG_IIR)) & IIR_IRQPEND)
			condWait(uart->intcond, uart->mutex, 0);

		/* Receive */
		if ((iir & IIR_DR) == IIR_DR) {

			while (1) {
				lsr = uarthw_read(uart->hwctx, REG_LSR);

				if ((lsr & 1) == 0)
					break;

				c = uarthw_read(uart->hwctx, REG_RBR);

				libtty_putchar(&uart->tty, c, NULL);
			}
		}

		/* Transmit */
		if ((iir & IIR_THRE) == IIR_THRE) {
			
			if (libtty_txready(&uart->tty)) {
				c = libtty_getchar(&uart->tty, NULL);
				uarthw_write(uart->hwctx, REG_THR, c);
			}
			else
				uarthw_write(uart->hwctx, REG_IMR, IMR_DR);
		}
	}
}


static int uart_write(uint8_t d, size_t len, char *buff, int mode)
{
	uart_t *serial;

	if (d >= sizeof(uarts) / sizeof(uart_t *))
		return -EINVAL;

	if ((serial = uarts[d]) == NULL)
		return -ENOENT;

	if (!len)
		return 0;

	return libtty_write(&serial->tty, buff, len, mode);
}


static int uart_read(uint8_t d, size_t len, char *buff, int mode)
{
	uart_t *serial;

	if (d >= sizeof(uarts) / sizeof(uart_t *))
		return -EINVAL;

	if ((serial = uarts[d]) == NULL)
		return -ENOENT;

	return libtty_read(&serial->tty, buff, len, mode);
}


static int uart_poll_status(uint8_t d)
{
	uart_t *serial;

	if (d >= sizeof(uarts) / sizeof(uart_t *))
		return POLLNVAL;

	if ((serial = uarts[d]) == NULL)
		return POLLNVAL;

	return libtty_poll_status(&serial->tty);
}


uint8_t uart_get(oid_t *oid)
{
	unsigned int i;

	for (i = 0; i < sizeof(uarts) / sizeof(uart_t *); i++) {
		if ((uarts[i]->oid.id == oid->id) && (uarts[i]->oid.port == oid->port))
			return i;
	}
	return -ENOENT;
}


static void uart_ioctl(unsigned port, msg_t *msg)
{
	uart_t *serial;
	unsigned long request;
	const void *in_data, *out_data;
	pid_t pid;
	int err;
	oid_t oid;
	uint8_t d;

	oid.port = port;

	in_data = ioctl_unpack(msg, &request, &oid.id);
	out_data = NULL;
	pid = ioctl_getSenderPid(msg);

	d = uart_get(&oid);

	if (d >= sizeof(uarts) / sizeof(uart_t *))
		err = -EINVAL;

	else if ((serial = uarts[d]) == NULL)
		err = -ENOENT;

	else
		err = libtty_ioctl(&serial->tty, pid, request, in_data, &out_data);

	ioctl_setResponse(msg, request, err, out_data);
}


int _uart_init(unsigned int uartn, unsigned int speed, uart_t **uart)
{
	uint8_t buff[64];
	char s[64];
	libtty_callbacks_t callbacks;
	uint8_t *stack;

	if (uarthw_init(uartn, buff, sizeof(buff)) < 0)
		return -ENOENT;

	printf(DRIVER ": Detected uart interface (%s)\n", uarthw_dump(buff, s, sizeof(s)));

	/* Allocate and map memory for driver structures */
	if ((*uart = malloc(sizeof(uart_t))) == NULL) {
		fprintf(stderr, DRIVER ": Out of memory!\n");
		return -ENOMEM;
	}

	/* Initialize uart strcture */
	memset((*uart), 0, sizeof(uart_t));
	memcpy((*uart)->hwctx, buff, sizeof(buff));

	callbacks.arg = *uart;
	callbacks.set_baudrate = uart_setbaudrate;
	callbacks.set_cflag = uart_setcflag;
	callbacks.signal_txready = uart_signaltxready;

	libtty_init(&(*uart)->tty, &callbacks, _PAGE_SIZE);

	condCreate(&(*uart)->intcond);
	mutexCreate(&(*uart)->mutex);

	/* Install interrupt */
	if ((stack = (uint8_t *)malloc(2 * 4096)) == NULL) {
		fprintf(stderr, DRIVER ": Out of memory!\n");
		return -ENOMEM;
	}
	beginthread(uart_intthr, 1, stack, 2 * 4096, (void *)*uart);

	interrupt(uarthw_irq((*uart)->hwctx), uart_interrupt, (*uart), (*uart)->intcond, &(*uart)->inth);

	/* Enable FIFO */
	uarthw_write((*uart)->hwctx, 2, 0x01);

	/* Enable hardware interrupts */
	uarthw_write((*uart)->hwctx, REG_MCR, MCR_OUT2);

	/* Set interrupt mask */
	uarthw_write((*uart)->hwctx, REG_IMR, IMR_DR);

	/* Set speed (MOD) */
	uarthw_write((*uart)->hwctx, REG_LCR, LCR_DLAB);
	uarthw_write((*uart)->hwctx, REG_LSB, speed);
	uarthw_write((*uart)->hwctx, REG_MSB, 0);

	/* Set data format (MOD) */
	uarthw_write((*uart)->hwctx, REG_LCR, LCR_D8N1);

	return EOK;
}


void poolthr(void *arg)
{
	uint32_t port = (uint32_t)(unsigned long)arg;
	msg_t msg;
	unsigned long rid, d;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtOpen:
			break;
		case mtWrite: {
			if ((d = uart_get(&msg.i.io.oid)) < 0) {
				msg.o.io.err = -ENOENT;
				break;
			}
			msg.o.io.err = uart_write(d, msg.i.size, msg.i.data, msg.i.io.mode);
		}
			break;
		case mtRead:
			if ((d = uart_get(&msg.i.io.oid)) < 0) {
				msg.o.io.err = -ENOENT;
				break;
			}

			msg.o.io.err = uart_read(d, msg.o.size, msg.o.data, msg.i.io.mode);
			break;
		case mtClose:
			break;
		case mtGetAttr:
			if (msg.i.attr.type == atPollStatus) {
				if ((d = uart_get(&msg.i.io.oid)) < 0) {
					msg.o.io.err = -ENOENT;
					break;
				}
				msg.o.attr.val = uart_poll_status(d);
			}
			else
				msg.o.attr.val = -EINVAL;
			break;
		case mtDevCtl:
			uart_ioctl(port, &msg);
			break;
		}

		msgRespond(port, &msg, rid);
	}
}


int main(void)
{
	unsigned int n;
	uint32_t port;
	void *stack;

	printf(DRIVER ": Initializing UART 16550 driver\n");

	portCreate(&port);

	for (n = 0; n < 4; n++) {
		if (_uart_init(n, B115200, &uarts[n]) < 0)
			continue;
		
		if (portRegister(port, "/dev/ttyS0", &uarts[n]->oid) < 0) {
			fprintf(stderr, DRIVER ": Can't register port %d\n", port);
			return -1;
		}
	}

	/* Run driver threads for message processing */
	if ((stack = malloc(2048)) == NULL) {
		fprintf(stderr, DRIVER ": Out of memory!\n");
		return -ENOMEM;
	}
	beginthread(poolthr, 4, stack, 2048, (void *)(unsigned long)port);
	poolthr((void *)(unsigned long)port);

	return 0;
}
