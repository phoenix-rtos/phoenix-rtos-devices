/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * UART 16550 driver for PC
 *
 * Copyright 2012-2015 Phoenix Systems
 * Copyright 2001, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej
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
#include <sys/io.h>
#include <sys/file.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/mman.h>

#include <libtty.h>
#include "pc-uart.h"


typedef struct {
	void *base;
	unsigned int irq;

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
/*
	uint8_t iir;
	if ((iir = inb(uart->base + REG_IIR)) & IIR_IRQPEND)
		return 0;
*/
	return uart->intcond;
}


static void set_baudrate(void *_uart, speed_t baud)
{
	/* TODO */
}


static void set_cflag(void *_uart, tcflag_t *cflag)
{
	/* TODO */
}


static void signal_txready(void *_uart)
{
	uart_t *uart = _uart;
	outb(uart->base + REG_IMR, IMR_THRE | IMR_DR);
	condSignal(uart->intcond);
}


void uart_intthr(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint8_t iir, lsr;
	char c;

	mutexLock(uart->mutex);
	for (;;) {
		while ((iir = inb(uart->base + REG_IIR)) & IIR_IRQPEND)
			condWait(uart->intcond, uart->mutex, 0);

		/* Receive */
		if ((iir & IIR_DR) == IIR_DR) {
			while (1) {
				lsr = inb(uart->base + REG_LSR);

				if ((lsr & 1) == 0)
					break;

				c = inb(uart->base + REG_RBR);
				libtty_putchar(&uart->tty, c, NULL);
			}
		}

		/* Transmit */
		if ((iir & IIR_THRE) == IIR_THRE) {
			if (libtty_txready(&uart->tty)) {
				c = libtty_getchar(&uart->tty, NULL);
				outb(uart->base + REG_THR, c);
			}
			else {
				outb(uart->base + REG_IMR, IMR_DR);
			}
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

	for (i = 0; i < sizeof(uarts) / sizeof(uart_t); i++) {
		if ((uarts[i]->oid.id == oid->id) && (uarts[i]->oid.port == oid->port))
			return i;
	}
	return 0;
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


int _uart_init(void *base, unsigned int irq, unsigned int speed, uart_t **uart)
{
	libtty_callbacks_t callbacks;

	/* Test if device exist */
	if (inb(base + REG_IIR) == 0xff)
		return -ENOENT;

	printf("pc-uart: Detected interface on 0x%x irq=%d\n", (uint32_t)base, irq);

	/* Allocate and map memory for driver structures */
	if ((*uart = malloc(sizeof(uart_t))) == NULL)
		return -ENOMEM;

	memset((*uart), 0, sizeof(uart_t));

	callbacks.arg = *uart;
	callbacks.set_baudrate = set_baudrate;
	callbacks.set_cflag = set_cflag;
	callbacks.signal_txready = signal_txready;

	libtty_init(&(*uart)->tty, &callbacks, _PAGE_SIZE);

	(*uart)->base = base;
	(*uart)->irq = irq;

	condCreate(&(*uart)->intcond);
	mutexCreate(&(*uart)->mutex);

	interrupt(irq, uart_interrupt, (*uart), (*uart)->intcond, &(*uart)->inth);

	uint8_t *stack;
	stack = (uint8_t *)malloc(2 * 4096);

//stack = mmap((void *)0, 4096 * 2, 0, 0, NULL, 0);
//stack += 0x10;

//printf("user stack=%p\n", stack);

//for (;;);

	beginthread(uart_intthr, 1, stack, 2 * 4096, (void *)*uart);

//for (;;);

	/* Set speed (MOD) */
	outb(base + REG_LCR, LCR_DLAB);
	outb(base + REG_LSB, speed);
	outb(base + REG_MSB, 0);

	/* Set data format (MOD) */
	outb(base + REG_LCR, LCR_D8N1);

	/* Enable FIFO - this is required for Transmeta Crusoe (MOD) */
	outb(base + 2, 0x01);

	/* Enable hardware interrupts */
	outb(base + REG_MCR, MCR_OUT2);

	/* Set interrupt mask */
	outb(base + REG_IMR, IMR_DR);

	return EOK;
}


void poolthr(void *arg)
{
	uint32_t port = (uint32_t)arg;
	msg_t msg;
	unsigned long rid;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtOpen:
			break;
		case mtWrite:
			msg.o.io.err = uart_write(uart_get(&msg.i.io.oid), msg.i.size, msg.i.data, msg.i.io.mode);
			break;
		case mtRead:
			msg.o.io.err = uart_read(uart_get(&msg.i.io.oid), msg.o.size, msg.o.data, msg.i.io.mode);
			break;
		case mtClose:
			break;
		case mtGetAttr:
			if (msg.i.attr.type == atPollStatus)
				msg.o.attr.val = uart_poll_status(uart_get(&msg.i.io.oid));
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
	void *base = (void *)0x3f8;
	unsigned int n = 4;
	uint32_t port;

	printf("pc-uart: Initializing UART 16550 driver %s\n", "");

	_uart_init(base, n, BPS_115200, &uarts[0]);

	portCreate(&port);
	if (portRegister(port, "/dev/ttyS0", &uarts[0]->oid) < 0) {
		printf("Can't register port %d\n", port);
//for (;;);
		return -1;
	}
	/*if (portRegister(port, "/dev/ttyS1", &uarts[1]->oid) < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}
	if (portRegister(port, "/dev/ttyS2", &uarts[2]->oid) < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}
	if (portRegister(port, "/dev/ttyS3", &uarts[3]->oid) < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}*/

	void *stack = malloc(2048);

	/* Run threads */
	beginthread(poolthr, 1, stack, 2048, (void *)port);
	poolthr((void *)port);

	return 0;
}
