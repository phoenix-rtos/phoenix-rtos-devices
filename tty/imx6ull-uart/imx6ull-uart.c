/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * i.MX6ULL UART driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/msg.h>
#include <errno.h>
#include <string.h>

#include "imx6ull-uart.h"

#include "../../../phoenix-rtos-kernel/include/arch/imx6ull.h"

#define UART_ADDR 0x02020000

uart_t uart = { 0 };

static int uart_write(void *data, size_t size)
{
	int i;

	mutexLock(uart.lock);

	/* wait for previous write to end */
	while (uart.tx_head != uart.tx_tail)
		condWait(uart.tx_cond, uart.lock, 0);

	/* write contents of the buffer */
	for (i = 0; i < size; i++)
		uart.tx_buff[uart.tx_tail++] = *(char *)(data + i);

	/* enable tx ready interrupt */
	*(uart.base + ucr1) |= 0x2000;

	/* wait for this write to end */
	while (uart.tx_head != uart.tx_tail)
		condWait(uart.tx_cond, uart.lock, 0);

	mutexUnlock(uart.lock);

	return i;
}


static int uart_read(void *data, size_t size)
{
	int i;
	int head;

	mutexLock(uart.lock);

	for (i = 0; i < size; i++) {
		/* wait for buffer to fill */
		while (uart.rx_head == uart.rx_tail)
			condWait(uart.rx_cond, uart.lock, 0);
		/* read buffer */
		*(char *)(data + i) = uart.rx_buff[uart.rx_head];
		head = uart.rx_head + 1;
		uart.rx_head = (uart.rx_head & ~0xff) + (head & 0xff);
	}

	if (uart.rx_head == uart.rx_tail)
		uart.ready = 0;

	mutexUnlock(uart.lock);

	return i;
}


void uart_thr(void *arg)
{
	u32 port = (u32)arg;
	msg_t msg;
	unsigned int rid;

	for (;;) {

		if (msgRecv(port, &msg, &rid) < 0) {
			memset(&msg, 0, sizeof(msg));
			msgRespond(port, &msg, rid);
			continue;
		}
		switch (msg.type) {
		case mtOpen:
			break;
		case mtWrite:
			msg.o.io.err = uart_write(msg.i.data, msg.i.size);
			break;
		case mtRead:
			msg.o.io.err = uart_read(msg.o.data, msg.o.size);
			break;
		case mtClose:
			break;
		}

		msgRespond(port, &msg, rid);
	}
	return;
}


static int uart_intr(unsigned int intr, void *data)
{
	/* disable tx ready interrupt */
	*(uart.base + ucr1) &= ~0x2000;

	return uart.cond;
}


static void uart_intrthr(void *arg)
{
	char c;
	int chr = 0;
	int tail;

	for (;;) {

		mutexLock(uart.lock);
		/* wait for character or transmit data */
		if (!(*(uart.base + usr1) & (1 << 9)) && uart.tx_head == uart.tx_tail)
			condWait(uart.cond, uart.lock, 0);

		/* receive */
		if ((*(uart.base + usr1) & (1 << 9))) {
			c = *(uart.base + urxd);

			if (c == 0xd) {
				chr = -1;
				c = 0xa;
				uart.ready = 1;
			}

			if (c == 0x8) {
				c = 0;
				if (chr > 0) {
					c = '\b';
					chr--;
				}
			}

			if (c) {
				chr++;
				uart.rx_buff[uart.rx_tail] = c;
				tail = uart.rx_tail + 1;
				uart.rx_tail = (uart.rx_tail & ~0xff) + (tail & 0xff);

				/* echo */
				if (c == '\b') {
					chr--;
					tail = uart.rx_tail - 2;
					uart.rx_tail = (uart.rx_tail & ~0xff) + (tail & 0xff);
					uart.tx_buff[uart.tx_tail++] = c;
					uart.tx_buff[uart.tx_tail++] = ' ';
				}
				uart.tx_buff[uart.tx_tail++] = c;

				if (uart.ready)
					condSignal(uart.rx_cond);
			}
		}

		/* transmit */
		while (uart.tx_head != uart.tx_tail) {
			if (*(uart.base + usr1) & (1 << 13))
				*(uart.base + utxd) = uart.tx_buff[uart.tx_head++];
		}

		if (uart.tx_head == uart.tx_tail) {
			uart.tx_head = 0;
			uart.tx_tail = 0;
			condSignal(uart.tx_cond);
		}

		mutexUnlock(uart.lock);
	}
}


char __attribute__((aligned(8))) stack[2048];
char __attribute__((aligned(8))) stack0[2048];

void main(void)
{
	u32 port;
	oid_t oid;
	platformctl_t uart_clk;

	if (portCreate(&port) != EOK)
		return;

	if (portRegister(0, "/dev/ttyS0", &oid) != EOK)
		return;

	uart.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, UART_ADDR);

	if (uart.base == NULL)
		return;

	uart_clk.action = pctl_set;
	uart_clk.type = pctl_devclock;
	uart_clk.devclock.dev = pctl_clk_uart1;
	uart_clk.devclock.state = 3;

	platformctl(&uart_clk);
	*(uart.base + ucr2) &= ~0;

	while (!(*(uart.base + ucr2) & 1));

	if (condCreate(&uart.tx_cond) != EOK)
		return;

	if (condCreate(&uart.rx_cond) != EOK)
		return;

	if (mutexCreate(&uart.lock) != EOK)
		return;

	if (condCreate(&uart.cond) != EOK)
		return;

	interrupt(58, uart_intr, NULL, uart.cond, &uart.inth);

	uart.ready = 0;
	/* enable uart and rx ready interrupt */
	*(uart.base + ucr1) |= 0x0201;

	/* soft reset, tx&rx enable, 8bit transmit, no parity (8N1) */
	*(uart.base + ucr2) = 0x4027;
	*(uart.base + ucr3) = 0x704;

	beginthread(uart_intrthr, 3, &stack0, 2048, NULL);
	beginthread(uart_thr, 3, &stack, 2048, (void *)port);
	uart_thr((void *)port);
}
