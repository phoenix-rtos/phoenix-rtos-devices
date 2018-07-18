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
#include <errno.h>
#include <poll.h>
#include <unistd.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/file.h>
#include <sys/platform.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/debug.h>

#include "imx6ull-uart.h"

#include "../../../phoenix-rtos-kernel/include/arch/imx6ull.h"

u32 uart_addr[8] = { 0x02020000, 0x021E8000, 0x021EC000, 0x021F0000,
	0x021F4000, 0x021FC000, 0x02018000, 0x02284000 };

u32 uart_pctl_clk[8] = { pctl_clk_uart1, pctl_clk_uart2, pctl_clk_uart3, pctl_clk_uart4,
	pctl_clk_uart5, pctl_clk_uart6, pctl_clk_uart7, pctl_clk_uart8 };

typedef struct {
	int pctl;
	char val;
} uart_pctl_t;

uart_pctl_t uart_pctl_mux[8][4] = {
	{ { pctl_mux_uart1_cts, 0 }, { pctl_mux_uart1_rts,  0 }, { pctl_mux_uart1_rx,  0 }, { pctl_mux_uart1_tx,  0 } },
	{ { pctl_mux_uart2_cts, 0 }, { pctl_mux_uart2_rts,  0 }, { pctl_mux_uart2_rx,  0 }, { pctl_mux_uart2_tx,  0 } },
	{ { pctl_mux_uart3_cts, 0 }, { pctl_mux_uart3_rts,  0 }, { pctl_mux_uart3_rx,  0 }, { pctl_mux_uart3_tx,  0 } },
	{ { pctl_mux_lcd_hsync, 2 }, { pctl_mux_lcd_vsync,  2 }, { pctl_mux_uart4_rx,  0 }, { pctl_mux_uart4_tx,  0 } },
	{ { pctl_mux_gpio1_09,  8 }, { pctl_mux_gpio1_08,   8 }, { pctl_mux_uart5_rx,  0 }, { pctl_mux_uart5_tx,  0 } },
	{ { pctl_mux_enet1_tx1, 1 }, { pctl_mux_enet1_txen, 1 }, { pctl_mux_enet2_rx1, 1 }, { pctl_mux_enet2_rx0, 1 } },
	{ { pctl_mux_lcd_d6,    1 }, { pctl_mux_lcd_d7,     1 }, { pctl_mux_lcd_d17,   1 }, { pctl_mux_lcd_d16,   1 } },
	{ { pctl_mux_lcd_d4,    1 }, { pctl_mux_lcd_d5,     1 }, { pctl_mux_lcd_d21,   1 }, { pctl_mux_lcd_d20,   1 } },
};

uart_pctl_t uart_pctl_isel[8][2] = {
	{ { pctl_isel_uart1_rts, 3 }, { pctl_isel_uart1_rx, 3 } },
	{ { pctl_isel_uart2_rts, 1 }, { pctl_isel_uart2_rx, 1 } },
	{ { pctl_isel_uart3_rts, 1 }, { pctl_isel_uart3_rx, 1 } },
	{ { pctl_isel_uart4_rts, 3 }, { pctl_isel_uart4_rx, 1 } },
	{ { pctl_isel_uart5_rts, 1 }, { pctl_isel_uart5_rx, 7 } },
	{ { pctl_isel_uart6_rts, 3 }, { pctl_isel_uart6_rx, 2 } },
	{ { pctl_isel_uart7_rts, 3 }, { pctl_isel_uart7_rx, 3 } },
	{ { pctl_isel_uart8_rts, 3 }, { pctl_isel_uart8_rx, 3 } },
};

uart_t uart = { 0 };

#define MODULE_CLK 20000000

#define IS_SYNC (uart.flags & FL_SYNC)
#define IS_COOL (uart.flags & FL_COOL)
#define IS_TTY (uart.mode & MODE_TTY)

static inline int tx_put(char data) {

	int tail;

	if (uart.tx_head == uart.tx_tail && (*(uart.base + usr1) & (1 << 13))) {
		*(uart.base + utxd) = data;
		return 0;
	}

	if (uart.tx_full)
		return 1;

	uart.tx_buff[uart.tx_tail] = data;
	tail = uart.tx_tail + 1;
	uart.tx_tail = (uart.tx_tail & ~0xff) + (tail & 0xff);

	if (uart.tx_head == uart.tx_tail) {
		uart.tx_full = 1;
		uart.tx_tail = tail - 1;
		return 1;
	}

	return 0;
}

static int uart_write(void *data, size_t size)
{
	int i;

	mutexLock(uart.lock);

	while (IS_SYNC && uart.tx_head != uart.tx_tail)
		condWait(uart.tx_cond, uart.lock, 0);

	/* write contents of the buffer */
	for (i = 0; i < size; i++) {

		if (tx_put(*((char *)data + i))) {
			if (!IS_COOL)
				break;
			else {
				*(uart.base + ucr1) |= 0x2000;
				condWait(uart.tx_cond, uart.lock, 0);
			}
		}
	}

	/* enable tx ready interrupt */
	*(uart.base + ucr1) |= 0x2000;

	while (IS_SYNC && uart.tx_head != uart.tx_tail)
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
		if (!IS_SYNC && uart.rx_head == uart.rx_tail) {
			mutexUnlock(uart.lock);
			return i;
		}

		while (IS_SYNC && uart.rx_head == uart.rx_tail)
			condWait(uart.rx_cond, uart.lock, 0);

		/* read buffer */
		*(char *)(data + i) = uart.rx_buff[uart.rx_head];
		head = uart.rx_head + 1;
		uart.rx_head = (uart.rx_head & ~0xff) + (head & 0xff);

		if (IS_TTY && *(char *)(data + i) == 0xa) {
			i++;
			break;
		}
	}

	if (IS_TTY && uart.rx_head == uart.rx_tail)
		uart.ready = 0;

	mutexUnlock(uart.lock);

	return i;
}


static int uart_poll_status()
{
	int revents = 0;

	mutexLock(uart.lock);

	if (uart.rx_head != uart.rx_tail)
		revents |= POLLIN|POLLRDNORM;
	if (uart.tx_head == uart.tx_tail)
		revents |= POLLOUT|POLLWRNORM;

	mutexUnlock(uart.lock);

	return revents;
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
		case mtGetAttr:
			if (msg.i.attr.type == atPollStatus)
				msg.o.attr.val = uart_poll_status();
			else
				msg.o.attr.val = -EINVAL;
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
	int head;

	mutexLock(uart.lock);
	for (;;) {

		/* wait for character or transmit data */
		while (!(*(uart.base + usr1) & (1 << 9)) && (!(*(uart.base + usr1) & (1 << 13)) || uart.tx_head == uart.tx_tail))
			condWait(uart.cond, uart.lock, 0);

		if (IS_TTY) {
			/* receive */
			while ((*(uart.base + usr1) & (1 << 9))) {
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
				}

				/* echo */
				if (c == '\b') {
					chr--;
					tail = uart.rx_tail - 2;
					uart.rx_tail = (uart.rx_tail & ~0xff) + (tail & 0xff);
					tx_put(c);
					tx_put(' ');
				}

				tx_put(c);
				if (IS_SYNC && uart.ready)
					condSignal(uart.rx_cond);
			}
		} else {
			while ((*(uart.base + usr1) & (1 << 9))) {
				uart.rx_buff[uart.rx_tail] = *(uart.base + urxd);
				tail = uart.rx_tail + 1;
				uart.rx_tail = (uart.rx_tail & ~0xff) + (tail & 0xff);
			}

			if (IS_SYNC)
				condSignal(uart.rx_cond);
		}

		/* transmit */
		*(uart.base + ufcr) |= 0x1f << 10;

		while (uart.tx_head != uart.tx_tail && (*(uart.base + usr1) & (1 << 13))) {
			*(uart.base + utxd) = uart.tx_buff[uart.tx_head];
			head = uart.tx_head + 1;
			uart.tx_head = (uart.tx_head & ~0xff) + (head & 0xff);
		}

		if (uart.tx_head != uart.tx_tail) {
			*(uart.base + ucr1) |= 0x2000;
			*(uart.base + ufcr) |= 0xf << 10;
		} else if (IS_SYNC || IS_COOL)
			condSignal(uart.tx_cond);
		//	*(uart.base + ufcr) |= 0x1f << 10;

		uart.tx_full = 0;
	}
	mutexUnlock(uart.lock);
}


void set_clk(int dev_no)
{
	platformctl_t uart_clk;

	uart_clk.action = pctl_set;
	uart_clk.type = pctl_devclock;
	uart_clk.devclock.dev = uart_pctl_clk[dev_no - 1];
	uart_clk.devclock.state = 3;

	platformctl(&uart_clk);
}

void set_mux(int dev_no)
{
	platformctl_t uart_ctl;
	int i;

	uart_ctl.action = pctl_set;
	uart_ctl.type = pctl_iomux;

	for (i = 0; i < 4; i++) {
		uart_ctl.iomux.mux = uart_pctl_mux[(dev_no - 1)][i].pctl;
		uart_ctl.iomux.sion = 0;
		uart_ctl.iomux.mode = uart_pctl_mux[(dev_no - 1)][i].val;
		platformctl(&uart_ctl);
	}

	uart_ctl.action = pctl_set;
	uart_ctl.type = pctl_ioisel;

	for (i = 0; i < 2; i++) {
		uart_ctl.ioisel.isel = uart_pctl_isel[(dev_no - 1)][i].pctl;
		uart_ctl.ioisel.daisy = uart_pctl_isel[(dev_no - 1)][i].val;
		platformctl(&uart_ctl);
	}
}

void set_baud_rate(int baud_rate)
{
	int md, in, div, res;

	if (!baud_rate)
		return;

	/* count gcd */
	md = MODULE_CLK;
	in = 16 * uart.baud_rate;

	div = 0;
	while (md % 2 == 0 && in % 2 == 0) {
		md = md / 2;
		md = md / 2;
		div++;
	}

	while (md != in) {
		if (md % 2 == 0)
			md = md / 2;
		else if (in % 2 == 0)
			in = in / 2;
		else if (md > in)
			md = (md - in) / 2;
		else in = (in - md) / 2;
	}
	res = md;

	/* pow */
	md = 1;
	in = 2;
	while (div) {
		if (div & 1)
			md *= in;
		div /= 2;
		in *= in;
	}

	res = md * res;

	/* set baud rate */
	*(uart.base + ucr1) &= ~(1 << 14);
	*(uart.base + ubir) = ((uart.baud_rate * 16) / res) - 1;
	*(uart.base + ubmr) = (MODULE_CLK / res) - 1;
}

char __attribute__((aligned(8))) stack[2048];
char __attribute__((aligned(8))) stack0[2048];

void main(int argc, char **argv)
{
	u32 port;
	char *uartn;
	oid_t dir, root;
	msg_t msg;
	int err;

	if (argc == 1) {
		uart.mode = MODE_TTY;
		uart.dev_no = 1;
		uart.flags = FL_SYNC | FL_COOL;
		uart.parity = 0;
		uart.baud_rate = 115200;
	} else if (argc == 6) {
		uart.mode = atoi(argv[1]);
		uart.dev_no = atoi(argv[2]);
		uart.baud_rate = atoi(argv[3]);
		uart.parity = atoi(argv[4]);
		uart.flags = atoi(argv[5]);
	} else {
		printf("Usage: imx6ull-uart [mode] [device] [speed] [parity] [flags] or no args for default setting(tty, uart1, 115200, 8N1)\n");
		printf("\tmode: 0 - raw, 1 - tty\n\tdevice: 1 to 8\n");
		printf("\tspeed: baud_rate\n\tparity: 0 - none, 1 - odd, 2 - even\n");
		printf("\tflags: 1 - return if buffer is full, 2 - wait for transmition to end\n");
		return;
	}

	if (uart.dev_no <= 0 || uart.dev_no > 8) {
		printf("device number must be value 1-8\n");
		return;
	}

	if (portCreate(&port) != EOK)
		return;

	uart.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, uart_addr[uart.dev_no - 1]);

	if (uart.base == MAP_FAILED)
		return;

	set_clk(uart.dev_no);
	*(uart.base + ucr2) &= ~0;

	/* set correct daisy for rx input */
	set_mux(uart.dev_no);

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

	uart.ready = uart.mode ? 0 : 1;

	*(uart.base + ufcr) |= 0x1f << 10;
	*(uart.base + ufcr) &= ~(0x1 << 6);
	/* enable uart and rx ready interrupt */
	*(uart.base + ucr1) |= 0x0201;

	/* set Reference Frequency Divider */
	*(uart.base + ufcr) &= ~(0b111 << 7);
	*(uart.base + ufcr) |= 0b010 << 7;

	/* soft reset, tx&rx enable, 8bit transmit */
	*(uart.base + ucr2) = 0x4027;
	if (uart.parity)
		*(uart.base + ucr2) |= (uart.parity | 2) << 7;

	set_baud_rate(uart.baud_rate);

	*(uart.base + ucr3) = 0x704;

	beginthread(uart_intrthr, 3, &stack0, 2048, NULL);
	beginthread(uart_thr, 3, &stack, 2048, (void *)port);

	while (lookup("/", &root) < 0)
		usleep(100000);

	err = mkdir("/dev", 0);

	if (err < 0 && err != -EEXIST) {
		debug("imx6ull-uart: mkdir /dev failed\n");
	}

	uartn = malloc(strlen("uartx") + 1);
	sprintf(uartn, "uart%d", uart.dev_no);

	if (lookup("/dev", &dir) < 0) {
		debug("imx6ull-uart: /dev lookup failed");
	}

	msg.type = mtCreate;
	msg.i.create.dir = dir;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0;
	msg.i.create.dev.port = port;
	msg.i.create.dev.id = 0;
	msg.i.data = uartn;
	msg.i.size = strlen(uartn);
	msg.o.data = NULL;
	msg.o.size = 0;

	if (msgSend(dir.port, &msg) < 0 || msg.o.create.err != EOK) {
		debug("imx6ull-uart: Could not create device file\n");
		free(uartn);
	}
	free(uartn);

	uart_thr((void *)port);
}
