/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * i.MX6ULL UART driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz, Marek Białowąs
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
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

#include <libtty.h>

#include <phoenix/arch/imx6ull.h>

enum { urxd = 0, utxd = 16, ucr1 = 32, ucr2, ucr3, ucr4, ufcr, usr1, usr2,
	uesc, utim, ubir, ubmr, ubrc, onems, uts, umcr };

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

unsigned uart_intr_number[8] = { 58, 59, 60, 61, 62, 49, 71, 72 };

typedef struct {
	volatile u32 *base;
	u32 mode;
	u16 dev_no;

	handle_t cond;
	handle_t inth;
	handle_t lock;

	libtty_common_t tty_common;
} uart_t;

uart_t uart = { 0 };

#define MODULE_CLK 20000000

#define BUFSIZE 4096

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
			// TODO: set PGID?
			break;
		case mtWrite:
			msg.o.io.err = libtty_write(&uart.tty_common, msg.i.data, msg.i.size, msg.i.io.mode);
			break;
		case mtRead:
			msg.o.io.err = libtty_read(&uart.tty_common, msg.o.data, msg.o.size, msg.i.io.mode);
			break;
		case mtClose:
			break;
		case mtGetAttr:
			if (msg.i.attr.type == atPollStatus)
				msg.o.attr.val = libtty_poll_status(&uart.tty_common);
			else
				msg.o.attr.val = -EINVAL;
			break;
		case mtDevCtl: { /* ioctl */
				unsigned long request;
				const void *in_data = ioctl_unpack(&msg, &request, NULL);
				const void *out_data = NULL;
				pid_t pid = ioctl_getSenderPid(&msg);

				int err = libtty_ioctl(&uart.tty_common, pid, request, in_data, &out_data);
				ioctl_setResponse(&msg, request, err, out_data);
			}
			break;
		}

		msgRespond(port, &msg, rid);
	}
	return;
}


static int uart_intr(unsigned int intr, void *data)
{
	/* disable tx ready interrupt ASAP to minimize interrupts received */
	*(uart.base + ucr1) &= ~0x2000;

	return uart.cond;
}


static void uart_intrthr(void *arg)
{
	for (;;) {
		/* wait for character or transmit data */
		mutexLock(uart.lock);
		while (!(*(uart.base + usr2) & (1 << 0))) {  // nothing to RX
			if (libtty_txready(&uart.tty_common)) { // we something to TX
				if ((*(uart.base + usr1) & (1 << 13))) // TX ready
					break;
				else
					*(uart.base + ucr1) |= 0x2000; // wait for TRDY interrupt
			}
			condWait(uart.cond, uart.lock, 0);
		}
		/* disable tx ready interrupt again (sticky conds) */
		*(uart.base + ucr1) &= ~0x2000;

		mutexUnlock(uart.lock);

		/* RX */
		while ((*(uart.base + usr2) & (1 << 0)))
			libtty_putchar(&uart.tty_common, *(uart.base + urxd), NULL);

		/* TX */
		while (libtty_txready(&uart.tty_common)) {
			if (*(uart.base + uts) & (1 << 4)) { // check TXFULL bit
				break; /* wait in main loop for TX to be ready before resuming operation */
			}
			*(uart.base + utxd) = libtty_getchar(&uart.tty_common, NULL);
		}
	}
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

void set_mux(int dev_no, int use_rts_cts)
{
	platformctl_t uart_ctl;
	int i, first_mux = 0;

	uart_ctl.action = pctl_set;
	uart_ctl.type = pctl_iomux;

	if (!use_rts_cts)
		first_mux = 2; /* Skip RTS/CTS mux configuration */

	for (i = first_mux; i < 4; i++) {
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

void set_baudrate(void* _uart, speed_t baud)
{
	int md, in, div, res;

	int baud_rate = libtty_baudrate_to_int(baud);
	uart_t* uartptr = (uart_t*) _uart;

	if (!baud_rate)
		return;

	/* count gcd */
	md = MODULE_CLK;
	in = 16 * baud_rate;

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
	*(uartptr->base + ucr1) &= ~(1 << 14);
	*(uartptr->base + ubir) = ((baud_rate * 16) / res) - 1;
	*(uartptr->base + ubmr) = (MODULE_CLK / res) - 1;
}

void set_cflag(void* _uart, tcflag_t* cflag)
{
	uart_t* uartptr = (uart_t*) _uart;

	/* CSIZE ony CS7 and CS8 (default) is supported */
	if ((*cflag & CSIZE) == CS7)
		*(uartptr->base + ucr2) &= ~(1 << 6);
	else { /* CS8 */
		*cflag &= ~CSIZE;
		*cflag |= CS8;

		*(uartptr->base + ucr2) |= (1 << 6);
	}

	/* parity */
	*(uartptr->base + ucr2) &= ~((1 << 8) | (1 << 7));
	*(uartptr->base + ucr2) |= (((*cflag & PARENB) != 0) << 8) | (((*cflag & PARODD) != 0) << 7);

	/* stop bits */
	if (*cflag & CSTOPB)
		*(uartptr->base + ucr2) |= (1 << 6);
	else
		*(uartptr->base + ucr2) &= ~(1 << 6);
}

static void signal_txready(void* _uart)
{
	uart_t* uartptr = (uart_t*) _uart;

	mutexLock(uartptr->lock);
	condSignal(uartptr->cond);
	mutexUnlock(uartptr->lock);
}


char __attribute__((aligned(8))) stack[2048];
char __attribute__((aligned(8))) stack0[2048];

static void print_usage(const char* progname) {
	printf("Usage: %s [mode] [device] [speed] [parity] [use_rts_cts] or no args for default settings (cooked, uart1, B115200, 8N1)\n", progname);
	printf("\tmode: 0 - raw, 1 - cooked\n\tdevice: 1 to 8\n");
	printf("\tspeed: baud_rate\n\tparity: 0 - none, 1 - odd, 2 - even\n");
	printf("\tuse_rts_cts: 0 - no hardware flow control, 1 - use hardware flow control\n");
}

int main(int argc, char **argv)
{
	u32 port;
	char uartn[sizeof("uartx") + 1];
	oid_t dir, root;
	msg_t msg;
	int err;
	speed_t baud = B115200;
	int parity = 0;
	int is_cooked = 1;
	int use_rts_cts = 0;

	libtty_callbacks_t callbacks = {
		.arg = &uart,
		.set_baudrate = &set_baudrate,
		.set_cflag = &set_cflag,
		.signal_txready = &signal_txready,
	};

	if (libtty_init(&uart.tty_common, &callbacks, BUFSIZE) < 0)
		return -1;

	if (argc == 1) {
		uart.dev_no = 1;
	} else if (argc == 6) {
		is_cooked = atoi(argv[1]);
		uart.dev_no = atoi(argv[2]);
		parity = atoi(argv[4]);
		baud = libtty_int_to_baudrate(atoi(argv[3]));
		use_rts_cts = atoi(argv[5]);
	} else {
		print_usage(argv[0]);
		return 0;
	}

	if (baud < 0) {
		printf("Invalid baud rate!\n");
		print_usage(argv[0]);
		return 1;
	}
	uart.tty_common.term.c_ispeed = uart.tty_common.term.c_ospeed = baud;

	if (parity < 0 || parity > 2) {
		printf("Invalid parity!\n");
		print_usage(argv[0]);
		return 1;
	}
	if (parity > 0)
		uart.tty_common.term.c_cflag = PARENB | ((parity == 1) ? PARODD : 0);

	if (!is_cooked)
		libtty_set_mode_raw(&uart.tty_common);

	if (uart.dev_no <= 0 || uart.dev_no > 8) {
		printf("device number must be value 1-8\n");
		print_usage(argv[0]);
		return 1;
	}

	if (portCreate(&port) != EOK)
		return 2;

	uart.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, uart_addr[uart.dev_no - 1]);

	if (uart.base == MAP_FAILED)
		return 2;

	set_clk(uart.dev_no);
	*(uart.base + ucr2) &= ~0;

	/* set correct daisy for rx input */
	set_mux(uart.dev_no, use_rts_cts);

	while (!(*(uart.base + ucr2) & 1));

	if (mutexCreate(&uart.lock) != EOK)
		return 2;

	if (condCreate(&uart.cond) != EOK)
		return 2;

	interrupt(uart_intr_number[uart.dev_no - 1], uart_intr, NULL, uart.cond, &uart.inth);


	/* set TX & RX FIFO watermark, DCE mode */
	*(uart.base + ufcr) = (0x04 << 10) | (0 << 6) | (0x1);

	/* set Reference Frequency Divider */
	*(uart.base + ufcr) &= ~(0b111 << 7);
	*(uart.base + ufcr) |= 0b010 << 7;

	/* enable uart and rx ready interrupt */
	*(uart.base + ucr1) |= 0x0201;

	/* soft reset, tx&rx enable, 8bit transmit */
	*(uart.base + ucr2) = 0x4027;

	set_cflag(&uart, &uart.tty_common.term.c_cflag);
	set_baudrate(&uart, baud);

	*(uart.base + ucr3) = 0x704;

	beginthread(uart_intrthr, 3, &stack0, 2048, NULL);
	beginthread(uart_thr, 3, &stack, 2048, (void *)port);

	while (lookup("/", NULL, &root) < 0)
		usleep(100000);

	err = mkdir("/dev", 0);

	if (err < 0 && errno != EEXIST) {
		debug("imx6ull-uart: mkdir /dev failed\n");
	}

	sprintf(uartn, "uart%u", uart.dev_no % 10);

	if (lookup("/dev", NULL, &dir) < 0) {
		debug("imx6ull-uart: /dev lookup failed");
	}

	msg.type = mtCreate;
	msg.i.create.dir = dir;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0;
	msg.i.create.dev.port = port;
	msg.i.create.dev.id = 0;
	msg.i.data = uartn;
	msg.i.size = strlen(uartn) + 1;
	msg.o.data = NULL;
	msg.o.size = 0;

	if (msgSend(dir.port, &msg) < 0 || msg.o.create.err != EOK) {
		debug("imx6ull-uart: Could not create device file\n");
	}

	uart_thr((void *)port);

	return 0;
}
