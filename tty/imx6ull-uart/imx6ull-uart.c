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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <paths.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/file.h>
#include <sys/platform.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/debug.h>
#include <posix/utils.h>
#include <fcntl.h>
#include <syslog.h>

#include <libtty.h>
#include <libtty-lf-fifo.h>
#include <libklog.h>

#include <phoenix/arch/imx6ull.h>

#define KMSG_CTRL_ID 100

#define LOG_TAG "imx6ull-uart"

#define READER 1
#define WRITER 2

#define MODULE_CLK 20000000

#define BUFSIZE 4096

#define RX_SW_FIFO_SIZE 256

/* interrupt flags */
#define RX_DONE        (1 << 0)
#define TX_DONE        (1 << 1)
#define PARITY_ERR     (1 << 2)
#define FRAME_ERR      (1 << 3)
#define HW_OVERRUN_ERR (1 << 4)
#define SW_OVERRUN_ERR (1 << 5)

/* register flags */
#define USR1_RRDY      (1 << 9)
#define USR1_TRDY      (1 << 13)
#define USR1_FRAMERR   (1 << 10)
#define USR1_PARITYERR (1 << 15)
#define USR2_RDR       (1 << 0)
#define USR2_ORE       (1 << 1)
#define UTS_TXFULL     (1 << 4)
#define UTS_SOFTRST    (1 << 0)
#define UCR1_UARTEN    (1 << 0)
#define UCR1_RRDYEN    (1 << 9)
#define UCR1_TRDYEN    (1 << 13)
#define UCR2_SRST      (1 << 0)
#define UCR2_RXEN      (1 << 1)
#define UCR2_TXEN      (1 << 2)
#define UCR2_WS        (1 << 5)
#define UCR2_IRTS      (1 << 14)
#define UCR3_RXDMUXSEL (1 << 2)
#define UCR3_RI        (1 << 8)
#define UCR3_DCD       (1 << 9)
#define UCR3_DSR       (1 << 10)


/* clang-format off */
enum { urxd = 0, utxd = 16, ucr1 = 32, ucr2, ucr3, ucr4, ufcr, usr1, usr2,
		uesc, utim, ubir, ubmr, ubrc, onems, uts, umcr };
/* clang-format on */


static uint32_t uart_addr[8] = { 0x02020000, 0x021E8000, 0x021EC000, 0x021F0000,
	0x021F4000, 0x021FC000, 0x02018000, 0x02288000 };

static uint32_t uart_pctl_clk[8] = { pctl_clk_uart1, pctl_clk_uart2, pctl_clk_uart3, pctl_clk_uart4,
	pctl_clk_uart5, pctl_clk_uart6, pctl_clk_uart7, pctl_clk_uart8 };

typedef struct {
	int pctl;
	char val;
} uart_pctl_t;

/* clang-format off */
static uart_pctl_t uart_pctl_mux[8][4] = {
	{ { pctl_mux_uart1_cts, 0 }, { pctl_mux_uart1_rts,  0 }, { pctl_mux_uart1_rx,  0 }, { pctl_mux_uart1_tx,  0 } },
	{ { pctl_mux_uart2_cts, 0 }, { pctl_mux_uart2_rts,  0 }, { pctl_mux_uart2_rx,  0 }, { pctl_mux_uart2_tx,  0 } },
	{ { pctl_mux_uart3_cts, 0 }, { pctl_mux_uart3_rts,  0 }, { pctl_mux_uart3_rx,  0 }, { pctl_mux_uart3_tx,  0 } },
	{ { pctl_mux_lcd_hsync, 2 }, { pctl_mux_lcd_vsync,  2 }, { pctl_mux_uart4_rx,  0 }, { pctl_mux_uart4_tx,  0 } },
	{ { pctl_mux_gpio1_09,  8 }, { pctl_mux_gpio1_08,   8 }, { pctl_mux_uart5_rx,  0 }, { pctl_mux_uart5_tx,  0 } },
	{ { pctl_mux_enet1_tx1, 1 }, { pctl_mux_enet1_txen, 1 }, { pctl_mux_enet2_rx1, 1 }, { pctl_mux_enet2_rx0, 1 } },
	{ { pctl_mux_lcd_d6,    1 }, { pctl_mux_lcd_d7,     1 }, { pctl_mux_lcd_d17,   1 }, { pctl_mux_lcd_d16,   1 } },
	{ { pctl_mux_lcd_d4,    1 }, { pctl_mux_lcd_d5,     1 }, { pctl_mux_lcd_d21,   1 }, { pctl_mux_lcd_d20,   1 } },
};
/* clang-format on */

static uart_pctl_t uart_pctl_isel[8][2] = {
	{ { pctl_isel_uart1_rts, 3 }, { pctl_isel_uart1_rx, 3 } },
	{ { pctl_isel_uart2_rts, 1 }, { pctl_isel_uart2_rx, 1 } },
	{ { pctl_isel_uart3_rts, 1 }, { pctl_isel_uart3_rx, 1 } },
	{ { pctl_isel_uart4_rts, 3 }, { pctl_isel_uart4_rx, 1 } },
	{ { pctl_isel_uart5_rts, 1 }, { pctl_isel_uart5_rx, 7 } },
	{ { pctl_isel_uart6_rts, 3 }, { pctl_isel_uart6_rx, 2 } },
	{ { pctl_isel_uart7_rts, 3 }, { pctl_isel_uart7_rx, 3 } },
	{ { pctl_isel_uart8_rts, 3 }, { pctl_isel_uart8_rx, 3 } },
};

static unsigned uart_intr_number[8] = { 58, 59, 60, 61, 62, 49, 71, 72 };

typedef struct {
	volatile uint32_t *base;
	uint32_t mode;
	uint16_t dev_no;
	uint8_t reader_busy;
	uint8_t print_errors;
	uint8_t use_syslog;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	handle_t openclose_lock;

	uint8_t int_flags;

	struct {
		uint32_t parity;
		uint32_t frame;
		uint32_t hw_overrun;
		uint32_t sw_overrun;
		uint32_t last_parity;
		uint32_t last_frame;
		uint32_t last_hw_overrun;
		uint32_t last_sw_overrun;
	} counters;

	/* unprocessed characters read from HW FIFO */
	lf_fifo_t rx_sw_fifo;
	uint8_t rx_sw_fifo_data[RX_SW_FIFO_SIZE];

	libtty_common_t tty_common;
} uart_t;

static uart_t uart = { 0 };


static void log_printf(int priority, const char *fmt, ...)
{
	va_list arg;

	va_start(arg, fmt);

	/* Don't use syslog until initialized */
	if (!uart.use_syslog) {
		printf("%s: ", LOG_TAG);
		vprintf(fmt, arg);
	}
	else {
		vsyslog(priority, fmt, arg);
	}

	va_end(arg);
}


static int flags_to_id(int flags)
{
	switch (flags & (O_RDONLY | O_WRONLY | O_RDWR)) {
		case O_RDONLY:
			return READER;
		case O_WRONLY:
			return WRITER;
		case O_RDWR:
			return (READER | WRITER);
		default:
			return 0;
	}
}

static int uart_open(int flags)
{
	/* TODO: set PGID? */
	int id = flags_to_id(flags);

	if (id <= 0) {
		return -EACCES;
	}

	if ((id & READER) != 0) {
		mutexLock(uart.openclose_lock);

		if (uart.reader_busy != 0) {
			mutexUnlock(uart.openclose_lock);
			return -EACCES;
		}

		/* start RX */
		*(uart.base + ucr2) |= UCR2_RXEN;
		*(uart.base + ucr1) |= UCR1_RRDYEN;

		uart.reader_busy = 1;

		mutexUnlock(uart.openclose_lock);
	}


	return id;
}


static int uart_close(id_t id)
{
	if (id <= 0) {
		return -EBADF;
	}

	if ((id & READER) != 0) {
		mutexLock(uart.openclose_lock);

		/* stop RX */
		*(uart.base + ucr1) &= ~UCR1_RRDYEN;
		*(uart.base + ucr2) &= ~UCR2_RXEN;

		uart.reader_busy = 0;

		mutexUnlock(uart.openclose_lock);
	}

	return 0;
}


static void uart_thr(void *arg)
{
	uint32_t port = (uint32_t)arg;
	msg_t msg;
	msg_rid_t rid;

	for (;;) {

		if (msgRecv(port, &msg, &rid) < 0) {
			memset(&msg, 0, sizeof(msg));
			msgRespond(port, &msg, rid);
			continue;
		}

		if (libklog_ctrlHandle(port, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.io.err = uart_open(msg.i.openclose.flags);
				break;
			case mtClose:
				msg.o.io.err = uart_close(msg.i.openclose.oid.id);
				break;
			case mtWrite:
				msg.o.io.err = libtty_write(&uart.tty_common, msg.i.data, msg.i.size, msg.i.io.mode);
				break;
			case mtRead:
				msg.o.io.err = libtty_read(&uart.tty_common, msg.o.data, msg.o.size, msg.i.io.mode);
				break;
			case mtGetAttr:
				if (msg.i.attr.type != atPollStatus) {
					msg.o.attr.err = -EINVAL;
					break;
				}
				msg.o.attr.val = libtty_poll_status(&uart.tty_common);
				msg.o.attr.err = EOK;
				break;
			case mtDevCtl: { /* ioctl */
				unsigned long request;
				const void *in_data = ioctl_unpack(&msg, &request, NULL);
				const void *out_data = NULL;
				pid_t pid = ioctl_getSenderPid(&msg);

				int err = libtty_ioctl(&uart.tty_common, pid, request, in_data, &out_data);
				ioctl_setResponse(&msg, request, err, out_data);

				break;
			}
		}

		msgRespond(port, &msg, rid);
	}
	return;
}


static int uart_intr(unsigned int intr, void *data)
{
	/* TX */
	*(uart.base + ucr1) &= ~UCR1_TRDYEN;

	/* RX */
	if ((*(uart.base + ucr1) & UCR1_RRDYEN) != 0) {
		while ((*(uart.base + usr2) & USR2_RDR) != 0) {
			/* NOTE: lock-free push */
			if (lf_fifo_push(&uart.rx_sw_fifo, (*(uart.base + urxd)) & 0xff) == 0) {
				uart.counters.sw_overrun++;
			}
		}
	}

	return 0;
}


static void check_errors(void)
{
	uint32_t stat1 = *(uart.base + usr1);
	uint32_t stat2 = *(uart.base + usr2);

	if ((stat1 & USR1_PARITYERR) != 0) {
		*(uart.base + usr1) = USR1_PARITYERR;
		uart.counters.parity++;
	}
	if ((stat1 & USR1_FRAMERR) != 0) {
		*(uart.base + usr1) = USR1_FRAMERR;
		uart.counters.frame++;
	}
	if ((stat2 & USR2_ORE) != 0) {
		*(uart.base + usr2) = USR2_ORE;
		uart.counters.hw_overrun++;
	}

	if (uart.print_errors != 0) {
		if (uart.counters.last_parity != uart.counters.parity) {
			log_printf(LOG_WARNING, "parity error (%u)\n", uart.counters.parity);
			uart.counters.last_parity = uart.counters.parity;
		}
		if (uart.counters.last_frame != uart.counters.frame) {
			log_printf(LOG_WARNING, "frame error (%u)\n", uart.counters.frame);
			uart.counters.last_frame = uart.counters.frame;
		}
		if (uart.counters.last_hw_overrun != uart.counters.hw_overrun) {
			log_printf(LOG_WARNING, "hw_overrun (%u)\n", uart.counters.hw_overrun);
			uart.counters.last_hw_overrun = uart.counters.hw_overrun;
		}
		if (uart.counters.last_sw_overrun != uart.counters.sw_overrun) {
			log_printf(LOG_WARNING, "sw_overrun (%u)\n", uart.counters.sw_overrun);
			uart.counters.last_sw_overrun = uart.counters.sw_overrun;
		}
	}
}


static void uart_process_rx(void)
{
	uint8_t c;

	/* NOTE: lock-free pop */
	while (lf_fifo_pop(&uart.rx_sw_fifo, &c) != 0) {
		libtty_putchar(&uart.tty_common, c, NULL);
	}
}


static void uart_process_tx(void)
{
	int wake = 0;

	while (libtty_txready(&uart.tty_common) != 0) {
		if ((*(uart.base + uts) & UTS_TXFULL) != 0) {
			*(uart.base + ucr1) |= UCR1_TRDYEN;
			break;
		}

		/* FIXME: potential data race on tx_fifo (lock-free access) */
		*(uart.base + utxd) = libtty_popchar(&uart.tty_common);

		wake = 1;
	}

	if (wake != 0) {
		libtty_wake_writer(&uart.tty_common);
	}
}


static void uart_intrthr(void *arg)
{
	mutexLock(uart.lock);

	for (;;) {
		check_errors();
		uart_process_rx();
		uart_process_tx();

		condWait(uart.cond, uart.lock, 0);

		/* NOTE: make sure TX interrupt is disabled (sticky cond) */
		*(uart.base + ucr1) &= ~UCR1_TRDYEN;

		if ((*(uart.base + ucr1) & UCR1_RRDYEN) && (*(uart.base + usr1) & USR1_RRDY)) {
			/* NOTE: disable & enable RX interrupt to avoid interrupt blocking (edge triggered) */
			*(uart.base + ucr1) &= ~UCR1_RRDYEN;
			*(uart.base + ucr1) |= UCR1_RRDYEN;
		}
	}

	mutexUnlock(uart.lock);
}


static void set_clk(int dev_no)
{
	platformctl_t uart_clk;

	uart_clk.action = pctl_set;
	uart_clk.type = pctl_devclock;
	uart_clk.devclock.dev = uart_pctl_clk[dev_no - 1];
	uart_clk.devclock.state = 3;

	platformctl(&uart_clk);
}


static void set_mux(int dev_no, int use_rts_cts)
{
	platformctl_t uart_ctl;
	int i, first_mux = 0;

	uart_ctl.action = pctl_set;
	uart_ctl.type = pctl_iomux;

	if (use_rts_cts == 0) {
		first_mux = 2; /* Skip RTS/CTS mux configuration */
	}

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


static void set_baudrate(void *_uart, speed_t baud)
{
	int md, in, div, res;

	int baud_rate = libtty_baudrate_to_int(baud);
	uart_t *uartptr = (uart_t *)_uart;

	if (baud_rate == 0) {
		return;
	}

	/* count gcd */
	md = MODULE_CLK;
	in = 16 * baud_rate;

	div = 0;
	while (((md % 2) == 0) && ((in % 2) == 0)) {
		md = md / 2;
		md = md / 2;
		div++;
	}

	while (md != in) {
		if ((md % 2) == 0) {
			md /= 2;
		}
		else if ((in % 2) == 0) {
			in /= 2;
		}
		else if (md > in) {
			md = (md - in) / 2;
		}
		else {
			in = (in - md) / 2;
		}
	}
	res = md;

	/* pow */
	md = 1;
	in = 2;
	while (div != 0) {
		if ((div & 1) != 0) {
			md *= in;
		}
		div /= 2;
		in *= in;
	}

	res = md * res;

	/* set baud rate */
	*(uartptr->base + ucr1) &= ~(1 << 14);
	*(uartptr->base + ubir) = ((baud_rate * 16) / res) - 1;
	*(uartptr->base + ubmr) = (MODULE_CLK / res) - 1;
}


static void set_cflag(void *_uart, tcflag_t *cflag)
{
	uart_t *uartptr = (uart_t *)_uart;

	/* CSIZE ony CS7 and CS8 (default) is supported */
	if ((*cflag & CSIZE) == CS7) {
		*(uartptr->base + ucr2) &= ~(1 << 6);
	}
	else { /* CS8 */
		*cflag &= ~CSIZE;
		*cflag |= CS8;

		*(uartptr->base + ucr2) |= (1 << 6);
	}

	/* parity */
	*(uartptr->base + ucr2) &= ~((1 << 8) | (1 << 7));
	*(uartptr->base + ucr2) |= (((*cflag & PARENB) != 0) << 8) | (((*cflag & PARODD) != 0) << 7);

	/* stop bits */
	if ((*cflag & CSTOPB) != 0) {
		*(uartptr->base + ucr2) |= (1 << 6);
	}
	else {
		*(uartptr->base + ucr2) &= ~(1 << 6);
	}
}


static void signal_txready(void *_uart)
{
	condSignal(uart.cond);
}


char __attribute__((aligned(8))) stack[2048];
char __attribute__((aligned(8))) stack0[2048];

static void print_usage(const char *progname)
{
	printf("Usage: %s [mode device speed parity use_rts_cts [-t] [-e] [-s]]\n", progname);
	printf("\tmode: 0 - raw, 1 - cooked (default cooked)\n");
	printf("\tdevice: 1 to 8 (default 1)\n");
	printf("\tspeed: baud_rate (default 115200)\n");
	printf("\tparity: 0 - none, 1 - odd, 2 - even (default none)\n");
	printf("\tuse_rts_cts: 0 - no hardware flow control, 1 - use hardware flow control (default no hardware flow control)\n");
	printf("\t-t - make it a default console device, might be empty (default yes)\n");
	printf("\t-e - report UART errors (default no)\n");
	printf("\t-s - use syslog for logs (default no)\n");
}


static void libklog_clbk(const char *data, size_t size)
{
	libtty_write(&uart.tty_common, data, size, 0);
}


int main(int argc, char **argv)
{
	uint32_t port;
	char uartn[sizeof("uartx") + 1];
	oid_t dev;
	int err;
	speed_t baud = B115200;
	int parity = 0;
	int is_cooked = 1;
	int use_rts_cts = 0;
	int is_console = 0;

	libtty_callbacks_t callbacks = {
		.arg = &uart,
		.set_baudrate = &set_baudrate,
		.set_cflag = &set_cflag,
		.signal_txready = &signal_txready,
	};

	if (libtty_init(&uart.tty_common, &callbacks, BUFSIZE, baud) < 0) {
		return -1;
	}

	if (argc == 1) {
		uart.dev_no = 1;
		is_console = 1;
	}
	else if ((argc >= 6) && (argc <= 8)) {
		is_cooked = atoi(argv[1]);
		uart.dev_no = atoi(argv[2]);
		baud = libtty_int_to_baudrate(atoi(argv[3]));
		parity = atoi(argv[4]);
		use_rts_cts = atoi(argv[5]);

		for (int num = 6; num < argc; num++) {
			if (strcmp(argv[num], "-t") == 0) {
				is_console = 1;
			}
			else if (strcmp(argv[num], "-e") == 0) {
				uart.print_errors = 1;
			}
			else if (strcmp(argv[num], "-s") == 0) {
				uart.use_syslog = 1;
			}
			else {
				print_usage(argv[0]);
				return 0;
			}
		}
	}
	else {
		print_usage(argv[0]);
		return 0;
	}

	if (uart.use_syslog != 0) {
		openlog(LOG_TAG, LOG_NDELAY, LOG_DAEMON);
	}

	if (baud < 0) {
		printf("Invalid baud rate!\n");
		print_usage(argv[0]);
		return 1;
	}
	uart.tty_common.term.c_ispeed = uart.tty_common.term.c_ospeed = baud;

	if ((parity < 0) || (parity > 2)) {
		printf("Invalid parity!\n");
		print_usage(argv[0]);
		return 1;
	}
	if (parity > 0) {
		uart.tty_common.term.c_cflag = PARENB | ((parity == 1) ? PARODD : 0);
	}

	if (is_cooked == 0) {
		libtty_set_mode_raw(&uart.tty_common);
	}

	if ((uart.dev_no <= 0) || (uart.dev_no > 8)) {
		printf("device number must be value 1-8\n");
		print_usage(argv[0]);
		return 1;
	}

	if (portCreate(&port) != EOK) {
		return 2;
	}

	lf_fifo_init(&uart.rx_sw_fifo, uart.rx_sw_fifo_data, sizeof(uart.rx_sw_fifo_data));

	uart.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, uart_addr[uart.dev_no - 1]);

	if (uart.base == MAP_FAILED) {
		return 2;
	}

	set_clk(uart.dev_no);

	/* software reset */
	*(uart.base + ucr2) = 0;
	while ((*(uart.base + uts) & UTS_SOFTRST) != 0) {
	}

	/* set correct daisy for rx input */
	set_mux(uart.dev_no, use_rts_cts);

	if (mutexCreate(&uart.lock) != EOK) {
		return 2;
	}

	if (mutexCreate(&uart.openclose_lock) != EOK) {
		return 2;
	}

	if (condCreate(&uart.cond) != EOK) {
		return 2;
	}

	interrupt(uart_intr_number[uart.dev_no - 1], uart_intr, NULL, uart.cond, &uart.inth);

	/* set TX & RX FIFO watermark, DCE mode */
	*(uart.base + ufcr) = (0x04 << 10) | (0 << 6) | (0x1);

	/* set Reference Frequency Divider */
	*(uart.base + ufcr) &= ~(0b111 << 7);
	*(uart.base + ufcr) |= 0b010 << 7;

	/* ignore RTS pin, 8-bit transmit, TX enable, soft reset */
	*(uart.base + ucr2) = UCR2_IRTS | UCR2_WS | UCR2_TXEN | UCR2_SRST;

	set_cflag(&uart, &uart.tty_common.term.c_cflag);
	set_baudrate(&uart, baud);

	/* set muxed mode */
	*(uart.base + ucr3) |= UCR3_RXDMUXSEL;

	/* enable UART */
	*(uart.base + ucr1) |= UCR1_UARTEN;

	beginthread(uart_intrthr, 3, &stack0, 2048, NULL);
	beginthread(uart_thr, 3, &stack, 2048, (void *)port);

	sprintf(uartn, "uart%u", uart.dev_no % 10u);

	dev.port = port;
	dev.id = 0;

	err = create_dev(&dev, uartn);
	if (err != 0) {
		debug("imx6ull-uart: Could not create device file\n");
	}

	if (is_console != 0) {
		create_dev(&dev, _PATH_CONSOLE);
		libklog_init(libklog_clbk);
		oid_t kmsgctrl = { .port = port, .id = KMSG_CTRL_ID };
		libklog_ctrlRegister(&kmsgctrl);
	}

	uart_thr((void *)port);

	return 0;
}
