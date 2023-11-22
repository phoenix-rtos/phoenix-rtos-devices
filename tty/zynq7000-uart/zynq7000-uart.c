/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Zynq - 7000 UART driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <paths.h>

#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/debug.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/interrupt.h>

#include <board_config.h>
#include <libtty.h>
#include <libklog.h>
#include <posix/utils.h>

#include <phoenix/ioctl.h>
#include <phoenix/arch/zynq7000.h>


#define UARTS_MAX_CNT 2
#define UART_REF_CLK  50000000 /* 50 MHz */

#define KMSG_CTRL_ID 100


typedef struct {
	volatile uint32_t *base;
	oid_t oid;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	libtty_common_t tty;

	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(8)));
} uart_t;


static const struct {
	uint32_t base;
	unsigned int irq;
	uint16_t clk;
	uint16_t rxPin;
	uint16_t txPin;
} info[UARTS_MAX_CNT] = {
	{ 0xe0000000, 59, pctl_amba_uart0_clk, UART0_RX, UART0_TX },
	{ 0xe0001000, 82, pctl_amba_uart1_clk, UART1_RX, UART1_TX }
};


static struct {
	uart_t uart;
	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(8)));
} uart_common;


enum {
	cr = 0, mr, ier, idr, imr, isr, baudgen, rxtout, rxwm, modemcr, modemsr, sr, fifo,
	baud_rate_divider_reg0, flow_delay_reg0, tx_fifo_trigger_level0,
};


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	/* RX Trigger IRQ occurred */
	if (*(uart->base + isr) & (1 << 0)) {
		*(uart->base + idr) = (1 << 0); /* Disable IRQ to not receive more interrupts */
	}

	return 1;
}


static void uart_intThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	int wake;

	mutexLock(uart->lock);

	for (;;) {
		while (!libtty_txready(&uart->tty) && (*(uart->base + sr) & (1 << 1))) {
			condWait(uart->cond, uart->lock, 0);
		}

		/* Receive data until RX FIFO is not empty */
		while (!(*(uart->base + sr) & (1 << 1))) {
			libtty_putchar(&uart->tty, *(uart->base + fifo), NULL);
		}

		/* Transmit data until TX TTY buffer is empty or TX FIFO is full */
		wake = 0;
		while (libtty_txready(&uart->tty) && !(*(uart->base + sr) & (1 << 4))) {
			*(uart->base + fifo) = libtty_popchar(&uart->tty);
			wake = 1;
		}

		if (wake) {
			libtty_wake_writer(&uart->tty);
		}

		/* RX Trigger IRQ occurred */
		if (*(uart->base + isr) & (1 << 0)) {
			*(uart->base + isr) = (1 << 0); /* RX Trigger status can be cleared after getting data from RX FIFO */
			*(uart->base + ier) = (1 << 0); /* Enable RX Trigger irq which has been disabled in uart irq handler */
		}
	}

	mutexUnlock(uart->lock);
}


/* According to TRM:
 *  baud_rate = ref_clk / (bgen * (bdiv + 1))
 *  bgen: 2 - 65535
 *  bdiv: 4 - 255                             */
static void uart_setBaudrate(void *data, speed_t speed)
{
	uint32_t bestDiff, diff;
	uint32_t calcBaudrate;
	uint32_t bdiv, bgen, bestBdiv = 4, bestBgen = 2;

	uart_t *uart = (uart_t *)data;
	int baudrate = libtty_baudrate_to_int(speed);

	bestDiff = (uint32_t)baudrate;

	for (bdiv = 4; bdiv < 255; bdiv++) {
		bgen = UART_REF_CLK / (baudrate * (bdiv + 1));

		if (bgen < 2 || bgen > 65535) {
			continue;
		}

		calcBaudrate = UART_REF_CLK / (bgen * (bdiv + 1));

		if (calcBaudrate > baudrate) {
			diff = calcBaudrate - baudrate;
		}
		else {
			diff = baudrate - calcBaudrate;
		}

		if (diff < bestDiff) {
			bestDiff = diff;
			bestBdiv = bdiv;
			bestBgen = bgen;
		}
	}

	/* Disable TX and RX */
	*(uart->base + cr) = (1 << 5) | (1 << 3);

	/* Configure baudrate */
	*(uart->base + baudgen) = bestBgen;
	*(uart->base + baud_rate_divider_reg0) = bestBdiv;

	/* Reset TX and RX */
	*(uart->base + cr) |= (1 << 1) | (1 << 0);

	/* Enable TX and RX */
	*(uart->base + cr) = (1 << 4) | (1 << 2);
}


static void uart_setCFlag(void *data, tcflag_t *cflag)
{
	uart_t *uart = (uart_t *)data;

	/* Number of bits in each character */
	if ((*cflag & CSIZE) == CS6) {
		*(uart->base + mr) = (*(uart->base + mr) & ~0x00000006) | (3 << 1);
	}
	else if ((*cflag & CSIZE) == CS7) {
		*(uart->base + mr) = (*(uart->base + mr) & ~0x00000006) | (2 << 1);
	}
	else {
		*(uart->base + mr) &= ~(3 << 1);
		*cflag &= ~CSIZE;
		*cflag |= CS8;
	}

	/* Parity */
	if (*cflag & PARENB) {
		*(uart->base + mr) &= ~(7 << 3);
	}
	else if (*cflag & PARODD) {
		*(uart->base + mr) = (*(uart->base + mr) & ~0x00000038) | (1 << 3);
	}
	else {
		*(uart->base + mr) = (*(uart->base + mr) & ~0x00000038) | (4 << 3);
	}

	/* Stop bits */
	if (*cflag & CSTOPB) {
		*(uart->base + mr) = (*(uart->base + mr) & ~0x000000c0) | (2 << 6);
	}
	else {
		*(uart->base + mr) &= ~0x000000c0;
	}
}


static void uart_signalTXReady(void *data)
{
	uart_t *uart = (uart_t *)data;

	condSignal(uart->cond);
}


static void uart_ioctl(unsigned port, msg_t *msg)
{
	int err;
	pid_t pid;
	unsigned long req;
	const void *inData, *outData = NULL;

	inData = ioctl_unpack(msg, &req, NULL);
	pid = ioctl_getSenderPid(msg);

	err = libtty_ioctl(&uart_common.uart.tty, pid, req, inData, &outData);

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
				break;

			case mtClose:
				break;

			case mtWrite:
				msg.o.io.err = libtty_write(&uart_common.uart.tty, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtRead:
				msg.o.io.err = libtty_read(&uart_common.uart.tty, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atPollStatus) {
					msg.o.attr.val = libtty_poll_status(&uart_common.uart.tty);
					msg.o.attr.err = EOK;
					break;
				}

				msg.o.attr.err = -EINVAL;
				break;

			case mtDevCtl:
				uart_ioctl(port, &msg);
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
	char path[12];

	snprintf(path, sizeof(path), "/dev/uart%u", id);
	if (create_dev(&uart_common.uart.oid, path) < 0) {
		debug("zynq7000-uart: cannot create device file\n");
	}

	if (id == UART_CONSOLE_USER) {
		libklog_init(uart_klogClbk);

		if (create_dev(&uart_common.uart.oid, _PATH_CONSOLE) < 0) {
			debug("zynq7000-uart: cannot create device file\n");
		}

		oid_t kmsgctrl = { .port = uart_common.uart.oid.port, .id = KMSG_CTRL_ID };
		libklog_ctrlRegister(&kmsgctrl);
	}
}


static int uart_setPin(uint32_t pin)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_mio;

	/* Set default properties for UART's pins */
	ctl.mio.pin = pin;
	ctl.mio.l0 = ctl.mio.l1 = ctl.mio.l2 = 0;
	ctl.mio.l3 = 0x7;
	ctl.mio.speed = 0;
	ctl.mio.ioType = 1;
	ctl.mio.pullup = 1;
	ctl.mio.disableRcvr = 1;

	switch (pin) {
		case UART0_RX:
		case UART1_RX:
			ctl.mio.triEnable = 1;
			break;

		case UART0_TX:
		case UART1_TX:
			ctl.mio.triEnable = 0;
			break;

		default:
			return -EINVAL;
	}

	return platformctl(&ctl);
}


static int uart_initAmbaClk(unsigned int dev)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_ambaclock;

	ctl.ambaclock.dev = dev;
	ctl.ambaclock.state = 1;

	return platformctl(&ctl);
}


static int uart_initClk(void)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devclock;

	/* Set IO PLL as source clock and set divider:
	 * IO_PLL / 0x14 :  1000 MHz / 20 = 50 MHz     */
	ctl.devclock.dev = pctl_ctrl_uart_clk;
	ctl.devclock.clkact0 = 0x1;
	ctl.devclock.clkact1 = 0x1;
	ctl.devclock.srcsel = 0;
	ctl.devclock.divisor0 = 0x14;

	return platformctl(&ctl);
}


static int uart_init(unsigned int n, speed_t baud, int raw)
{
	libtty_callbacks_t callbacks;
	uart_t *uart = &uart_common.uart;

	if (uart_setPin(info[n].rxPin) < 0 || uart_setPin(info[n].txPin) < 0 || uart_initAmbaClk(info[n].clk) < 0) {
		return -EINVAL;
	}

	uart->base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, info[n].base);
	if (uart->base == MAP_FAILED) {
		return -ENOMEM;
	}

	callbacks.arg = uart;
	callbacks.set_cflag = uart_setCFlag;
	callbacks.set_baudrate = uart_setBaudrate;
	callbacks.signal_txready = uart_signalTXReady;

	if (libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, baud) < 0) {
		munmap((void *)uart->base, _PAGE_SIZE);
		return -ENOENT;
	}

	if (condCreate(&uart->cond) != EOK) {
		munmap((void *)uart->base, _PAGE_SIZE);
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		return -ENOENT;
	}

	if (mutexCreate(&uart->lock) != EOK) {
		munmap((void *)uart->base, _PAGE_SIZE);
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		resourceDestroy(uart->cond);
		return -ENOENT;
	}

	/* Set raw mode */
	if (raw) {
		libtty_set_mode_raw(&uart->tty);
	}

	/* normal mode, 1 stop bit, no parity, 8 bits */
	uart_setCFlag(uart, &uart->tty.term.c_cflag);

	/* Set trigger level, range: 1-63 */
	*(uart->base + rxwm) = 1;

	/* Enable RX FIFO trigger */
	*(uart->base + ier) = (1 << 0);

	uart->tty.term.c_ispeed = uart->tty.term.c_ospeed = baud;
	uart_setBaudrate(uart, baud);

	beginthread(uart_intThread, 4, &uart->stack, sizeof(uart->stack), (void *)uart);
	interrupt(info[n].irq, uart_interrupt, uart, uart->cond, &uart->inth);

	return EOK;
}


static void uart_help(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("Options:\n");
	printf("\t-b <baudrate>   - baudrate\n");
	printf("\t-n <id>         - uart controller ID\n");
	printf("\t-r              - set raw mode (default cooked)\n");
	printf("\t-h              - print this message\n");
}


int main(int argc, char **argv)
{
	/* Default console configuration */
	int uartn = UART_CONSOLE_USER;
	speed_t baud = B115200;
	int c, raw = 0;

	if (argc > 1) {
		while ((c = getopt(argc, argv, "n:b:rh")) != -1) {
			switch (c) {
				case 'b':
					baud = libtty_int_to_baudrate(atoi(optarg));
					if (baud == (speed_t)-1) {
						debug("zynq7000-uart: wrong baudrate value\n");
						return EXIT_FAILURE;
					}
					break;

				case 'n':
					uartn = atoi(optarg);
					if (uartn >= UARTS_MAX_CNT) {
						debug("zynq7000-uart: wrong uart ID\n");
						return EXIT_FAILURE;
					}
					break;

				case 'r':
					raw = 1;
					break;

				case 'h':
					uart_help(argv[0]);
					return EXIT_SUCCESS;

				default:
					uart_help(argv[0]);
					return EXIT_FAILURE;
			}
		}
	}

	if (uartn < 0) {
		debug("zynq7000-uart: wrong uart ID, this uart cannot be a console\n");
		return EXIT_FAILURE;
	}

	if (uart_initClk() < 0) {
		debug("zynq7000-uart: cannot initialize clocks\n");
		return EXIT_FAILURE;
	}

	portCreate(&uart_common.uart.oid.port);

	if (uart_init(uartn, baud, raw) < 0) {
		debug("zynq7000-uart: cannot initialize uart\n");
		return EXIT_FAILURE;
	}

	beginthread(uart_dispatchMsg, 4, uart_common.stack, sizeof(uart_common.stack), NULL);
	uart_mkDev(uartn);
	uart_dispatchMsg(NULL);

	return EXIT_SUCCESS;
}
