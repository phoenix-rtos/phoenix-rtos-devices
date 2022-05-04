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

#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/debug.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/interrupt.h>

#include <libtty.h>
#include <posix/utils.h>

#include <phoenix/ioctl.h>
#include <phoenix/arch/zynq7000.h>


#define UARTS_MAX_CNT 2
#define UART_REF_CLK  50000000 /* 50 MHz */


typedef struct {
	volatile uint32_t *base;
	oid_t oid;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	libtty_common_t tty;

	uint8_t stack[_PAGE_SIZE];
} uart_t;


static const struct {
	uint32_t base;
	unsigned int irq;
	uint16_t clk;
	uint16_t rxPin;
	uint16_t txPin;
} info[UARTS_MAX_CNT] = {
	{ 0xe0000000, 59, pctl_amba_uart0_clk, pctl_mio_pin_10, pctl_mio_pin_11 },
	{ 0xe0001000, 82, pctl_amba_uart1_clk, pctl_mio_pin_49, pctl_mio_pin_48 }
};


struct {
	uart_t uarts[UARTS_MAX_CNT];
	uint8_t stack[_PAGE_SIZE];
} uart_common;


enum {
	cr = 0, mr, ier, idr, imr, isr, baudgen, rxtout, rxwm, modemcr, modemsr, sr, fifo,
	baud_rate_divider_reg0, flow_delay_reg0, tx_fifo_trigger_level0,
};


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	/* RX Trigger IRQ occurred */
	if (*(uart->base + isr) & 0x1)
		*(uart->base + idr) = 0x1; /* Disable IRQ to not receive more interrupts */

	return 1;
}


static void uart_intThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;

	for (;;) {
		mutexLock(uart->lock);
		while (!libtty_txready(&uart->tty) && (*(uart->base + sr) & 0x2))
			condWait(uart->cond, uart->lock, 0);
		mutexUnlock(uart->lock);

		/* Receive data until RX FIFO is not empty */
		while (!(*(uart->base + sr) & 0x2))
			libtty_putchar(&uart->tty, *(uart->base + fifo), NULL);

		/* Transmit data until TX TTY buffer is empty or TX FIFO is full */
		while (libtty_txready(&uart->tty) && !(*(uart->base + sr) & (0x1 << 4)))
			*(uart->base + fifo) = libtty_popchar(&uart->tty);

		libtty_wake_writer(&uart->tty);

		/* RX Trigger IRQ occurred */
		if (*(uart->base + isr) & 0x1) {
			*(uart->base + isr) = 0x1; /* RX Trigger status can be cleared after getting data from RX FIFO */
			*(uart->base + ier) = 0x1; /* Enable RX Trigger irq which has been disabled in uart irq handler */
		}
	}
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

		if (bgen < 2 || bgen > 65535)
			continue;

		calcBaudrate = UART_REF_CLK / (bgen * (bdiv + 1));

		if (calcBaudrate > baudrate)
			diff = calcBaudrate - baudrate;
		else
			diff = baudrate - calcBaudrate;

		if (diff < bestDiff) {
			bestDiff = diff;
			bestBdiv = bdiv;
			bestBgen = bgen;
		}
	}

	*(uart->base + baudgen) = bestBgen;
	*(uart->base + baud_rate_divider_reg0) = bestBdiv;
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
	if (*cflag & PARENB)
		*(uart->base + mr) &= ~(7 << 3);
	else if (*cflag & PARODD)
		*(uart->base + mr) = (*(uart->base + mr) & ~0x00000038) | (1 << 3);


	/* Stop bits */
	if (*cflag & CSTOPB)
		*(uart->base + mr) = (*(uart->base + mr) & ~0x000000c0) | (2 << 6);
	else
		*(uart->base + mr) &= ~0x000000c0;
}


static void uart_signalTXReady(void *data)
{
	uart_t *uart = (uart_t *)data;

	condSignal(uart->cond);
}


static uart_t *uart_get(oid_t *oid)
{
	int i;

	for (i = 0; i < UARTS_MAX_CNT; i++) {
		if ((uart_common.uarts[i].base != NULL) && (uart_common.uarts[i].oid.id == oid->id) && (uart_common.uarts[i].oid.port == oid->port))
			return &uart_common.uarts[i];
	}

	return NULL;
}


static void uart_ioctl(unsigned port, msg_t *msg)
{
	int err;
	pid_t pid;
	oid_t oid;
	uart_t *uart;
	unsigned long req;
	ioctl_in_t *ioctl;
	const void *inData, *outData = NULL;

	ioctl = (ioctl_in_t *)msg->i.raw;

	oid.port = port;
	oid.id = ioctl->id;

	inData = ioctl_unpack(msg, &req, NULL);
	pid = ioctl_getSenderPid(msg);

	if ((uart = uart_get(&oid)) != NULL)
		err = libtty_ioctl(&uart->tty, pid, req, inData, &outData);
	else
		err = -EINVAL;

	ioctl_setResponse(msg, req, err, outData);
}


static void uart_dispatchMsg(void *arg)
{
	msg_t msg;
	uart_t *uart;
	unsigned long int rid;
	uint32_t port = (uint32_t)arg;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
			case mtOpen:
				break;

			case mtClose:
				break;

			case mtWrite:
				if ((uart = uart_get(&msg.i.io.oid)) == NULL) {
					msg.o.io.err = -ENOENT;
					break;
				}

				msg.o.io.err = libtty_write(&uart->tty, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtRead:
				if ((uart = uart_get(&msg.i.io.oid)) == NULL) {
					msg.o.io.err = -ENOENT;
					break;
				}

				msg.o.io.err = libtty_read(&uart->tty, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atPollStatus && (uart = uart_get(&msg.i.attr.oid)) != NULL) {
					msg.o.attr.val = libtty_poll_status(&uart->tty);
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


static void uart_mkDevFiles(void)
{
	int i;
	oid_t dir;
	char path[12];

	while (lookup("/", NULL, &dir) < 0)
		usleep(10000);

	for (i = 0; i < UARTS_MAX_CNT; ++i) {
		if (uart_common.uarts[i].base == NULL)
			continue;

		snprintf(path, sizeof(path), "/dev/uart%d", i);
		if (create_dev(&uart_common.uarts[i].oid, path) < 0)
			debug("uart: Cannot create device file.\n");
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
	ctl.mio.pullup = 0;
	ctl.mio.disableRcvr = 0;

	switch (pin) {
		/* Uart Rx */
		case pctl_mio_pin_10:
		case pctl_mio_pin_49:
			ctl.mio.triEnable = 1;
			break;

		/* Uart Tx */
		case pctl_mio_pin_11:
		case pctl_mio_pin_48:
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


static int uart_init(unsigned int n, speed_t baud)
{
	libtty_callbacks_t callbacks;
	uart_t *uart = &uart_common.uarts[n];

	if (uart_setPin(info[n].rxPin) < 0 || uart_setPin(info[n].txPin) < 0 || uart_initAmbaClk(info[n].clk) < 0)
		return -EINVAL;

	uart->base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, info[n].base);
	if (uart->base == MAP_FAILED)
		return -ENOMEM;

	callbacks.arg = uart;
	callbacks.set_cflag = uart_setCFlag;
	callbacks.set_baudrate = uart_setBaudrate;
	callbacks.signal_txready = uart_signalTXReady;

	if (libtty_init(&uart->tty, &callbacks, _PAGE_SIZE) < 0) {
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

	/* Reset RX & TX */
	*(uart->base + cr) = 0x3;

	/* Disable TX and RX */
	*(uart->base + cr) = (*(uart->base + cr) & ~0x000001ff) | 0x00000028;

	uart_setBaudrate(uart, baud);
	uart->tty.term.c_ispeed = uart->tty.term.c_ospeed = baud;

	/* normal mode, 1 stop bit, no parity, 8 bits */
	uart_setCFlag(uart, &uart->tty.term.c_cflag);

	/* Set trigger level, range: 1-63 */
	*(uart->base + rxwm) = 1;
	/* Enable RX FIFO trigger */
	*(uart->base + ier) |= 0x1;

	/* Uart Control Register
	 * TXEN = 0x1; RXEN = 0x1; TXRES = 0x1; RXRES = 0x1 */
	*(uart->base + cr) = (*(uart->base + cr) & ~0x000001ff) | 0x00000017;


	beginthread(uart_intThread, 4, &uart->stack, sizeof(uart->stack), (void *)uart);
	interrupt(info[n].irq, uart_interrupt, uart, uart->cond, &uart->inth);

	return EOK;
}


int main(int argc, char **argv)
{
	uart_t *uart;
	speed_t baud = B115200;
	uint32_t port;
	int n, c, console = 1;

	/* Get console ID and baudrate */
	while ((c = getopt(argc, argv, "b:c:")) != -1) {
		switch (c) {
			case 'b':
				if ((baud = libtty_int_to_baudrate(atoi(optarg))) == (speed_t)-1) {
					debug("uart: wrong baudrate value\n");
					return EXIT_FAILURE;
				}
				break;
			case 'c':
				console = atoi(optarg);
				if (console < 0 || console >= UARTS_MAX_CNT) {
					debug("uart: wrong console ID\n");
					return EXIT_FAILURE;
				}
				break;
			default:
				break;
		}
	}

	uart_initClk();
	portCreate(&port);

	for (n = 0; n < UARTS_MAX_CNT; ++n) {
		if (uart_init(n, baud) < 0)
			continue;

		uart = &uart_common.uarts[n];
		uart->oid.port = port;

		/* port = 0 & id = 0 are reserved for CONSOLE */
		uart->oid.id = (n == console) ? 0 : n + 1;
	}

	beginthread(uart_dispatchMsg, 4, uart_common.stack, sizeof(uart_common.stack), (void *)port);
	uart_mkDevFiles();
	uart_dispatchMsg((void *)port);

	return 0;
}
