/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * QEMU ARM Virt (PL011) User-Space UART driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Hubert Buczynski
 * Ported to PL011 / QEMU Virt
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

#ifndef UART_CONSOLE_USER
#define UART_CONSOLE_USER 0
#endif

#define UARTS_MAX_CNT 1
#define KMSG_CTRL_ID 100

typedef struct {
	volatile uint32_t *base;
	oid_t oid;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	libtty_common_t tty;

	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(16)));
} uart_t;

static const struct {
	uint32_t base;
	unsigned int irq;
} info[UARTS_MAX_CNT] = {
	{ 0x09000000, 33 } /* PL011 UART0 on QEMU Virt */
};

static struct {
	uart_t uart;
	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(16)));
} uart_common;

/* PL011 Register Offsets (in 32-bit words) */
enum {
	dr = 0x00,    /* Data Register */
	rsrecr = 0x01,/* Receive Status/Error Clear Register */
	fr = 0x06,    /* Flag Register */
	ibrd = 0x09,  /* Integer Baud Rate */
	fbrd = 0x0a,  /* Fractional Baud Rate */
	lcr_h = 0x0b, /* Line Control */
	cr = 0x0c,    /* Control Register */
	ifls = 0x0d,  /* Interrupt FIFO Level Select */
	imsc = 0x0e,  /* Interrupt Mask Set/Clear */
	ris = 0x0f,   /* Raw Interrupt Status */
	mis = 0x10,   /* Masked Interrupt Status */
	icr = 0x11,   /* Interrupt Clear */
};

/* PL011 Bit Masks */
#define PL011_FR_TXFF  (1 << 5) /* Transmit FIFO Full */
#define PL011_FR_RXFE  (1 << 4) /* Receive FIFO Empty */
#define PL011_INT_RX   (1 << 4) /* Receive Interrupt */
#define PL011_INT_RT   (1 << 6) /* Receive Timeout Interrupt */

static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint32_t st = *(uart->base + mis);

	if (st & (PL011_INT_RX | PL011_INT_RT)) {
		*(uart->base + imsc) &= ~(PL011_INT_RX | PL011_INT_RT);
		return 1;
	}
	return 0;
}

static void uart_intThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	int wake;
	uint32_t status;
	int count;

	mutexLock(uart->lock);

	for (;;) {
		/* Sleep until there is work to do */
		while (!libtty_txready(&uart->tty) && (*(uart->base + fr) & PL011_FR_RXFE)) {
			condWait(uart->cond, uart->lock, 0);
		}

		/* BOUNDED RX LOOP: Read max 32 chars to prevent infinite CPU lockup */
		count = 32;
		while (count--) {
			status = *(uart->base + fr);
			if (status & PL011_FR_RXFE) {
				break;
			}
			libtty_putchar(&uart->tty, *(uart->base + dr) & 0xFF, NULL);
			*(uart->base + rsrecr) = 0xFF; /* Force clear hardware errors */
		}

		/* BOUNDED TX LOOP: Transmit max 32 chars */
		wake = 0;
		count = 32;
		while (count-- && libtty_txready(&uart->tty)) {
			status = *(uart->base + fr);
			if (status & PL011_FR_TXFF) {
				break;
			}
			*(uart->base + dr) = libtty_popchar(&uart->tty);
			wake = 1;
		}

		if (wake) {
			libtty_wake_writer(&uart->tty);
		}

		/* Clear and re-enable interrupts */
		*(uart->base + icr) = (PL011_INT_RX | PL011_INT_RT);
		*(uart->base + imsc) |= (PL011_INT_RX | PL011_INT_RT);
	}

	mutexUnlock(uart->lock);
}

static void uart_setCFlag(void *data, tcflag_t *cflag)
{
	uart_t *uart = (uart_t *)data;
	uint32_t lcr = *(uart->base + lcr_h) & ~0x60;

	if ((*cflag & CSIZE) == CS6) {
		lcr |= (1 << 5);
	}
	else if ((*cflag & CSIZE) == CS7) {
		lcr |= (2 << 5);
	}
	else {
		*cflag &= ~CSIZE;
		*cflag |= CS8;
		lcr |= (3 << 5);
	}

	*(uart->base + lcr_h) = lcr;
}

static void uart_setBaudrate(void *data, int baudrate)
{
	uart_t *uart = (uart_t *)data;
	*(uart->base + ibrd) = 1;
	*(uart->base + fbrd) = 0;
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

	if (req == KIOEN) {
		if (UART_CONSOLE_USER >= 0) {
			libklog_enable((int)(intptr_t)inData);
			err = EOK;
		}
		else {
			err = -EINVAL;
		}
	}
	else {
		err = libtty_ioctl(&uart_common.uart.tty, pid, req, inData, &outData);
	}

	ioctl_setResponse(msg, req, err, outData);
}

static void uart_dispatchMsg(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	uint32_t port = uart_common.uart.oid.port;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0) {
			usleep(10000);
			continue;
		}

		if (libklog_ctrlHandle(port, &msg, rid) == 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;
			case mtWrite:
				msg.o.err = libtty_write(&uart_common.uart.tty, msg.i.data, msg.i.size, msg.i.io.mode);
				break;
			case mtRead:
				msg.o.err = libtty_read(&uart_common.uart.tty, msg.o.data, msg.o.size, msg.i.io.mode);
				break;
			case mtGetAttr:
				if (msg.i.attr.type == atPollStatus) {
					msg.o.attr.val = libtty_poll_status(&uart_common.uart.tty);
					msg.o.err = EOK;
					break;
				}
				msg.o.err = -ENOSYS;
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
	libtty_write(&uart_common.uart.tty, data, size, 0);
}

static void uart_mkDev(unsigned int id)
{
	char path[20];

	snprintf(path, sizeof(path), "/dev/uart%u", id);
	if (create_dev(&uart_common.uart.oid, path) < 0) {
		debug("virt-uart: cannot create device file\n");
	}

	if (id == UART_CONSOLE_USER) {
		libklog_init(uart_klogClbk);

		if (create_dev(&uart_common.uart.oid, _PATH_CONSOLE) < 0) {
			debug("virt-uart: cannot create console device file\n");
		}

		oid_t kmsgctrl = { .port = uart_common.uart.oid.port, .id = KMSG_CTRL_ID };
		libklog_ctrlRegister(&kmsgctrl);
	}
}

static int uart_init(unsigned int n, int baud, int raw)
{
	libtty_callbacks_t callbacks;
	uart_t *uart = &uart_common.uart;

	/* Removed MAP_UNCACHED, relies on MAP_DEVICE. */
	uart->base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, info[n].base);
	
	/* STRICT NULL CHECK: If this triggers, your kernel's sys_mmap is failing */
	if (uart->base == MAP_FAILED || uart->base == NULL) {
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

	if (condCreate(&uart->cond) != EOK || mutexCreate(&uart->lock) != EOK) {
		return -ENOENT;
	}

	if (raw) {
		libtty_set_mode_raw(&uart->tty);
	}

	/* Initialize Hardware */
	*(uart->base + cr) = 0;
	*(uart->base + icr) = 0x7FF;
	
	uart_setBaudrate(uart, baud);
	
	*(uart->base + lcr_h) = 0x70;
	uart_setCFlag(uart, &uart->tty.term.c_cflag);
	*(uart->base + ifls) = 0;
	*(uart->base + cr) = (1 << 0) | (1 << 8) | (1 << 9);

	*(uart->base + imsc) = (PL011_INT_RX | PL011_INT_RT);

	beginthread(uart_intThread, 4, &uart->stack, sizeof(uart->stack), (void *)uart);
	interrupt(info[n].irq, uart_interrupt, uart, uart->cond, &uart->inth);

	return EOK;
}

int main(int argc, char **argv)
{
	volatile int trap = 0;
	while (trap) {
		/* Spin forever until GDB rescues us */
	}
	int uartn = UART_CONSOLE_USER;
	int baud = 115200;
	int raw = 0;

	portCreate(&uart_common.uart.oid.port);

	if (uart_init(uartn, baud, raw) < 0) {
		/* If you see this in your logs, the MMU mapping is fundamentally failing */
		debug("virt-uart: cannot initialize uart memory\n");
		return EXIT_FAILURE;
	}

	beginthread(uart_dispatchMsg, 4, uart_common.stack, sizeof(uart_common.stack), NULL);
	uart_mkDev(uartn);
	while (trap) {
		/* Spin forever until GDB rescues us */
	}
	uart_dispatchMsg(NULL);

	return EXIT_SUCCESS;
}
