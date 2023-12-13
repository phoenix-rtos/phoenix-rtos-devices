/*
 * Phoenix-RTOS
 *
 * sparcv8leon3-grlib UART driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <paths.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <board_config.h>
#include <libtty.h>
#include <libklog.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <posix/utils.h>

#include <phoenix/ioctl.h>

#if defined(__CPU_GR716)
#include <phoenix/arch/gr716.h>
#elif defined(__CPU_GR712RC)
#include <phoenix/arch/gr712rc.h>
#else
#error "Unsupported target"
#endif

#include <phoenix/arch/sparcv8leon3.h>

#define UART_STACKSZ (4096)

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

#define KMSG_CTRL_ID 100


enum {
	uart_data,   /* Data register           : 0x00 */
	uart_status, /* Status register         : 0x04 */
	uart_ctrl,   /* Control register        : 0x08 */
	uart_scaler, /* Scaler reload register  : 0x0C */
	uart_dbg     /* FIFO debug register     : 0x10 */
};


typedef struct {
	volatile uint32_t *base;
	oid_t oid;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	libtty_common_t tty;

	uint8_t stack[UART_STACKSZ] __attribute__((aligned(8)));
} uart_t;


static struct {
	uint32_t *base;
	unsigned int irq;
	uint8_t txPin;
	uint8_t rxPin;
} info[] = {
	{ .txPin = UART0_TX, .rxPin = UART0_RX },
	{ .txPin = UART1_TX, .rxPin = UART1_RX },
	{ .txPin = UART2_TX, .rxPin = UART2_RX },
	{ .txPin = UART3_TX, .rxPin = UART3_RX },
	{ .txPin = UART4_TX, .rxPin = UART4_RX },
	{ .txPin = UART5_TX, .rxPin = UART5_RX }
};


static struct {
	uart_t uart;
	uint8_t stack[UART_STACKSZ] __attribute__((aligned(8)));
} uart_common;


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	/* Check if we have data to receive */
	if ((*(uart->base + uart_ctrl) & RX_INT) != 0 && (*(uart->base + uart_status) & DATA_READY) != 0) {
		/* Disable irq */
		*(uart->base + uart_ctrl) &= ~RX_INT;
	}

	return 1;
}


static void uart_intThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	int wake;

	mutexLock(uart->lock);

	for (;;) {
		while (libtty_txready(&uart->tty) == 0 && (*(uart->base + uart_status) & DATA_READY) == 0) {
			condWait(uart->cond, uart->lock, 0);
		}

		/* Receive data until RX FIFO is not empty */
		while ((*(uart->base + uart_status) & DATA_READY) != 0) {
			libtty_putchar(&uart->tty, (*(uart->base + uart_data) & 0xff), NULL);
		}

		/* Transmit data until TX TTY buffer is empty or TX FIFO is full */
		wake = 0;
		while (libtty_txready(&uart->tty) != 0 && (*(uart->base + uart_status) & TX_FIFO_FULL) == 0) {
			*(uart->base + uart_data) = libtty_popchar(&uart->tty);
			wake = 1;
		}

		if (wake == 1) {
			libtty_wake_writer(&uart->tty);
		}

		/* If RX int is disabled */
		if ((*(uart->base + uart_ctrl) & RX_INT) == 0) {
			/* Enable RX int */
			*(uart->base + uart_ctrl) |= RX_INT;
		}
	}

	mutexUnlock(uart->lock);
}


static void uart_setBaudrate(void *data, speed_t speed)
{
	uart_t *uart = (uart_t *)data;
	uint32_t scaler = (UART_CLK / (libtty_baudrate_to_int(speed) * 8 + 7));

	*(uart->base + uart_scaler) = scaler;
}


static void uart_setCFlag(void *data, tcflag_t *cflag)
{
	uart_t *uart = (uart_t *)data;

	/* Parity */
	if ((*cflag & PARENB) != 0) {
		*(uart->base + uart_ctrl) |= (PARITY_EN | PARITY_ODD);
		if ((*cflag & PARODD) == 0) {
			*(uart->base + uart_ctrl) &= ~PARITY_ODD;
		}
	}
	else {
		*(uart->base + uart_ctrl) &= ~PARITY_EN;
	}

	/* Stop bits */
	if ((*cflag & CSTOPB) != 0) {
		*(uart->base + uart_ctrl) |= STOP_BITS;
	}
	else {
		*(uart->base + uart_ctrl) &= ~STOP_BITS;
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
				msg.o.io.err = EOK;
				break;

			case mtClose:
				msg.o.io.err = EOK;
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
	char path[7];

	snprintf(path, sizeof(path), "uart%u", id);
	if (create_dev(&uart_common.uart.oid, path) < 0) {
		debug("grlib-uart: Cannot create device file.\n");
		return;
	}

	if (id == UART_CONSOLE_USER) {
		oid_t kmsg = { .port = uart_common.uart.oid.port, .id = KMSG_CTRL_ID };

		libklog_init(uart_klogClbk);

		if (create_dev(&uart_common.uart.oid, _PATH_CONSOLE) < 0) {
			debug("grlib-uart: Cannot create device file.\n");
		}

		if (libklog_ctrlRegister(&kmsg) < 0) {
			debug("grlib-uart: Cannot create kmsg control device file.\n");
		}
	}
}


static int uart_cguInit(unsigned int n)
{
#if defined(__CPU_GR716)
	platformctl_t ctl;
	static const unsigned int cguinfo[] = {
		cgudev_apbuart0,
		cgudev_apbuart1,
		cgudev_apbuart2,
		cgudev_apbuart3,
		cgudev_apbuart4,
		cgudev_apbuart5
	};

	ctl.action = pctl_set;
	ctl.type = pctl_cguctrl;

	ctl.cguctrl.state = enable;
	ctl.cguctrl.cgu = cgu_primary;
	ctl.cguctrl.cgudev = cguinfo[n];

	return platformctl(&ctl);
#else
	return 0;
#endif
}


static int uart_init(unsigned int n, speed_t baud, int raw)
{
	libtty_callbacks_t callbacks;
	uart_t *uart = &uart_common.uart;
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

	if (uart_cguInit(n) < 0) {
		return -1;
	}

	/* Get info from AMBA PnP about APBUART */
	unsigned int instance = n;
	ambapp_dev_t dev = { .devId = CORE_ID_APBUART };

	ctl.action = pctl_get;
	ctl.type = pctl_ambapp;
	ctl.ambapp.dev = &dev;
	ctl.ambapp.instance = &instance;

	if (platformctl(&ctl) < 0) {
		return -1;
	}

	if (dev.bus != BUS_AMBA_APB) {
		/* APBUART should be on APB bus */
		return -1;
	}
	info[n].base = dev.info.apb.base;
	info[n].irq = dev.irqn;

	uintptr_t base = ((uintptr_t)info[n].base) & ~(_PAGE_SIZE - 1);
	uart->base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (uart->base == MAP_FAILED) {
		return -1;
	}

	callbacks.arg = uart;
	callbacks.set_cflag = uart_setCFlag;
	callbacks.set_baudrate = uart_setBaudrate;
	callbacks.signal_txready = uart_signalTXReady;

	if (libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, baud) < 0) {
		munmap((void *)uart->base, _PAGE_SIZE);
		return -1;
	}

	if (condCreate(&uart->cond) != EOK) {
		munmap((void *)uart->base, _PAGE_SIZE);
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		return -1;
	}

	if (mutexCreate(&uart->lock) != EOK) {
		munmap((void *)uart->base, _PAGE_SIZE);
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		resourceDestroy(uart->cond);
		return -1;
	}

	uart->base += ((uintptr_t)info[n].base - base) / sizeof(uintptr_t);

	/* Set raw mode */
	if (raw == 1) {
		libtty_set_mode_raw(&uart->tty);
	}

	*(uart->base + uart_ctrl) = 0;
	*(uart->base + uart_scaler) = 0;

	/* Clear UART FIFO */
	while ((*(uart->base + uart_status) & (1 << 0)) != 0) {
		(void)*(uart->base + uart_data);
	}

	/* normal mode, 1 stop bit, no parity, 8 bits */
	uart_setCFlag(uart, &uart->tty.term.c_cflag);

	uart->tty.term.c_ispeed = uart->tty.term.c_ospeed = baud;
	uart_setBaudrate(uart, baud);
	*(uart->base + uart_ctrl) = RX_INT | RX_EN | TX_EN;

	beginthread(uart_intThread, 2, &uart->stack, sizeof(uart->stack), (void *)uart);
	interrupt(info[n].irq, uart_interrupt, uart, uart->cond, &uart->inth);

	return EOK;
}


static void uart_usage(const char *progname)
{
	printf("Usage: %s [options]\n", progname);
	printf("Options:\n");
	printf("\t-b <baudrate>   - baudrate\n");
	printf("\t-n <id>         - uart controller id\n");
	printf("\t-r              - set raw mode (default cooked)\n");
	printf("\t-h              - print this message\n");
}


int main(int argc, char **argv)
{
	int uartn = UART_CONSOLE_USER;
	speed_t baud = B115200;
	int c, raw = 0;

	if (argc > 1) {
		while ((c = getopt(argc, argv, "n:b:rh")) != -1) {
			switch (c) {
				case 'b':
					baud = libtty_int_to_baudrate(atoi(optarg));
					if (baud == (speed_t)-1) {
						debug("grlib-uart: wrong baudrate value\n");
						return EXIT_FAILURE;
					}
					break;

				case 'n':
					uartn = atoi(optarg);
					if ((uartn >= UART_MAX_CNT) || (uartn < 0)) {
						debug("grlib-uart: wrong uart ID\n");
						return EXIT_FAILURE;
					}
					break;

				case 'r':
					raw = 1;
					break;

				case 'h':
					uart_usage(argv[0]);
					return EXIT_SUCCESS;

				default:
					uart_usage(argv[0]);
					return EXIT_FAILURE;
			}
		}
	}

	portCreate(&uart_common.uart.oid.port);

	if (uart_init(uartn, baud, raw) < 0) {
		debug("grlib-uart: cannot initialize uart\n");
		return EXIT_FAILURE;
	}

	beginthread(uart_dispatchMsg, 3, uart_common.stack, sizeof(uart_common.stack), NULL);
	uart_mkDev(uartn);
	uart_dispatchMsg(NULL);

	return EXIT_SUCCESS;
}
