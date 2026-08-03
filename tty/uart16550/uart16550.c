/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Copyright 2012-2015, 2020-2022 Phoenix Systems
 * Copyright 2001, 2008 Pawel Pisarczyk
 * Authors: Pawel Pisarczyk, Pawel Kolodziej,
 *          Julia Kosowska, Lukasz Kosinski,
 *          Gerard Swiderski et al.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <paths.h>
#include <string.h>
#include <stdatomic.h>
#include <stddef.h>
#include <lf-fifo.h>

#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libtty.h>
#include <libklog.h>
#include <board_config.h>
#include <posix/utils.h>
#include <syslog.h>

#include "uarthw.h"
#include "uart16550.h"

#define KMSG_CTRL_ID 100

#define SW_BUF_SIZE 64

#define N_UARTS 4

#define THREAD_STACK_SIZE 2048

/*
 * Note: different 16550 implementations have different meanings for FIFO threshold settings.
 * Typically 0x2 is 8 characters threshold in a FIFO of 16 characters.
 */
#define UART16550_FIFO_DEFAULT 0x2

#ifndef UART16550_FIFO0
#define UART16550_FIFO0 UART16550_FIFO_DEFAULT
#endif

#ifndef UART16550_FIFO1
#define UART16550_FIFO1 UART16550_FIFO_DEFAULT
#endif

#ifndef UART16550_FIFO2
#define UART16550_FIFO2 UART16550_FIFO_DEFAULT
#endif

#ifndef UART16550_FIFO3
#define UART16550_FIFO3 UART16550_FIFO_DEFAULT
#endif


static const int8_t uart_defaultFifo[N_UARTS] = {
	UART16550_FIFO0,
	UART16550_FIFO1,
	UART16550_FIFO2,
	UART16550_FIFO3,
};


typedef struct {
	uint8_t hwctx[64];
	lf_fifo_t rxSwFifo;
	uint8_t rxSwFifoData[SW_BUF_SIZE];
	atomic_uint_fast8_t lineStatus;

	unsigned int init;
	uarthw_info_t hwInfo;
	atomic_uint_fast32_t hwOverruns;

	handle_t mutex;
	handle_t intcond;
	handle_t inth;

	oid_t oid;
	libtty_common_t tty;

	char stack[THREAD_STACK_SIZE] __attribute__((aligned(8)));
} uart_t;


typedef struct {
	bool isConsole;
	bool reportErrors;
} uart_args_t;


static struct {
	uart_t uarts[N_UARTS];
	char stack[THREAD_STACK_SIZE] __attribute__((aligned(8)));
} uart_common;


static uart_t *uart_get(oid_t *oid)
{
	unsigned int i;

	for (i = 0; i < N_UARTS; i++) {
		if ((uart_common.uarts[i].oid.port == oid->port) && (uart_common.uarts[i].oid.id == oid->id)) {
			return uart_common.uarts + i;
		}
	}

	return NULL;
}


static void set_baudrate(void *_uart, int baud_rate)
{
	uint8_t reg;
	uart_t *uart = (uart_t *)_uart;

	if (baud_rate <= 0) {
		return;
	}

	/* Baud divisor */
	baud_rate = uart->hwInfo.fclk / (16 * baud_rate);

	if (baud_rate > UINT16_MAX) {
		return;
	}

	reg = uarthw_read(uart->hwctx, REG_LCR);

	/* Set baud rate */
	uarthw_write(uart->hwctx, REG_LCR, reg | LCR_DLAB);
	uarthw_write(uart->hwctx, REG_LSB, (uint8_t)((unsigned)baud_rate));
	uarthw_write(uart->hwctx, REG_MSB, (uint8_t)((unsigned)baud_rate >> 8));
	uarthw_write(uart->hwctx, REG_LCR, reg & ~LCR_DLAB);
}


static void set_cflag(void *_uart, tcflag_t *cflag)
{
	uart_t *uart = (uart_t *)_uart;
	uint8_t lcr = uarthw_read(uart->hwctx, REG_LCR);

	lcr &= ~((3 << 0) | (1 << 2) | (1 << 3) | (1 << 4));

	/* Character length */
	switch (*cflag & CSIZE) {
		case CS5: lcr |= 0; break;
		case CS6: lcr |= 1; break;
		case CS7: lcr |= 2; break;
		case CS8: /* fallthrough */
		default: lcr |= 3; break;
	}

	/* Parity */
	lcr |= (((*cflag & PARENB) != 0) << 3) | (((*cflag & PARODD) == 0) << 4);

	/* Stop bits */
	lcr |= ((*cflag & CSTOPB) != 0) << 2;

	uarthw_write(uart->hwctx, REG_LCR, lcr);
}


static void signal_txready(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	condSignal(uart->intcond);
}


/*
 * This function is necessary because Line Status Register contains error bits that are clear-on-read
 * (like overrun bit), but also hardware-cleared bits like data ready bit.
 * We need to store and accumulate error bits until we are ready to handle them.
 */
static uint8_t uart_readUpdateLineStatus(uart_t *uart)
{
	uint8_t lineStatus = uarthw_read(uart->hwctx, REG_LSR);
	atomic_fetch_or(&uart->lineStatus, lineStatus);
	return lineStatus;
}


#ifdef __TARGET_RISCV64
__attribute__((section(".interrupt"))) static int uart_interrupt(unsigned int n, void *arg)
{
	/* RISC-V platform is very special in how it handles memory during interrupts.
	 * Due to this the uart_interrupt function cannot call uarthw_* functions.
	 * Fortunately the UART IRQ on this platform is edge-triggered so we can just
	 * exit interrupt, signal uart->intcond and main thread will handle the rest.
	 */
	return 1;
}
#else
static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	/* Caution: implementation-dependent behavior!
	 * On some UARTs masking interrupts after an interrupt has been asserted
	 * does not de-assert the IRQ line. In this case it is necessary to handle
	 * the interrupt's cause fully within the ISR (e.g. reading the whole FIFO).
	 * On other UARTs masking interrupts de-asserts IRQ and changes the value of IIR.
	 * To handle this case we read IIR before masking interrupts. Note that in this
	 * case the FIFO will not be fully read within the ISR.
	 */
	uint8_t iir = uarthw_read(uart->hwctx, REG_IIR);
	uarthw_write(uart->hwctx, REG_IMR, 0);
	do {
		uint8_t intr_type = (iir >> 1) & 0x7;
		if ((intr_type == IIR_CODE_DR) || (intr_type == IIR_CODE_RTO)) {
			uint8_t status = uart_readUpdateLineStatus(uart);
			while ((status & LSR_DR) != 0) {
				lf_fifo_ow_push(&uart->rxSwFifo, uarthw_read(uart->hwctx, REG_RBR));
				status = uart_readUpdateLineStatus(uart);
			}
		}
		else if (intr_type == IIR_CODE_LS) {
			uart_readUpdateLineStatus(uart);
		}
		else if (intr_type == IIR_CODE_MS) {
			uarthw_read(uart->hwctx, REG_MSR);
		}

		iir = uarthw_read(uart->hwctx, REG_IIR);
	} while ((iir & IIR_IRQPEND) == 0);

	return 1;
}
#endif


static void uart_handleErrors(uart_t *uart)
{
	(void)uart_readUpdateLineStatus(uart);
	uint8_t lineStatus = atomic_exchange(&uart->lineStatus, 0);
	if (lineStatus & LSR_OE) {
		atomic_fetch_add(&uart->hwOverruns, 1);
	}
}


static bool uart_handleTx(uart_t *uart)
{
	bool waitForTx = false;
	int wake = 0;
	while (libtty_txready(&uart->tty) != 0) {
		if ((uart_readUpdateLineStatus(uart) & LSR_THRE) != 0) {
			int wakeHelper;
			uarthw_write(uart->hwctx, REG_THR, libtty_getchar(&uart->tty, &wakeHelper));
			wake |= wakeHelper;
		}
		else {
			waitForTx = true;
			break;
		}
	}

	if (wake != 0) {
		libtty_wake_writer(&uart->tty);
	}

	return waitForTx;
}


static void uart_intthr(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint8_t target_imr = IMR_DR | IMR_LS;

	mutexLock(uart->mutex);
	for (;;) {
		uarthw_write(uart->hwctx, REG_IMR, target_imr);
		uart_handleErrors(uart);
		condWait(uart->intcond, uart->mutex, 0);
		/* For the following part the interrupt needs to be masked */
		uarthw_write(uart->hwctx, REG_IMR, 0);

		if (!lf_fifo_empty(&uart->rxSwFifo) || (uart_readUpdateLineStatus(uart) & LSR_DR) != 0) {
			int wake = 0, wakeHelper = 0;
			libtty_putchar_lock(&uart->tty);
			/* Empty received buffer */
			uint8_t c;
			while (lf_fifo_ow_pop(&uart->rxSwFifo, &c) != 0) {
				libtty_putchar_unlocked(&uart->tty, c, &wakeHelper);
				wake |= wakeHelper;
			}

			/* Depending on implementation we may have more characters in hardware FIFO */
			while ((uart_readUpdateLineStatus(uart) & LSR_DR) != 0) {
				c = uarthw_read(uart->hwctx, REG_RBR);
				libtty_putchar_unlocked(&uart->tty, c, &wakeHelper);
				wake |= wakeHelper;
			}

			libtty_putchar_unlock(&uart->tty);
			if (wake != 0) {
				libtty_wake_reader(&uart->tty);
			}
		}

		/* Check for transmit */
		if (uart_handleTx(uart)) {
			target_imr |= IMR_THRE;
		}
		else {
			target_imr &= ~IMR_THRE;
		}
	}
}


static void uart_ioctl(unsigned int port, msg_t *msg)
{
	const void *idata, *odata = NULL;
	oid_t oid = { .port = port };
	uart_t *uart;
	unsigned long req;
	int err;

	idata = ioctl_unpack(msg, &req, &oid.id);

	uart = uart_get(&oid);
	if (uart == NULL) {
		err = -EINVAL;
	}
	else if (req == KIOEN) {
		if ((UART16550_CONSOLE_USER >= 0) && (uart == &uart_common.uarts[UART16550_CONSOLE_USER])) {
			libklog_enable((int)(intptr_t)idata);
			err = EOK;
		}
		else {
			err = -EINVAL;
		}
	}
	else {
		err = libtty_ioctl(&uart->tty, ioctl_getSenderPid(msg), req, idata, &odata);
	}

	ioctl_setResponse(msg, req, err, odata);
}


static void uart_errorThread(void *arg)
{
	(void)arg;
	uint32_t overrunsLast[N_UARTS];
	memset(&overrunsLast, 0, sizeof(overrunsLast));

	for (;;) {
		for (unsigned int i = 0; i < N_UARTS; i++) {
			uart_t *uart = &uart_common.uarts[i];
			if (uart->init == 0) {
				continue;
			}

			uint32_t overrunsNow = atomic_load(&uart->hwOverruns);
			if (overrunsNow != overrunsLast[i]) {
				syslog(LOG_WARNING, "UART16550 %u: %u overrun(s) detected", i, overrunsNow - overrunsLast[i]);
				overrunsLast[i] = overrunsNow;
			}
		}

		sleep(1);
	}
}


static void poolthr(void *arg)
{
	unsigned int port = (uintptr_t)arg;
	uart_t *uart;
	msg_rid_t rid;
	msg_t msg;

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
			case mtClose:
				uart = uart_get(&msg.oid);
				if (uart == NULL) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.err = EOK;
				break;

			case mtRead:
				uart = uart_get(&msg.oid);
				if (uart == NULL) {
					msg.o.err = -EINVAL;
				}
				else {
					msg.o.err = libtty_read(&uart->tty, msg.o.data, msg.o.size, msg.i.io.mode);
				}
				break;

			case mtWrite:
				uart = uart_get(&msg.oid);
				if (uart == NULL) {
					msg.o.err = -EINVAL;
				}
				else {
					msg.o.err = libtty_write(&uart->tty, msg.i.data, msg.i.size, msg.i.io.mode);
				}
				break;

			case mtGetAttr:
				uart = uart_get(&msg.oid);
				if ((msg.i.attr.type != atPollStatus) || (uart == NULL)) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.attr.val = libtty_poll_status(&uart->tty);
				msg.o.err = EOK;
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
	libtty_write(&uart_common.uarts[UART16550_CONSOLE_USER].tty, data, size, 0);
}


static void _uart_mkDev(uint32_t port, bool isconsole)
{
	char path[12];
	unsigned int i;

	for (i = 0; i < N_UARTS; i++) {
		if (uart_common.uarts[i].init == 1) {
			uart_common.uarts[i].oid.port = port;
			uart_common.uarts[i].oid.id = (i == UART16550_CONSOLE_USER) ? 0 : i + 1;
			snprintf(path, sizeof(path), "/dev/ttyS%u", i);

			if (create_dev(&uart_common.uarts[i].oid, path) < 0) {
				fprintf(stderr, "uart16550: failed to register %s\n", path);
				return;
			}

			if (i == UART16550_CONSOLE_USER) {
				if (isconsole) {
					libklog_init(uart_klogClbk);
					if (create_dev(&uart_common.uarts[i].oid, _PATH_CONSOLE) < 0) {
						fprintf(stderr, "uart16550: failed to register %s\n", _PATH_CONSOLE);
						return;
					}

					oid_t kmsgctrl = { .port = port, .id = KMSG_CTRL_ID };
					libklog_ctrlRegister(&kmsgctrl);
				}
				else {
					libklog_initNoDev(uart_klogClbk);
				}
			}
		}
	}
}


static int _uart_init(uart_t *uart, unsigned int uartn, unsigned int speed, int8_t fifo)
{
	unsigned int divisor;
	libtty_callbacks_t callbacks = {
		.arg = uart,
		.set_baudrate = set_baudrate,
		.set_cflag = set_cflag,
		.signal_txready = signal_txready,
	};

	int err = uarthw_init(uartn, uart->hwctx, sizeof(uart->hwctx), &uart->hwInfo);
	if (err < 0) {
		return err;
	}

	divisor = uart->hwInfo.fclk / (16 * speed);

	err = libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, speed);
	if (err < 0) {
		return err;
	}

	atomic_store(&uart->hwOverruns, 0);
	atomic_store(&uart->lineStatus, 0);
	lf_fifo_init(&uart->rxSwFifo, uart->rxSwFifoData, sizeof(uart->rxSwFifoData));
	err = condCreate(&uart->intcond);
	if (err < 0) {
		libtty_destroy(&uart->tty);
		return err;
	}

	err = mutexCreate(&uart->mutex);
	if (err < 0) {
		resourceDestroy(uart->intcond);
		libtty_destroy(&uart->tty);
		return err;
	}

	/* Set speed (MOD) */
	uarthw_write(uart->hwctx, REG_LCR, LCR_DLAB);
	uarthw_write(uart->hwctx, REG_LSB, divisor);
	uarthw_write(uart->hwctx, REG_MSB, divisor >> 8);

	/* Set data format, leave DLAB on because some 16750 implementations require it to enable extended FIFO settings */
	uarthw_write(uart->hwctx, REG_LCR, LCR_DLAB | LCR_D8N1);

	/* Enable and configure FIFOs */
	if (fifo < 0) {
		uarthw_write(uart->hwctx, REG_FCR, 0);
	}
	else {
		/* On some implementations we need to set FIFOEN separately before any other write can take effect */
		uarthw_write(uart->hwctx, REG_FCR, FCR_FIFOEN);
		uint8_t fifoSetting = (((uint8_t)fifo & 0x3) << 6) | uart->hwInfo.fcr;
		uarthw_write(uart->hwctx, REG_FCR, fifoSetting | FCR_RESETTX | FCR_RESETRX | FCR_FIFOEN);
	}

	/* Set LCR to its final value */
	uarthw_write(uart->hwctx, REG_LCR, LCR_D8N1);

	/* Enable hardware interrupts */
	uarthw_write(uart->hwctx, REG_MCR, MCR_OUT2);

	/* Set interrupt mask */
	uarthw_write(uart->hwctx, REG_IMR, 0);

	err = interrupt(uarthw_irq(uart->hwctx), uart_interrupt, uart, uart->intcond, &uart->inth);
	if (err < 0) {
		uarthw_write(uart->hwctx, REG_MCR, 0);
		resourceDestroy(uart->mutex);
		resourceDestroy(uart->intcond);
		libtty_destroy(&uart->tty);
		return err;
	}

	err = beginthread(uart_intthr, 1, uart->stack, sizeof(uart->stack), uart);
	if (err < 0) {
		resourceDestroy(uart->inth);
		uarthw_write(uart->hwctx, REG_MCR, 0);
		resourceDestroy(uart->mutex);
		resourceDestroy(uart->intcond);
		libtty_destroy(&uart->tty);
		return err;
	}

	return 0;
}


static void print_usage(const char *name)
{
	printf("UART16550 driver. Usage:\n");
	printf("%s [-n] [-e]\n", name);
	printf("\t-n - Skip creating %s device (default: no)\n", _PATH_CONSOLE);
	printf("\t-e - Report data errors to syslog (default: no)\n");
}


static int parse_args(int argc, char *argv[], uart_args_t *args)
{
	int opt;
	args->isConsole = true;
	args->reportErrors = false;

	while ((opt = getopt(argc, argv, "hne")) != -1) {
		switch (opt) {
			case 'n':
				args->isConsole = false;
				break;

			case 'e':
				args->reportErrors = true;
				break;

			case 'h': /* Fall-through */
			default:
				print_usage(argv[0]);
				return -1;
		}
	}

	if (optind < argc) {
		fprintf(stderr, "%s: unrecognized argument: %s\n", argv[0], argv[optind]);
		print_usage(argv[0]);
		return -1;
	}

	return 0;
}


int main(int argc, char *argv[])
{
	unsigned int i;
	uint32_t port;
	int err;
	uart_args_t args;

	if (parse_args(argc, argv, &args) < 0) {
		return EXIT_FAILURE;
	}

	err = portCreate(&port);
	if (err < 0) {
		fprintf(stderr, "uart16550: failed to create port\n");
		return EXIT_FAILURE;
	}

	for (i = 0; i < N_UARTS; i++) {
		err = _uart_init(&uart_common.uarts[i], i, UART16550_BAUDRATE, uart_defaultFifo[i]);
		if (err < 0) {
			if (err != -ENODEV) {
				fprintf(stderr, "uart16550: failed to init ttyS%u, err: %d\n", i, err);
			}
		}
		else {
			uart_common.uarts[i].init = 1;
		}
	}

	if (args.reportErrors) {
		char *stack = malloc(THREAD_STACK_SIZE);
		if (stack == NULL) {
			fprintf(stderr, "uart16550: failed to allocate memory for error thread. Errors will not be reported.\n");
		}
		else {
			err = beginthread(uart_errorThread, 5, stack, THREAD_STACK_SIZE, NULL);
			if (err < 0) {
				fprintf(stderr, "uart16550: failed to start error thread. Errors will not be reported.\n");
				free(stack);
			}
		}
	}

	err = beginthread(poolthr, 4, uart_common.stack, sizeof(uart_common.stack), (void *)(uintptr_t)port);
	if (err < 0) {
		fprintf(stderr, "uart16550: failed to start thread\n");
		return EXIT_FAILURE;
	}

	_uart_mkDev(port, args.isConsole);
	poolthr((void *)(uintptr_t)port);

	return EXIT_SUCCESS;
}
