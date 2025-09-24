/*
 * Phoenix-RTOS
 *
 * GRLIB UART driver
 *
 * Copyright 2023, 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>

#include <board_config.h>
#include <endian.h>
#include <libtty.h>
#include <stdlib.h>
#include <stdatomic.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/time.h>
#include <posix/utils.h>

#include <phoenix/ioctl.h>

#include <phoenix/arch/sparcv8leon/sparcv8leon.h>

#include <grdmac2.h>

#include "uart.h"
#include "grlib-multi.h"


#if !defined(DMA_MAX_CNT) || (UART0_DMA + UART1_DMA + UART2_DMA + UART3_DMA + UART4_DMA + UART5_DMA) > DMA_MAX_CNT
#error "Unsupported DMA configuration"
#endif


#define UART_STACKSZ (4096)

/* UART registers */
#define UART_DATA   0
#define UART_STATUS 1
#define UART_CTRL   2
#define UART_SCALER 3
#define UART_DEBUG  4

/* Byte offset of LSB of UART_DATA register */
#if __BYTE_ORDER == __BIG_ENDIAN
#define UART_DATA_OFFS 3
#elif __BYTE_ORDER == __LITTLE_ENDIAN
#define UART_DATA_OFFS 0
#endif

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

#define DMA_RECEIVE_SIZE    64 /* Should be a power of 2 for modulo operation */
#define DMA_RECEIVE_IRQSIZE 16


typedef struct {
	volatile uint32_t *vbase;
	oid_t oid;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	libtty_common_t tty;

	/* DMA */
	grdma_ctx_t *dmaCtx;
	volatile unsigned char *dmaRxBuf;
	volatile atomic_size_t nextPos;
	volatile atomic_size_t irqCnt;
	size_t lastPos;

	uint8_t stack[UART_STACKSZ] __attribute__((aligned(8)));
} uart_t;


typedef struct {
	grdma_condDescr_t condDescr;
	grdma_dataDescr_t dataDescr;
} __attribute__((packed, aligned(4))) uart_dmaDescr_t;


static struct {
	uint32_t *base;
	unsigned int irq;
	uint8_t txPin;
	uint8_t rxPin;
	uint8_t active;
	uint8_t useDma;
} info[] = {
	{ .txPin = UART0_TX, .rxPin = UART0_RX, .active = UART0_ACTIVE, .useDma = UART0_DMA },
	{ .txPin = UART1_TX, .rxPin = UART1_RX, .active = UART1_ACTIVE, .useDma = UART1_DMA },
	{ .txPin = UART2_TX, .rxPin = UART2_RX, .active = UART2_ACTIVE, .useDma = UART2_DMA },
	{ .txPin = UART3_TX, .rxPin = UART3_RX, .active = UART3_ACTIVE, .useDma = UART3_DMA },
	{ .txPin = UART4_TX, .rxPin = UART4_RX, .active = UART4_ACTIVE, .useDma = UART4_DMA },
	{ .txPin = UART5_TX, .rxPin = UART5_RX, .active = UART5_ACTIVE, .useDma = UART5_DMA }
};


static struct {
	uart_t uart[UART_MAX_CNT];
	uint8_t stack[UART_STACKSZ] __attribute__((aligned(8)));
} uart_common;


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	(void)n;

	/* Check if we have data to receive */
	if ((*(uart->vbase + UART_CTRL) & RX_INT) != 0 && (*(uart->vbase + UART_STATUS) & DATA_READY) != 0) {
		/* Disable irq */
		*(uart->vbase + UART_CTRL) &= ~RX_INT;
	}

	return 1;
}


static int uart_dmaInterrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uart_dmaDescr_t *done = &((uart_dmaDescr_t *)uart->dmaCtx->descr)[uart->nextPos];
	(void)n;

	uart->nextPos = (uart->nextPos + DMA_RECEIVE_IRQSIZE) % DMA_RECEIVE_SIZE;

	for (size_t i = 0; i < DMA_RECEIVE_IRQSIZE; i++) {
		/* Clear descriptor statuses */
		done[i].condDescr.sts = 0;
		done[i].dataDescr.sts = 0;
	}

	uart->irqCnt = 1 + (uart->irqCnt % 4);

	return 1;
}


static size_t uart_dmaGetPos(const uart_t *uart)
{
	uart_dmaDescr_t *done = (uart_dmaDescr_t *)uart->dmaCtx->descr;
	const size_t pos = uart->nextPos;

	for (size_t i = 0; i < (uart->irqCnt * DMA_RECEIVE_IRQSIZE); i++) {
		size_t next = (pos + i) % DMA_RECEIVE_SIZE;
		if ((done[next].dataDescr.sts & 0x1) == 0) {
			return next;
		}
	}

	return pos;
}


static void uart_intThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;

	mutexLock(uart->lock);

	for (;;) {
		while ((libtty_txready(&uart->tty) == 0) && ((*(uart->vbase + UART_STATUS) & DATA_READY) == 0)) {
			condWait(uart->cond, uart->lock, 0);
		}

		int wake = 0;
		if ((*(uart->vbase + UART_STATUS) & DATA_READY) != 0) {
			int wakehelper;
			libtty_putchar_lock(&uart->tty);
			do {
				libtty_putchar_unlocked(&uart->tty, (*(uart->vbase + UART_DATA) & 0xff), &wakehelper);
				wake |= wakehelper;
			} while ((*(uart->vbase + UART_STATUS) & DATA_READY) != 0);
			libtty_putchar_unlock(&uart->tty);

			if (wake != 0) {
				libtty_wake_reader(&uart->tty);
			}
		}

		/* Transmit data until TX TTY buffer is empty or TX FIFO is full */
		wake = 0;
		while ((libtty_txready(&uart->tty) != 0) && ((*(uart->vbase + UART_STATUS) & TX_FIFO_FULL) == 0)) {
			*(uart->vbase + UART_DATA) = libtty_popchar(&uart->tty);
			wake = 1;
		}

		if (wake != 0) {
			libtty_wake_writer(&uart->tty);
		}

		if ((*(uart->vbase + UART_CTRL) & RX_INT) == 0) {
			*(uart->vbase + UART_CTRL) |= RX_INT;
		}
	}

	mutexUnlock(uart->lock);
}


static void uart_dmaThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;

	mutexLock(uart->lock);

	for (;;) {
		size_t pos = uart_dmaGetPos(uart);
		while ((libtty_txready(&uart->tty) == 0) && (uart->lastPos == pos)) {
			(void)condWait(uart->cond, uart->lock, 50000);
			pos = uart_dmaGetPos(uart);
		}

		int wake = 0;
		if (uart->lastPos != pos) {
			libtty_putchar_lock(&uart->tty);
			do {
				int wakeHelper;
				libtty_putchar_unlocked(&uart->tty, uart->dmaRxBuf[uart->lastPos], &wakeHelper);
				uart->lastPos = (uart->lastPos + 1) % DMA_RECEIVE_SIZE;
				wake |= wakeHelper;
			} while (uart->lastPos != pos);

			/* Unroll one loop to speed up RX */
			pos = uart_dmaGetPos(uart);
			if (uart->lastPos != pos) {
				do {
					int wakeHelper;
					libtty_putchar_unlocked(&uart->tty, uart->dmaRxBuf[uart->lastPos], &wakeHelper);
					uart->lastPos = (uart->lastPos + 1) % DMA_RECEIVE_SIZE;
					wake |= wakeHelper;
				} while (uart->lastPos != pos);
			}
			libtty_putchar_unlock(&uart->tty);

			if (wake != 0) {
				libtty_wake_reader(&uart->tty);
			}
		}

		wake = 0;
		while ((libtty_txready(&uart->tty) != 0) && ((*(uart->vbase + UART_STATUS) & TX_FIFO_FULL) == 0)) {
			*(uart->vbase + UART_DATA) = libtty_popchar(&uart->tty);
			wake = 1;
		}

		if (wake != 0) {
			libtty_wake_writer(&uart->tty);
		}
	}
}


static void uart_setBaudrate(void *data, int speed)
{
	uart_t *uart = (uart_t *)data;
	uint32_t scaler = (UART_CLK / (speed * 8 + 7));

	*(uart->vbase + UART_SCALER) = scaler;
}


static void uart_setCFlag(void *data, tcflag_t *cflag)
{
	uart_t *uart = (uart_t *)data;

	/* Parity */
	if ((*cflag & PARENB) != 0) {
		*(uart->vbase + UART_CTRL) |= (PARITY_EN | PARITY_ODD);
		if ((*cflag & PARODD) == 0) {
			*(uart->vbase + UART_CTRL) &= ~PARITY_ODD;
		}
	}
	else {
		*(uart->vbase + UART_CTRL) &= ~PARITY_EN;
	}

	/* Stop bits */
	if ((*cflag & CSTOPB) != 0) {
		*(uart->vbase + UART_CTRL) |= STOP_BITS;
	}
	else {
		*(uart->vbase + UART_CTRL) &= ~STOP_BITS;
	}
}


static void uart_signalTXReady(void *data)
{
	uart_t *uart = (uart_t *)data;

	condSignal(uart->cond);
}


static void uart_ioctl(msg_t *msg, int dev)
{
	int err;
	pid_t pid;
	unsigned long req;
	const void *inData, *outData = NULL;

	inData = ioctl_unpack(msg, &req, NULL);
	pid = ioctl_getSenderPid(msg);

	err = libtty_ioctl(&uart_common.uart[dev].tty, pid, req, inData, &outData);

	ioctl_setResponse(msg, req, err, outData);
}


static int uart_cguInit(unsigned int n)
{
#if defined(__CPU_GR716)
	platformctl_t pctl;
	static const unsigned int cguinfo[] = {
		cgudev_apbuart0,
		cgudev_apbuart1,
		cgudev_apbuart2,
		cgudev_apbuart3,
		cgudev_apbuart4,
		cgudev_apbuart5
	};

	pctl.action = pctl_set;
	pctl.type = pctl_cguctrl;

	pctl.task.cguctrl.v.state = enable;
	pctl.task.cguctrl.cgu = cgu_primary;
	pctl.task.cguctrl.cgudev = cguinfo[n];

	return platformctl(&pctl);
#elif defined(__CPU_GR740)
	static const unsigned int cguinfo[] = {
		cgudev_apbuart0,
		cgudev_apbuart1
	};
	platformctl_t ctl = {
		.action = pctl_get,
		.type = pctl_cguctrl,
		.task.cguctrl.cgudev = cguinfo[n]
	};

	if (platformctl(&ctl) < 0) {
		return -1;
	}

	if (ctl.task.cguctrl.v.stateVal == 1) {
		return 0;
	}

	ctl.action = pctl_set;
	ctl.type = pctl_cguctrl;

	ctl.task.cguctrl.v.state = enable;
	ctl.task.cguctrl.cgudev = cguinfo[n];

	return platformctl(&ctl);
#else
	return 0;
#endif
}


static int uart_dmaSetup(uart_t *uart)
{
	*(uart->vbase + UART_CTRL) = RX_EN | TX_EN;

	grdma_ctx_t *ctx = grdma_init(0);
	if (ctx == NULL) {
		return -1;
	}

	if (grdma_descrAlloc(ctx, (sizeof(grdma_condDescr_t) + sizeof(grdma_dataDescr_t)) * DMA_RECEIVE_SIZE) < 0) {
		grdma_destroy(ctx);
		return -1;
	}

	uart->dmaRxBuf = malloc(DMA_RECEIVE_SIZE * sizeof(unsigned char));

	if (uart->dmaRxBuf == NULL) {
		grdma_destroy(ctx);
		return -1;
	}

	uart->dmaCtx = ctx;
	uart->nextPos = 0;
	uart->lastPos = 0;
	uart->irqCnt = 1;

	uart_dmaDescr_t *descr = ctx->descr;

	for (size_t i = 0; i < DMA_RECEIVE_SIZE; i++) {
		descr[i].condDescr.ctrl = GRDMA_COND_EN | GRDMA_DESC_TYPE(1) | GRDMA_COND_INTRV(0xff) | GRDMA_COND_CNT(0xff) | GRDMA_COND_WB;
		descr[i].condDescr.next = (uintptr_t)&descr[i].dataDescr & ~0x1;
		descr[i].condDescr.nextFail = (uintptr_t)&descr[i].condDescr & ~0x1;
		descr[i].condDescr.poll = (uintptr_t)(uart->vbase + UART_STATUS);
		descr[i].condDescr.expData = DATA_READY;
		descr[i].condDescr.mask = DATA_READY;
		descr[i].condDescr.sts = 0;

		descr[i].dataDescr.ctrl = GRDMA_DATA_EN | GRDMA_DESC_TYPE(0) | GRDMA_DATA_SF | GRDMA_DATA_SZ(1) | GRDMA_DATA_WB;
		if ((i % DMA_RECEIVE_IRQSIZE) == (DMA_RECEIVE_IRQSIZE - 1)) {
			descr[i].dataDescr.ctrl |= GRDMA_DATA_IE;
		}
		if (i == DMA_RECEIVE_SIZE - 1) {
			descr[i].dataDescr.next = (uintptr_t)ctx->descr & ~0x1;
		}
		else {
			descr[i].dataDescr.next = (uintptr_t)&descr[i + 1] & ~0x1;
		}
		descr[i].dataDescr.dest = (uintptr_t)&uart->dmaRxBuf[i];
		descr[i].dataDescr.src = ((uintptr_t)(uart->vbase + UART_DATA)) + UART_DATA_OFFS;
		descr[i].dataDescr.sts = 0;
	}

	interrupt(ctx->irq, uart_dmaInterrupt, uart, uart->cond, &uart->inth);

	grdma_setup(ctx, ctx->descr);
	grdma_start(ctx);

	return 0;
}


static int uart_irqSetup(unsigned int n, uart_t *uart)
{
	*(uart->vbase + UART_CTRL) = RX_INT | RX_EN | TX_EN;

	interrupt(info[n].irq, uart_interrupt, uart, uart->cond, &uart->inth);

	return 0;
}


static int uart_setup(unsigned int n, int baud, int raw)
{
	libtty_callbacks_t callbacks;
	uart_t *uart = &uart_common.uart[n];
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_iomux,
		.task.iocfg = {
			.opt = 0x1,
			.pin = info[n].rxPin,
			.pullup = 0,
			.pulldn = 0,
		}
	};

	if (platformctl(&pctl) < 0) {
		return -1;
	}

	pctl.task.iocfg.pin = info[n].txPin;

	if (platformctl(&pctl) < 0) {
		return -1;
	}

	if (uart_cguInit(n) < 0) {
		return -1;
	}

	uintptr_t base = ((uintptr_t)info[n].base) & ~(_PAGE_SIZE - 1);
	uart->vbase = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (uart->vbase == MAP_FAILED) {
		return -1;
	}

	callbacks.arg = uart;
	callbacks.set_cflag = uart_setCFlag;
	callbacks.set_baudrate = uart_setBaudrate;
	callbacks.signal_txready = uart_signalTXReady;

	if (libtty_init(&uart->tty, &callbacks, _PAGE_SIZE, baud) < 0) {
		munmap((void *)uart->vbase, _PAGE_SIZE);
		return -1;
	}

	if (condCreate(&uart->cond) != EOK) {
		munmap((void *)uart->vbase, _PAGE_SIZE);
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		return -1;
	}

	if (mutexCreate(&uart->lock) != EOK) {
		munmap((void *)uart->vbase, _PAGE_SIZE);
		libtty_close(&uart->tty);
		libtty_destroy(&uart->tty);
		resourceDestroy(uart->cond);
		return -1;
	}

	/* Set raw mode */
	if (raw == 1) {
		libtty_set_mode_raw(&uart->tty);
	}

	uart->vbase += ((uintptr_t)info[n].base - base) / sizeof(uintptr_t);

	*(uart->vbase + UART_CTRL) = 0;
	*(uart->vbase + UART_SCALER) = 0;
	*(uart->vbase + UART_STATUS) = 0;

	/* Clear UART FIFO */
	while ((*(uart->vbase + UART_STATUS) & DATA_READY) != 0) {
		(void)*(uart->vbase + UART_DATA);
	}

	/* normal mode, 1 stop bit, no parity, 8 bits */
	uart_setCFlag(uart, &uart->tty.term.c_cflag);

	uart->tty.term.c_ispeed = uart->tty.term.c_ospeed = baud;
	uart_setBaudrate(uart, baud);

	if (info[n].useDma == 1) {
		if (uart_dmaSetup(uart) == 0) {
			beginthread(uart_dmaThread, 1, &uart->stack, sizeof(uart->stack), (void *)uart);
			return 0;
		}
	}
	else {
		if (uart_irqSetup(n, uart) == 0) {
			beginthread(uart_intThread, 2, &uart->stack, sizeof(uart->stack), (void *)uart);
			return 0;
		}
	}

	return -1;
}


void uart_handleMsg(msg_t *msg, int dev)
{
	dev -= id_uart0;

	if (info[dev].active == 0) {
		msg->o.err = -EINVAL;
		return;
	}

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = EOK;
			break;

		case mtWrite:
			msg->o.err = libtty_write(&uart_common.uart[dev].tty, msg->i.data, msg->i.size, msg->i.io.mode);
			break;

		case mtRead:
			msg->o.err = libtty_read(&uart_common.uart[dev].tty, msg->o.data, msg->o.size, msg->i.io.mode);
			break;

		case mtGetAttr:
			if (msg->i.attr.type == atPollStatus) {
				msg->o.attr.val = libtty_poll_status(&uart_common.uart[dev].tty);
				msg->o.err = EOK;
			}
			else {
				msg->o.err = -ENOSYS;
			}
			break;

		case mtDevCtl:
			uart_ioctl(msg, dev);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


void uart_klogClbk(const char *data, size_t size)
{
	libtty_write(&uart_common.uart[UART_CONSOLE_USER].tty, data, size, 0);
}


int uart_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < UART_MAX_CNT; i++) {
		if (info[i].active == 0) {
			continue;
		}

		char buf[8];
		if (snprintf(buf, sizeof(buf), "uart%u", i) >= sizeof(buf)) {
			return -1;
		}

		oid->id = id_uart0 + i;
		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


int uart_init(void)
{
	int raw = 0;
	int baud = 115200;

	for (unsigned int n = 0; n < UART_MAX_CNT; n++) {
		if (info[n].active == 0) {
			continue;
		}

		unsigned int instance = n;
		ambapp_dev_t dev = { .devId = CORE_ID_APBUART };
		platformctl_t pctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.task.ambapp = {
				.dev = &dev,
				.instance = &instance,
			}
		};

		if (platformctl(&pctl) < 0) {
			return -1;
		}

		if (dev.bus != BUS_AMBA_APB) {
			/* APBUART should be on APB bus */
			return -1;
		}
		info[n].base = dev.info.apb.base;
		info[n].irq = dev.irqn;

		if (uart_setup(n, baud, raw) < 0) {
			return -1;
		}
	}

	return 0;
}
