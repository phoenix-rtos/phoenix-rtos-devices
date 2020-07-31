/*
 * Phoenix-RTOS
 *
 * iMX RT UART driver
 *
 * Copyright 2017-2019 Phoenix Systems
 * Author: Kamil Amanowicz, Marek Bialowas, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/threads.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include <libtty.h>

#include "common.h"
#include "uart.h"


#define UART1_POS 0
#define UART2_POS (UART1_POS + UART1)
#define UART3_POS (UART2_POS + UART2)
#define UART4_POS (UART3_POS + UART3)
#define UART5_POS (UART4_POS + UART4)
#define UART6_POS (UART5_POS + UART5)
#define UART7_POS (UART6_POS + UART6)
#define UART8_POS (UART7_POS + UART7)

#define UART_CNT (UART1 + UART2 + UART3 + UART4 + UART5 + UART6 + UART7 + UART8)

#define BUFSIZE 512


typedef struct uart_s {
	char stack[1024] __attribute__ ((aligned(8)));

	volatile uint32_t *base;
	uint32_t mode;
	uint16_t dev_no;

	handle_t cond;
	handle_t inth;
	handle_t lock;

	size_t rxFifoSz;
	size_t txFifoSz;

	libtty_common_t tty_common;
} uart_t;


struct {
	uart_t uarts[UART_CNT];
	unsigned int port;
} uart_common;


static const int uartConfig[] = { UART1, UART2, UART3, UART4, UART5, UART6, UART7, UART8 };


static const int uartPos[] = { UART1_POS, UART2_POS, UART3_POS, UART4_POS, UART5_POS, UART6_POS,
	UART7_POS, UART8_POS };


enum { veridr = 0, paramr, globalr, pincfgr, baudr, statr, ctrlr, datar, matchr, modirr, fifor, waterr };


static int uart_handleIntr(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;

	*(uart->base + ctrlr) &= ~((1 << 23) | (1 << 21));

	return uart->cond;
}


static inline int uart_getRXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 24) & 0xff;
}


static inline int uart_getTXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 8) & 0xff;
}


static void uart_intrThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;

	for (;;) {
		/* wait for character or transmit data */
		mutexLock(uart->lock);
		while (!uart_getRXcount(uart)) { /* nothing to RX */
			if (libtty_txready(&uart->tty_common)) { /* something to TX */
				if (uart_getTXcount(uart) < uart->txFifoSz) /* TX ready */
					break;
				else
					*(uart->base + ctrlr) |= 1 << 23;
			}

			*(uart->base + ctrlr) |= 1 << 21;

			condWait(uart->cond, uart->lock, 0);
		}

		mutexUnlock(uart->lock);

		/* RX */
		while (uart_getRXcount(uart))
			libtty_putchar(&uart->tty_common, *(uart->base + datar), NULL);

		/* TX */
		while (libtty_txready(&uart->tty_common) && uart_getTXcount(uart) < uart->txFifoSz)
			*(uart->base + datar) = libtty_getchar(&uart->tty_common, NULL);
	}
}


static void signal_txready(void *_uart)
{
	uart_t *uartptr = (uart_t *)_uart;
	condSignal(uartptr->cond);
}


static void set_cflag(void *_uart, tcflag_t* cflag)
{
	uart_t *uartptr = (uart_t *)_uart;
	uint32_t t;

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	/* CSIZE ony CS7 and CS8 (default) is supported */
	if ((*cflag & CSIZE) == CS7) {
		*(uartptr->base + ctrlr) |= 1 << 11;
	}
	else { /* CS8 */
		*cflag &= ~CSIZE;
		*cflag |= CS8;

		*(uartptr->base + ctrlr) &= ~(1 << 11);
	}

	/* parity */
	t = *(uartptr->base + ctrlr) & ~3;
	if (*cflag & PARENB) {
		t |= 1 << 1;
		if (*cflag & PARODD)
			t |= 1;
	}
	*(uartptr->base + ctrlr) = t;

	/* stop bits */
	if (*cflag & CSTOPB)
		*(uartptr->base + baudr) |= (1 << 13);
	else
		*(uartptr->base + baudr) &= ~(1 << 13);

	/* reenable TX and RX */
	*(uartptr->base + ctrlr) |= (1 << 19) | (1 << 18);
}


static uint32_t calculate_baudrate(speed_t baud)
{
	int osr, sbr, bestSbr = 0, bestOsr = 0, bestErr = 1000, t, baud_rate = libtty_baudrate_to_int(baud);

	if (!baud_rate)
		return 0;

	for (osr = 3; osr < 32; ++osr) {
		sbr = UART_CLK / (baud_rate * (osr + 1));
		sbr &= 0xfff;
		t = UART_CLK / (sbr * (osr + 1));

		if (t > baud_rate)
			t = ((t - baud_rate) * 1000) / baud_rate;
		else
			t = ((baud_rate - t) * 1000) / baud_rate;

		if (t < bestErr) {
			bestErr = t;
			bestOsr = osr;
			bestSbr = sbr;
		}

		/* Finish if error is < 1% */
		if (bestErr < 10)
			break;
	}

	return (bestOsr << 24) | ((bestOsr <= 6) << 17) | bestSbr;
}


static void set_baudrate(void *_uart, speed_t baud)
{
	uint32_t reg, t;
	uart_t *uartptr = (uart_t *)_uart;

	reg = calculate_baudrate(baud);

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	t = *(uartptr->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0xfff);
	*(uartptr->base + baudr) = t | reg;

	/* reenable TX and RX */
	*(uartptr->base + ctrlr) |= (1 << 19) | (1 << 18);
}


int uart_handleMsg(msg_t *msg, int dev)
{
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	uart_t *uart;

	dev -= id_uart1;

	if (dev > 7 || !uartConfig[dev])
		return -EINVAL;

	uart = &uart_common.uarts[uartPos[dev]];

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.io.err = EOK;
			break;

		case mtWrite:
			msg->o.io.err = libtty_write(&uart->tty_common, msg->i.data, msg->i.size, msg->i.io.mode);
			break;

		case mtRead:
			msg->o.io.err = libtty_read(&uart->tty_common, msg->o.data, msg->o.size, msg->i.io.mode);
			break;

		case mtGetAttr:
			if (msg->i.attr.type == atPollStatus)
				msg->o.attr.val = libtty_poll_status(&uart->tty_common);
			else
				msg->o.attr.val = -EINVAL;
			break;

		case mtDevCtl:
			in_data = ioctl_unpack(msg, &request, NULL);
			pid = ioctl_getSenderPid(msg);
			err = libtty_ioctl(&uart->tty_common, pid, request, in_data, &out_data);
			ioctl_setResponse(msg, request, err, out_data);
			break;
	}

	return EOK;
}


#ifdef TARGET_IMXRT1170
static int uart_getIsel(int mux, int *isel, int *val)
{
	/* TODO */
	return -1;
}


static int uart_muxVal(int mux)
{
	/* TODO */
	return 0;
}


#else
static int uart_muxVal(int mux)
{
	switch (mux) {
		case pctl_mux_gpio_b1_12:
		case pctl_mux_gpio_b1_13:
			return 1;

		case pctl_mux_gpio_b0_08:
		case pctl_mux_gpio_b0_09:
			return 3;

		case pctl_mux_gpio_sd_b1_00:
		case pctl_mux_gpio_sd_b1_01:
			return 4;
	}

	return 2;
}


static int uart_getIsel(int mux, int *isel, int *val)
{
	switch (mux) {
		case pctl_mux_gpio_ad_b1_02: *isel = pctl_isel_lpuart2_tx; *val = 1; break;
		case pctl_mux_gpio_sd_b1_11: *isel = pctl_isel_lpuart2_tx; *val = 0; break;
		case pctl_mux_gpio_ad_b1_03: *isel = pctl_isel_lpuart2_rx; *val = 1; break;
		case pctl_mux_gpio_sd_b1_10: *isel = pctl_isel_lpuart2_rx; *val = 0; break;
		case pctl_mux_gpio_emc_13:   *isel = pctl_isel_lpuart3_tx; *val = 1; break;
		case pctl_mux_gpio_ad_b1_06: *isel = pctl_isel_lpuart3_tx; *val = 0; break;
		case pctl_mux_gpio_b0_08:    *isel = pctl_isel_lpuart3_tx; *val = 2; break;
		case pctl_mux_gpio_emc_14:   *isel = pctl_isel_lpuart3_rx; *val = 1; break;
		case pctl_mux_gpio_ad_b1_07: *isel = pctl_isel_lpuart3_rx; *val = 0; break;
		case pctl_mux_gpio_b0_09:    *isel = pctl_isel_lpuart3_rx; *val = 2; break;
		case pctl_mux_gpio_emc_15:   *isel = pctl_isel_lpuart3_cts_b; *val = 0; break;
		case pctl_mux_gpio_ad_b1_04: *isel = pctl_isel_lpuart3_cts_b; *val = 1; break;
		case pctl_mux_gpio_emc_19:   *isel = pctl_isel_lpuart4_tx; *val = 1; break;
		case pctl_mux_gpio_b1_00:    *isel = pctl_isel_lpuart4_tx; *val = 2; break;
		case pctl_mux_gpio_sd_b1_00: *isel = pctl_isel_lpuart4_tx; *val = 0; break;
		case pctl_mux_gpio_emc_20:   *isel = pctl_isel_lpuart4_rx; *val = 1; break;
		case pctl_mux_gpio_b1_01:    *isel = pctl_isel_lpuart4_rx; *val = 2; break;
		case pctl_mux_gpio_sd_b1_01: *isel = pctl_isel_lpuart4_rx; *val = 0; break;
		case pctl_mux_gpio_emc_23:   *isel = pctl_isel_lpuart5_tx; *val = 0; break;
		case pctl_mux_gpio_b1_12:    *isel = pctl_isel_lpuart5_tx; *val = 1; break;
		case pctl_mux_gpio_emc_24:   *isel = pctl_isel_lpuart5_rx; *val = 0; break;
		case pctl_mux_gpio_b1_13:    *isel = pctl_isel_lpuart5_rx; *val = 1; break;
		case pctl_mux_gpio_emc_25:   *isel = pctl_isel_lpuart6_tx; *val = 0; break;
		case pctl_mux_gpio_ad_b0_12: *isel = pctl_isel_lpuart6_tx; *val = 1; break;
		case pctl_mux_gpio_emc_26:   *isel = pctl_isel_lpuart6_rx; *val = 0; break;
		case pctl_mux_gpio_ad_b0_03: *isel = pctl_isel_lpuart6_rx; *val = 1; break;
		case pctl_mux_gpio_emc_31:   *isel = pctl_isel_lpuart7_tx; *val = 1; break;
		case pctl_mux_gpio_sd_b1_08: *isel = pctl_isel_lpuart7_tx; *val = 0; break;
		case pctl_mux_gpio_emc_32:   *isel = pctl_isel_lpuart7_rx; *val = 1; break;
		case pctl_mux_gpio_sd_b1_09: *isel = pctl_isel_lpuart7_rx; *val = 0; break;
		case pctl_mux_gpio_emc_38:   *isel = pctl_isel_lpuart8_tx; *val = 2; break;
		case pctl_mux_gpio_ad_b1_10: *isel = pctl_isel_lpuart8_tx; *val = 1; break;
		case pctl_mux_gpio_sd_b0_04: *isel = pctl_isel_lpuart8_tx; *val = 0; break;
		case pctl_mux_gpio_emc_39:   *isel = pctl_isel_lpuart8_rx; *val = 2; break;
		case pctl_mux_gpio_ad_b1_11: *isel = pctl_isel_lpuart8_rx; *val = 1; break;
		case pctl_mux_gpio_sd_b0_05: *isel = pctl_isel_lpuart8_rx; *val = 0; break;
		default: return -1;
	}

	return 0;
}
#endif


static void uart_initPins(void)
{
	int i, isel, val;
	static const int muxes[] = {
#if UART1
		PIN2MUX(UART1_TX_PIN), PIN2MUX(UART1_RX_PIN),
#endif
#if UART1_HW_FLOWCTRL
		PIN2MUX(UART1_RTS_PIN), PIN2MUX(UART1_CTS_PIN),
#endif
#if UART2
		PIN2MUX(UART2_TX_PIN), PIN2MUX(UART2_RX_PIN),
#endif
#if UART2_HW_FLOWCTRL
		PIN2MUX(UART2_RTS_PIN), PIN2MUX(UART2_CTS_PIN),
#endif
#if UART3
		PIN2MUX(UART3_TX_PIN), PIN2MUX(UART3_RX_PIN),
#endif
#if UART3_HW_FLOWCTRL
		PIN2MUX(UART3_RTS_PIN), PIN2MUX(UART3_CTS_PIN),
#endif
#if UART4
		PIN2MUX(UART4_TX_PIN), PIN2MUX(UART4_RX_PIN),
#endif
#if UART4_HW_FLOWCTRL
		PIN2MUX(UART4_RTS_PIN), PIN2MUX(UART4_CTS_PIN),
#endif
#if UART5
		PIN2MUX(UART5_TX_PIN), PIN2MUX(UART5_RX_PIN),
#endif
#if UART5_HW_FLOWCTRL
		PIN2MUX(UART5_RTS_PIN), PIN2MUX(UART5_CTS_PIN),
#endif
#if UART6
		PIN2MUX(UART6_TX_PIN), PIN2MUX(UART6_RX_PIN),
#endif
#if UART6_HW_FLOWCTRL
		PIN2MUX(UART6_RTS_PIN), PIN2MUX(UART6_CTS_PIN),
#endif
#if UART7
		PIN2MUX(UART7_TX_PIN), PIN2MUX(UART7_RX_PIN),
#endif
#if UART7_HW_FLOWCTRL
		PIN2MUX(UART7_RTS_PIN), PIN2MUX(UART7_CTS_PIN),
#endif
#if UART8
		PIN2MUX(UART8_TX_PIN), PIN2MUX(UART8_RX_PIN),
#endif
#if UART8_HW_FLOWCTRL
		PIN2MUX(UART8_RTS_PIN), PIN2MUX(UART8_CTS_PIN),
#endif
	};

	for (i = 0; i < sizeof(muxes) / sizeof(muxes[0]); ++i) {
		common_setMux(muxes[i], 0, uart_muxVal(muxes[i]));

		if (uart_getIsel(muxes[i], &isel, &val) < 0)
			continue;

		common_setInput(isel, val);
	}
}


int uart_init(void)
{
	int i, dev;
	uint32_t t;
	uart_t *uart;
	libtty_callbacks_t callbacks;
	static const size_t fifoSzLut[] = { 1, 4, 8, 16, 32, 64, 128, 256 };
	static const struct {
		volatile uint32_t *base;
		int dev;
		unsigned irq;
	} info[] = {
		{ UART1_BASE, UART1_CLK, UART1_IRQ },
		{ UART2_BASE, UART2_CLK, UART2_IRQ },
		{ UART3_BASE, UART3_CLK, UART3_IRQ },
		{ UART4_BASE, UART4_CLK, UART4_IRQ },
		{ UART5_BASE, UART5_CLK, UART5_IRQ },
		{ UART6_BASE, UART6_CLK, UART6_IRQ },
		{ UART7_BASE, UART7_CLK, UART7_IRQ },
		{ UART8_BASE, UART8_CLK, UART8_IRQ }
	};

	uart_initPins();

	for (i = 0, dev = 0; dev < sizeof(uartConfig) / sizeof(uartConfig[0]); ++dev) {
		if (!uartConfig[dev])
			continue;

		uart = &uart_common.uarts[i++];
		uart->base = info[dev].base;
		common_setClock(info[dev].dev, clk_state_run);

		if (condCreate(&uart->cond) < 0 || mutexCreate(&uart->lock) < 0)
			return -1;

		callbacks.arg = uart;
		callbacks.set_baudrate = set_baudrate;
		callbacks.set_cflag = set_cflag;
		callbacks.signal_txready = signal_txready;

		if (libtty_init(&uart->tty_common, &callbacks, BUFSIZE) < 0)
			return -1;

		/* Wait for kernel to stop sending data over uart */
		while (*(uart->base + waterr) & 0x700)
			usleep(100);

		/* Disable TX and RX */
		*(uart->base + ctrlr) &= ~((1 << 19) | (1 << 18));

		/* Reset all internal logic and registers, except the Global Register */
		*(uart->base + globalr) |= 1 << 1;
		common_dataBarrier();
		*(uart->base + globalr) &= ~(1 << 1);
		common_dataBarrier();

		/* Disable input trigger */
		*(uart->base + pincfgr) &= ~3;

		/* Set 115200 default baudrate */
		t = *(uart->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0xfff);
		*(uart->base + baudr) = t | calculate_baudrate(B115200);

		/* Set 8 bit and no parity mode */
		*(uart->base + ctrlr) &= ~0x117;

		/* One stop bit */
		*(uart->base + baudr) &= ~(1 << 13);

		*(uart->base + waterr) = 0;

		/* Enable FIFO */
		*(uart->base + fifor) |= (1 << 7) | (1 << 3);
		*(uart->base + fifor) |= 0x3 << 14;

		/* Clear all status flags */
		*(uart->base + statr) |= 0xc01fc000;

		uart->rxFifoSz = fifoSzLut[*(uart->base + fifor) & 0x7];
		uart->txFifoSz = fifoSzLut[(*(uart->base + fifor) >> 4) & 0x7];

		/* Enable receiver interrupt */
		*(uart->base + ctrlr) |= 1 << 21;

		/* Enable TX and RX */
		*(uart->base + ctrlr) |= (1 << 19) | (1 << 18);

		beginthread(uart_intrThread, 2, &uart->stack, sizeof(uart->stack), uart);
		interrupt(info[dev].irq, uart_handleIntr, (void *)uart, uart->cond, NULL);
	}

	return 0;
}
