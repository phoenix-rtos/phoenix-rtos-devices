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


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/ioctl.h>
#include <sys/msg.h>
#include <sys/threads.h>

#include <libtty.h>
#include <libtty-lf-fifo.h>
#include <libklog.h>

#include "gpio.h"
#include "common.h"
#include "uart.h"


#define UART1_POS  0
#define UART2_POS  (UART1_POS + UART1)
#define UART3_POS  (UART2_POS + UART2)
#define UART4_POS  (UART3_POS + UART3)
#define UART5_POS  (UART4_POS + UART4)
#define UART6_POS  (UART5_POS + UART5)
#define UART7_POS  (UART6_POS + UART6)
#define UART8_POS  (UART7_POS + UART7)
#define UART9_POS  (UART8_POS + UART8)
#define UART10_POS (UART9_POS + UART9)
#define UART11_POS (UART10_POS + UART10)
#define UART12_POS (UART11_POS + UART11)

#define UART_CNT (UART1 + UART2 + UART3 + UART4 + UART5 + UART6 + UART7 + UART8 + UART9 + UART10 + UART11 + UART12)


#ifndef UART_RXFIFOSIZE
#define UART_RXFIFOSIZE 128
#endif


typedef struct {
	uint16_t port;
	uint8_t pin;
	uint8_t onTx;
} uart_halfDuplexAction_t;


typedef struct uart_s {
	char stack[1024] __attribute__ ((aligned(8)));

	volatile uint32_t *base;
	uint32_t mode;
	uint16_t dev_no;
	uart_halfDuplexAction_t halfDuplexAction;

	handle_t cond;
	handle_t inth;
	handle_t lock;

	size_t rxFifoSz;
	size_t txFifoSz;

	libtty_common_t tty_common;

	/* fifo between irq and thread */
	lf_fifo_t rxFifoCtx;
	uint8_t rxFifoData[UART_RXFIFOSIZE];

	/* statistics */
	struct {
		/* NOTE: to read statistics use debugger */
		size_t hw_overrunCntr;
		size_t sw_overrunCntr;
	} stat;
} uart_t;


struct {
	uart_t uarts[UART_CNT];
} uart_common;


static const struct {
	volatile uint32_t *base;
	int clk;
	unsigned irq;
	int active;
	int pos;
	uart_halfDuplexAction_t halfDuplexAction;
} uart_preConfig[] = {
	{
		UART1_BASE,
		UART1_CLK,
		UART1_IRQ,
		UART1,
		UART1_POS,
		{ UART1_HALF_DUPLEX_GPIO },
	},
	{
		UART2_BASE,
		UART2_CLK,
		UART2_IRQ,
		UART2,
		UART2_POS,
		{ UART2_HALF_DUPLEX_GPIO },
	},
	{
		UART3_BASE,
		UART3_CLK,
		UART3_IRQ,
		UART3,
		UART3_POS,
		{ UART3_HALF_DUPLEX_GPIO },
	},
	{
		UART4_BASE,
		UART4_CLK,
		UART4_IRQ,
		UART4,
		UART4_POS,
		{ UART4_HALF_DUPLEX_GPIO },
	},
	{
		UART5_BASE,
		UART5_CLK,
		UART5_IRQ,
		UART5,
		UART5_POS,
		{ UART5_HALF_DUPLEX_GPIO },
	},
	{
		UART6_BASE,
		UART6_CLK,
		UART6_IRQ,
		UART6,
		UART6_POS,
		{ UART6_HALF_DUPLEX_GPIO },
	},
	{
		UART7_BASE,
		UART7_CLK,
		UART7_IRQ,
		UART7,
		UART7_POS,
		{ UART7_HALF_DUPLEX_GPIO },
	},
	{
		UART8_BASE,
		UART8_CLK,
		UART8_IRQ,
		UART8,
		UART8_POS,
		{ UART8_HALF_DUPLEX_GPIO },
	},
#ifdef __CPU_IMXRT117X
	{
		UART9_BASE,
		UART9_CLK,
		UART9_IRQ,
		UART9,
		UART9_POS,
		{ UART9_HALF_DUPLEX_GPIO },
	},
	{
		UART10_BASE,
		UART10_CLK,
		UART10_IRQ,
		UART10,
		UART10_POS,
		{ UART10_HALF_DUPLEX_GPIO },
	},
	{
		UART11_BASE,
		UART11_CLK,
		UART11_IRQ,
		UART11,
		UART11_POS,
		{ UART11_HALF_DUPLEX_GPIO },
	},
	{
		UART12_BASE,
		UART12_CLK,
		UART12_IRQ,
		UART12,
		UART12_POS,
		{ UART12_HALF_DUPLEX_GPIO },
	},
#endif
};


enum { veridr = 0, paramr, globalr, pincfgr, baudr, statr, ctrlr, datar, matchr, modirr, fifor, waterr };


static inline int uart_getRXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 24) & 0xff;
}


static inline int uart_getTXcount(uart_t *uart)
{
	return (*(uart->base + waterr) >> 8) & 0xff;
}


static int uart_performHalfDuplexAction(const uart_halfDuplexAction_t *action, int tx)
{
	uint32_t val = ((tx != 0) ? 0 : 1) ^ ((action->onTx != 0) ? 1 : 0);
	return gpio_setPort(action->port, 1 << action->pin, val << action->pin);
}


static int uart_handleIntr(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint32_t status = *(uart->base + statr);

	/* Disable interrupts, enabled in uart_intrThread */
	*(uart->base + ctrlr) &= ~((1 << 27) | (1 << 26) | (1 << 25) | (1 << 23) | (1 << 22) | (1 << 21));

	/* check for RX overrun */
	if ((status & (1uL << 19u)) != 0u) {
		/* invalidate fifo */
		*(uart->base + fifor) |= 1uL << 14u;
		uart->stat.hw_overrunCntr++;
	}

	/* Process received data */
	while (uart_getRXcount(uart) != 0) {
		uint8_t data = *(uart->base + datar);
		if (lf_fifo_push(&uart->rxFifoCtx, data) == 0) {
			uart->stat.sw_overrunCntr++;
		}
	}

	/* Clear errors: parity, framing, noise, overrun and idle flag */
	*(uart->base + statr) = (status & (0x1fuL << 16));

	return 1;
}


static void uart_intrThread(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	uint8_t mask;
	uint8_t c;
	int rxActive = 1;

	for (;;) {
		/* wait for character or transmit data */
		mutexLock(uart->lock);
		while (lf_fifo_empty(&uart->rxFifoCtx) != 0) { /* nothing to RX */
			uint32_t intrs = (1 << 27) | (1 << 26) | (1 << 25) | (1 << 21);
			if (libtty_txready(&uart->tty_common)) {          /* something to TX */
				if (uart_getTXcount(uart) < uart->txFifoSz) { /* TX ready */
					break;
				}
				else {
					intrs |= 1 << 23;
				}
			}

			if ((uart->halfDuplexAction.port != 0) && (rxActive == 0)) {
				uint32_t status = *(uart->base + statr);
				if ((status & (1 << 22)) != 0u) {
					/* Transmission finished, activate RX */
					uart_performHalfDuplexAction(&uart->halfDuplexAction, 0);
					*(uart->base + ctrlr) |= (1 << 18);
					rxActive = 1;
				}
				else {
					/* Transmission in progress, activate interrupt */
					intrs |= (1 << 22);
				}
			}

			*(uart->base + ctrlr) |= intrs;

			condWait(uart->cond, uart->lock, 0);
		}

		if ((uart->tty_common.term.c_cflag & CSIZE) == CS7) {
			mask = 0x7f;
		}
		else {
			mask = 0xff;
		}

		mutexUnlock(uart->lock);

		/* RX */
		while (lf_fifo_pop(&uart->rxFifoCtx, &c) != 0) {
			libtty_putchar(&uart->tty_common, c & mask, NULL);
		}

		/* TX */
		if (libtty_txready(&uart->tty_common) && uart_getTXcount(uart) < uart->txFifoSz) {
			if ((uart->halfDuplexAction.port != 0) && (rxActive != 0)) {
				*(uart->base + ctrlr) &= ~(1 << 18);
				uart_performHalfDuplexAction(&uart->halfDuplexAction, 1);
				rxActive = 0;
			}

			do {
				*(uart->base + datar) = libtty_getchar(&uart->tty_common, NULL);
			} while (libtty_txready(&uart->tty_common) && uart_getTXcount(uart) < uart->txFifoSz);
		}
	}
}


static void signal_txready(void *_uart)
{
	uart_t *uartptr = (uart_t *)_uart;
	condSignal(uartptr->cond);
}


static void set_cflag(void *_uart, tcflag_t *cflag)
{
	uart_t *uartptr = (uart_t *)_uart;
	uint32_t t;

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	/* CSIZE only CS7 and CS8 (default) is supported */
	if ((*cflag & CSIZE) != CS7) { /* CS8 */
		*cflag &= ~CSIZE;
		*cflag |= CS8;
	}

	/* If parity bit is enabled data character length must be incremented */
	t = *(uartptr->base + ctrlr) & ~(1 << 4 | 1 << 11);
	if ((*cflag & CSIZE) == CS7) {
		if (!(*cflag & PARENB)) {
			t |= 1 << 11;
		}
	}
	else if (*cflag & PARENB) {
		t |= 1 << 4;
	}
	*(uartptr->base + ctrlr) = t;

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


static uint32_t calculate_baudrate(uint32_t baud)
{
	uint32_t osr, sbr, bestSbr = 0, bestOsr = 0, bestDiff, t, tDiff;

	if (baud == 0)
		return 0;

	bestDiff = baud;

	for (osr = 4; osr <= 32; ++osr) {
		/* find sbr value in range between 1 and 8191 */
		sbr = (UART_CLK / (baud * osr)) & 0x1fff;
		sbr = (sbr == 0) ? 1 : sbr;

		/* baud rate difference based on temporary osr and sbr */
		tDiff = UART_CLK / (osr * sbr) - baud;
		t = UART_CLK / (osr * (sbr + 1));

		/* select best values between sbr and sbr+1 */
		if (tDiff > baud - t) {
			tDiff = baud - t;
			sbr += (sbr < 0x1fff);
		}

		if (tDiff <= bestDiff) {
			bestDiff = tDiff;
			bestOsr = osr - 1;
			bestSbr = sbr;
		}
	}

	return (bestOsr << 24) | ((bestOsr <= 6) << 17) | (bestSbr & 0x1fff);
}


static void set_baudrate(void *_uart, speed_t baud)
{
	uint32_t reg = 0, t;
	int b;
	uart_t *uartptr = (uart_t *)_uart;

	if ((b = libtty_baudrate_to_int(baud)) > 0)
		reg = calculate_baudrate((uint32_t)b);

	/* disable TX and RX */
	*(uartptr->base + ctrlr) &= ~((1 << 19) | (1 << 18));

	t = *(uartptr->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0x1fff);
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

	if ((dev < 0) || (dev >= (sizeof(uart_preConfig) / sizeof(uart_preConfig[0]))) || (uart_preConfig[dev].active == 0))
		return -EINVAL;

	uart = &uart_common.uarts[uart_preConfig[dev].pos];

	switch (msg->type) {
		case mtWrite:
			msg->o.err = libtty_write(&uart->tty_common, msg->i.data, msg->i.size, msg->i.io.mode);
			break;

		case mtRead:
			msg->o.err = libtty_read(&uart->tty_common, msg->o.data, msg->o.size, msg->i.io.mode);
			break;

		case mtGetAttr:
			if (msg->i.attr.type != atPollStatus) {
				msg->o.err = -ENOSYS;
				break;
			}
			msg->o.attr.val = libtty_poll_status(&uart->tty_common);
			msg->o.err = EOK;
			break;

		case mtDevCtl:
			in_data = ioctl_unpack(msg, &request, NULL);
			pid = ioctl_getSenderPid(msg);

			if (request == KIOEN) {
#if !ISEMPTY(UART_CONSOLE_USER)
				if (dev == UART_CONSOLE_USER - 1) {
					libklog_enable((int)(intptr_t)in_data);
					err = EOK;
				}
				else {
					err = -EINVAL;
				}
#else
				err = -EINVAL;
#endif
			}
			else {
				err = libtty_ioctl(&uart->tty_common, pid, request, in_data, &out_data);
			}

			ioctl_setResponse(msg, request, err, out_data);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}

	return EOK;
}

#ifdef __CPU_IMXRT117X

static int uart_muxVal(int uart, int mux)
{
	if (mux == pctl_mux_gpio_ad_02)
		return (uart == 6) ? 1 : 6;

	if (mux == pctl_mux_gpio_ad_03)
		return (uart == 6) ? 1 : 6;

	if (mux == pctl_mux_gpio_disp_b2_08)
		return (uart == 0) ? 9 : 2;

	if (mux == pctl_mux_gpio_disp_b2_09)
		return (uart == 0) ? 9 : 2;

	switch (mux) {
		case pctl_mux_gpio_ad_24:
		case pctl_mux_gpio_ad_25:
		case pctl_mux_gpio_ad_26:
		case pctl_mux_gpio_ad_27:
		case pctl_mux_gpio_lpsr_08:
		case pctl_mux_gpio_lpsr_09:
			return 0;

		case pctl_mux_gpio_ad_04:
		case pctl_mux_gpio_ad_05:
		case pctl_mux_gpio_ad_15:
		case pctl_mux_gpio_ad_16:
		case pctl_mux_gpio_ad_28:
		case pctl_mux_gpio_ad_29:
			return 1;

		case pctl_mux_gpio_disp_b1_04:
		case pctl_mux_gpio_disp_b1_05:
		case pctl_mux_gpio_disp_b1_06:
		case pctl_mux_gpio_disp_b1_07:
		case pctl_mux_gpio_disp_b2_06:
		case pctl_mux_gpio_disp_b2_07:
		case pctl_mux_gpio_disp_b2_10:
		case pctl_mux_gpio_disp_b2_11:
			return 2;

		case pctl_mux_gpio_disp_b2_12:
		case pctl_mux_gpio_disp_b2_13:
		case pctl_mux_gpio_sd_b2_00:
		case pctl_mux_gpio_sd_b2_01:
		case pctl_mux_gpio_sd_b2_02:
		case pctl_mux_gpio_sd_b2_03:
		case pctl_mux_gpio_sd_b2_07:
		case pctl_mux_gpio_sd_b2_08:
		case pctl_mux_gpio_sd_b2_09:
		case pctl_mux_gpio_sd_b2_10:
		case pctl_mux_gpio_emc_b1_40:
		case pctl_mux_gpio_emc_b1_41:
		case pctl_mux_gpio_emc_b2_00:
		case pctl_mux_gpio_emc_b2_01:
		case pctl_mux_gpio_lpsr_06:
		case pctl_mux_gpio_lpsr_07:
			return 3;

		case pctl_mux_gpio_ad_30:
		case pctl_mux_gpio_ad_31:
			return 4;

		case pctl_mux_gpio_ad_00:
		case pctl_mux_gpio_ad_01:
		case pctl_mux_gpio_lpsr_00:
		case pctl_mux_gpio_lpsr_01:
		case pctl_mux_gpio_lpsr_04:
		case pctl_mux_gpio_lpsr_05:
			return 6;

		case pctl_mux_gpio_ad_32:
		case pctl_mux_gpio_ad_33:
		case pctl_mux_gpio_ad_34:
		case pctl_mux_gpio_ad_35:
		case pctl_mux_gpio_lpsr_10:
		case pctl_mux_gpio_lpsr_11:
			return 8;

		case pctl_mux_gpio_disp_b1_02:
		case pctl_mux_gpio_disp_b1_03:
			return 9;
	}

	return 2;
}


static int uart_getIsel(int uart, int mux, int *isel, int *val)
{
	switch (mux) {
		case pctl_mux_gpio_ad_24:      *isel = pctl_isel_lpuart1_txd; *val = 0; break;
		case pctl_mux_gpio_disp_b1_02: *isel = pctl_isel_lpuart1_txd; *val = 1; break;
		case pctl_mux_gpio_ad_25:      *isel = pctl_isel_lpuart1_rxd; *val = 0; break;
		case pctl_mux_gpio_disp_b1_03: *isel = pctl_isel_lpuart1_rxd; *val = 1; break;
		case pctl_mux_gpio_disp_b2_06: *isel = pctl_isel_lpuart7_txd; *val = 1; break;
		case pctl_mux_gpio_ad_00:      *isel = pctl_isel_lpuart7_txd; *val = 0; break;
		case pctl_mux_gpio_disp_b2_07: *isel = pctl_isel_lpuart7_rxd; *val = 1; break;
		case pctl_mux_gpio_ad_01:      *isel = pctl_isel_lpuart7_rxd; *val = 0; break;
		case pctl_mux_gpio_ad_02:      *isel = pctl_isel_lpuart8_txd; *val = 0; break;
		case pctl_mux_gpio_ad_03:      *isel = pctl_isel_lpuart8_rxd; *val = 0; break;
		case pctl_mux_gpio_lpsr_08:    *isel = pctl_isel_lpuart11_txd; *val = 1; break;
		case pctl_mux_gpio_lpsr_04:    *isel = pctl_isel_lpuart11_txd; *val = 0; break;
		case pctl_mux_gpio_lpsr_09:    *isel = pctl_isel_lpuart11_rxd; *val = 1; break;
		case pctl_mux_gpio_lpsr_05:    *isel = pctl_isel_lpuart11_rxd; *val = 0; break;
		case pctl_mux_gpio_lpsr_06:    *isel = pctl_isel_lpuart12_txd; *val = 1; break;
		case pctl_mux_gpio_lpsr_00:    *isel = pctl_isel_lpuart12_txd; *val = 0; break;
		case pctl_mux_gpio_lpsr_10:    *isel = pctl_isel_lpuart12_txd; *val = 2; break;
		case pctl_mux_gpio_lpsr_07:    *isel = pctl_isel_lpuart12_rxd; *val = 1; break;
		case pctl_mux_gpio_lpsr_01:    *isel = pctl_isel_lpuart12_rxd; *val = 0; break;
		case pctl_mux_gpio_lpsr_11:    *isel = pctl_isel_lpuart12_rxd; *val = 2; break;
		case pctl_mux_gpio_disp_b2_09:
			if (uart == 0) {
				*isel = pctl_isel_lpuart1_rxd;
				*val = 2;
			}
			else {
				*isel = pctl_isel_lpuart8_rxd;
				*val = 1;
			}
			break;
		case pctl_mux_gpio_disp_b2_08:
			if (uart == 0) {
				*isel = pctl_isel_lpuart1_txd;
				*val = 2;
			}
			else {
				*isel = pctl_isel_lpuart8_txd;
				*val = 1;
			}
			break;
		default: return -1;
	}

	return 0;
}


static void uart_initPins(void)
{
	int i, j, isel, val;
	static const struct {
		int uart;
		int muxes[2];
		int pads[2];
	} info[] = {
	/* clang-format off */
#if UART1
		{ 0,
		{ PIN2MUX(UART1_TX_PIN), PIN2MUX(UART1_RX_PIN) },
		{ PIN2PAD(UART1_TX_PIN), PIN2PAD(UART1_RX_PIN) } },
#if UART1_HW_FLOWCTRL
		{ 0,
		{ PIN2MUX(UART1_RTS_PIN), PIN2MUX(UART1_CTS_PIN) },
		{ PIN2PAD(UART1_RTS_PIN), PIN2PAD(UART1_CTS_PIN) } },
#endif
#endif

#if UART2
		{ 1,
		{ PIN2MUX(UART2_TX_PIN), PIN2MUX(UART2_RX_PIN) },
		{ PIN2PAD(UART2_TX_PIN), PIN2PAD(UART2_RX_PIN) } },
#if UART2_HW_FLOWCTRL
		{ 1,
		{ PIN2MUX(UART2_RTS_PIN), PIN2MUX(UART2_CTS_PIN) },
		{ PIN2PAD(UART2_RTS_PIN), PIN2PAD(UART2_CTS_PIN) } },
#endif
#endif

#if UART3
		{ 2,
		{ PIN2MUX(UART3_TX_PIN), PIN2MUX(UART3_RX_PIN) },
		{ PIN2PAD(UART3_TX_PIN), PIN2PAD(UART3_RX_PIN) } },
#if UART3_HW_FLOWCTRL
		{ 2,
		{ PIN2MUX(UART3_RTS_PIN), PIN2MUX(UART3_CTS_PIN) },
		{ PIN2PAD(UART3_RTS_PIN), PIN2PAD(UART3_CTS_PIN) } },
#endif
#endif

#if UART4
		{ 3,
		{ PIN2MUX(UART4_TX_PIN), PIN2MUX(UART4_RX_PIN) },
		{ PIN2PAD(UART4_TX_PIN), PIN2PAD(UART4_RX_PIN) } },
#if UART4_HW_FLOWCTRL
		{ 3,
		{ PIN2MUX(UART4_RTS_PIN), PIN2MUX(UART4_CTS_PIN) },
		{ PIN2PAD(UART4_RTS_PIN), PIN2PAD(UART4_CTS_PIN) } },
#endif
#endif

#if UART5
		{ 4,
		{ PIN2MUX(UART5_TX_PIN), PIN2MUX(UART5_RX_PIN) },
		{ PIN2PAD(UART5_TX_PIN), PIN2PAD(UART5_RX_PIN) } },
#if UART5_HW_FLOWCTRL
		{ 4,
		{ PIN2MUX(UART5_RTS_PIN), PIN2MUX(UART5_CTS_PIN) },
		{ PIN2PAD(UART5_RTS_PIN), PIN2PAD(UART5_CTS_PIN) } },
#endif
#endif

#if UART6
		{ 5,
		{ PIN2MUX(UART6_TX_PIN), PIN2MUX(UART6_RX_PIN) },
		{ PIN2PAD(UART6_TX_PIN), PIN2PAD(UART6_RX_PIN) } },
#if UART6_HW_FLOWCTRL
		{ 5,
		{ PIN2MUX(UART6_RTS_PIN), PIN2MUX(UART6_CTS_PIN) },
		{ PIN2PAD(UART6_RTS_PIN), PIN2PAD(UART6_CTS_PIN) } },
#endif
#endif

#if UART7
		{ 6,
		{ PIN2MUX(UART7_TX_PIN), PIN2MUX(UART7_RX_PIN) },
		{ PIN2PAD(UART7_TX_PIN), PIN2PAD(UART7_RX_PIN) } },
#if UART7_HW_FLOWCTRL
		{ 6,
		{ PIN2MUX(UART7_RTS_PIN), PIN2MUX(UART7_CTS_PIN) },
		{ PIN2PAD(UART7_RTS_PIN), PIN2PAD(UART7_CTS_PIN) } },
#endif
#endif

#if UART8
		{ 7,
		{ PIN2MUX(UART8_TX_PIN), PIN2MUX(UART8_RX_PIN) },
		{ PIN2PAD(UART8_TX_PIN), PIN2PAD(UART8_RX_PIN) } },
#if UART8_HW_FLOWCTRL
		{ 7,
		{ PIN2MUX(UART8_RTS_PIN), PIN2MUX(UART8_CTS_PIN) },
		{ PIN2PAD(UART8_RTS_PIN), PIN2PAD(UART8_CTS_PIN) } },
#endif
#endif

#if UART9
		{ 8,
		{ PIN2MUX(UART9_TX_PIN), PIN2MUX(UART9_RX_PIN) },
		{ PIN2PAD(UART9_TX_PIN), PIN2PAD(UART9_RX_PIN) } },
#if UART9_HW_FLOWCTRL
		{ 8,
		{ PIN2MUX(UART9_RTS_PIN), PIN2MUX(UART9_CTS_PIN) },
		{ PIN2PAD(UART9_RTS_PIN), PIN2PAD(UART9_CTS_PIN) } },
#endif
#endif

#if UART10
		{ 9,
		{ PIN2MUX(UART10_TX_PIN), PIN2MUX(UART10_RX_PIN) },
		{ PIN2PAD(UART10_TX_PIN), PIN2PAD(UART10_RX_PIN) } },
#if UART10_HW_FLOWCTRL
		{ 9,
		{ PIN2MUX(UART10_RTS_PIN), PIN2MUX(UART10_CTS_PIN) },
		{ PIN2PAD(UART10_RTS_PIN), PIN2PAD(UART10_CTS_PIN) } },
#endif
#endif

#if UART11
		{ 10,
		{ PIN2MUX(UART11_TX_PIN), PIN2MUX(UART11_RX_PIN) },
		{ PIN2PAD(UART11_TX_PIN), PIN2PAD(UART11_RX_PIN) } },
#if UART11_HW_FLOWCTRL
		{ 10,
		{ PIN2MUX(UART11_RTS_PIN), PIN2MUX(UART11_CTS_PIN) },
		{ PIN2PAD(UART11_RTS_PIN), PIN2PAD(UART11_CTS_PIN) } },
#endif
#endif

#if UART12
		{ 11,
		{ PIN2MUX(UART12_TX_PIN), PIN2MUX(UART12_RX_PIN) },
		{ PIN2PAD(UART12_TX_PIN), PIN2PAD(UART12_RX_PIN) } },
#if UART12_HW_FLOWCTRL
		{ 11,
		{ PIN2MUX(UART12_RTS_PIN), PIN2MUX(UART12_CTS_PIN) },
		{ PIN2PAD(UART12_RTS_PIN), PIN2PAD(UART12_CTS_PIN) } },
#endif
#endif
	};
	/* clang-format on */

	for (i = 0; i < sizeof(info) / sizeof(info[0]); ++i) {
		for (j = 0; j < sizeof(info[0].muxes) / sizeof(info[0].muxes[0]); ++j) {
			common_setMux(info[i].muxes[j], 0, uart_muxVal(info[i].uart, info[i].muxes[j]));

			if (uart_getIsel(info[i].uart, info[i].muxes[j], &isel, &val) >= 0)
				common_setInput(isel, val);
		}

		common_setPad(info[i].pads[0], 0, 0, 0, 0, 0, 0, 0, 0);
		common_setPad(info[i].pads[1], 0, 1, 1, 1, 0, 0, 0, 0);
	}
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

static void uart_initPins(void)
{
	int i, isel, val;
	static const int muxes[] = {
	/* clang-format off */
#if UART1
		PIN2MUX(UART1_TX_PIN), PIN2MUX(UART1_RX_PIN),
#if UART1_HW_FLOWCTRL
		PIN2MUX(UART1_RTS_PIN), PIN2MUX(UART1_CTS_PIN),
#endif
#endif
#if UART2
		PIN2MUX(UART2_TX_PIN), PIN2MUX(UART2_RX_PIN),
#if UART2_HW_FLOWCTRL
		PIN2MUX(UART2_RTS_PIN), PIN2MUX(UART2_CTS_PIN),
#endif
#endif
#if UART3
		PIN2MUX(UART3_TX_PIN), PIN2MUX(UART3_RX_PIN),
#if UART3_HW_FLOWCTRL
		PIN2MUX(UART3_RTS_PIN), PIN2MUX(UART3_CTS_PIN),
#endif
#endif
#if UART4
		PIN2MUX(UART4_TX_PIN), PIN2MUX(UART4_RX_PIN),
#if UART4_HW_FLOWCTRL
		PIN2MUX(UART4_RTS_PIN), PIN2MUX(UART4_CTS_PIN),
#endif
#endif
#if UART5
		PIN2MUX(UART5_TX_PIN), PIN2MUX(UART5_RX_PIN),
#if UART5_HW_FLOWCTRL
		PIN2MUX(UART5_RTS_PIN), PIN2MUX(UART5_CTS_PIN),
#endif
#endif
#if UART6
		PIN2MUX(UART6_TX_PIN), PIN2MUX(UART6_RX_PIN),
#if UART6_HW_FLOWCTRL
		PIN2MUX(UART6_RTS_PIN), PIN2MUX(UART6_CTS_PIN),
#endif
#endif
#if UART7
		PIN2MUX(UART7_TX_PIN), PIN2MUX(UART7_RX_PIN),
#if UART7_HW_FLOWCTRL
		PIN2MUX(UART7_RTS_PIN), PIN2MUX(UART7_CTS_PIN),
#endif
#endif
#if UART8
		PIN2MUX(UART8_TX_PIN), PIN2MUX(UART8_RX_PIN),
#if UART8_HW_FLOWCTRL
		PIN2MUX(UART8_RTS_PIN), PIN2MUX(UART8_CTS_PIN),
#endif
#endif
	};
	/* clang-format on */

	for (i = 0; i < sizeof(muxes) / sizeof(muxes[0]); ++i) {
		common_setMux(muxes[i], 0, uart_muxVal(muxes[i]));

		if (uart_getIsel(muxes[i], &isel, &val) < 0)
			continue;

		common_setInput(isel, val);
	}
}
#endif


void uart_klogCblk(const char *data, size_t size)
{
#if !ISEMPTY(UART_CONSOLE_USER)
	libtty_write(&uart_common.uarts[uart_preConfig[UART_CONSOLE_USER - 1].pos].tty_common, data, size, 0);
#endif
}


int uart_init(void)
{
	int i, dev;
	uint32_t t;
	uart_t *uart;
	libtty_callbacks_t callbacks;
	static const size_t fifoSzLut[] = { 1, 4, 8, 16, 32, 64, 128, 256 };

	memset(&uart_common, 0, sizeof(uart_common));

	uart_initPins();

	const uint32_t default_baud[] = { UART_BAUDRATES };
	const uint32_t tty_bufsz[] = { UART_BUFSIZES };

	for (i = 0, dev = 0; dev < (sizeof(uart_preConfig) / sizeof(uart_preConfig[0])); ++dev) {
		if (uart_preConfig[dev].active == 0) {
			continue;
		}

		uart = &uart_common.uarts[i++];
		uart->base = uart_preConfig[dev].base;
		uart->dev_no = dev;
		uart->halfDuplexAction = uart_preConfig[dev].halfDuplexAction;

#ifdef __CPU_IMXRT117X
		common_setClock(uart_preConfig[dev].clk, -1, -1, -1, -1, 1);
#else
		common_setClock(uart_preConfig[dev].clk, clk_state_run);
#endif
		if (condCreate(&uart->cond) < 0 || mutexCreate(&uart->lock) < 0)
			return -1;

		callbacks.arg = uart;
		callbacks.set_baudrate = set_baudrate;
		callbacks.set_cflag = set_cflag;
		callbacks.signal_txready = signal_txready;

		if (libtty_init(&uart->tty_common, &callbacks, tty_bufsz[dev], libtty_int_to_baudrate(default_baud[dev])) < 0) {
			return -1;
		}

		lf_fifo_init(&uart->rxFifoCtx, uart->rxFifoData, sizeof(uart->rxFifoData));

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
		t = *(uart->base + baudr) & ~((0x1f << 24) | (1 << 17) | 0x1fff);
		*(uart->base + baudr) = t | calculate_baudrate(default_baud[dev]);

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

		/* Enable overrun, noise, framing error and receiver interrupts */
		*(uart->base + ctrlr) |= (1 << 27) | (1 << 26) | (1 << 25) | (1 << 21);

		if (uart->halfDuplexAction.port != 0) {
			const uart_halfDuplexAction_t *action = &uart->halfDuplexAction;
			if (gpio_setDir(action->port, 1 << action->pin, 1 << action->pin) < 0) {
				return -1;
			}

			uart_performHalfDuplexAction(action, 0);
		}

		/* Enable TX and RX */
		*(uart->base + ctrlr) |= (1 << 19) | (1 << 18);

		beginthread(uart_intrThread, IMXRT_MULTI_PRIO, &uart->stack, sizeof(uart->stack), uart);
		interrupt(uart_preConfig[dev].irq, uart_handleIntr, (void *)uart, uart->cond, NULL);
	}

	return 0;
}
