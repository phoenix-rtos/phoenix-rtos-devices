/*
 * Phoenix-RTOS
 *
 * Quad-SPI Controller driver
 *
 * Copyright 2021, 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "qspi.h"

#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/interrupt.h>
#include <sys/minmax.h>

#include <board_config.h>
#include <phoenix/arch/zynq7000.h>


/* clang-format off */
enum { cr = 0, sr, ier, idr, imr, er, dr, txd00, rxd, sicr, txth, rxth, gpio,
	   lpbk = 0xe, txd01 = 0x20, txd10, txd11,
	   lqspi_cr = 0x28, lqspi_sr, modid = 0x3f };
/* clang-format on */


struct {
	volatile uint32_t *base;
	handle_t cond;
	handle_t inth;
	handle_t irqLock;
} qspi_common;


static inline void qspi_dataMemoryBarrier(void)
{
	/* clang-format off */
	__asm__ volatile ("dmb");
	/* clang-format on */
}


static int qspi_irqHandler(unsigned int n, void *arg)
{
	*(qspi_common.base + idr) |= (1 << 2);

	return 1;
}


static int qspi_rxFifoEmpty(void)
{
	/* Update of RX not empty bit is delayed, thus it should be read twice. */
	/* https://support.xilinx.com/s/article/47575?language=en_US */
	(void)*(qspi_common.base + sr);

	return ((*(qspi_common.base + sr) & (1 << 4)) == 0) ? 1 : 0;
}


/* TODO: add timeout exit condition to be compliant with MISRA */
void qspi_stop(void)
{
	/* Clean up RX Fifo */
	while (qspi_rxFifoEmpty() == 0) {
		*(qspi_common.base + rxd);
	}

	*(qspi_common.base + cr) |= (1 << 10);
	qspi_dataMemoryBarrier();
	*(qspi_common.base + er) = 0;
	qspi_dataMemoryBarrier();
}


void qspi_start(void)
{
	*(qspi_common.base + rxth) = 0x1;
	*(qspi_common.base + cr) &= ~(1 << 10);
	qspi_dataMemoryBarrier();

	*(qspi_common.base + er) = 0x1;
	qspi_dataMemoryBarrier();
}


static unsigned int qspi_rxData(uint8_t *rxBuff, size_t size)
{
	const uint32_t data = *(qspi_common.base + rxd);

	if (rxBuff != NULL) {
		switch (size) {
			case 0:
				break;

			case 1:
				rxBuff[0] = data >> 24;
				break;

			case 2:
				rxBuff[0] = (data >> 16) & 0xff;
				rxBuff[1] = data >> 24;
				break;

			case 3:
				rxBuff[0] = (data >> 8) & 0xff;
				rxBuff[1] = (data >> 16) & 0xff;
				rxBuff[2] = data >> 24;
				break;

			case 4:
			default:
				rxBuff[0] = data & 0xff;
				rxBuff[1] = (data >> 8) & 0xff;
				rxBuff[2] = (data >> 16) & 0xff;
				rxBuff[3] = data >> 24;
				break;
		}
	}

	return min(size, 4);
}


static unsigned int qspi_txData(const uint8_t *txBuff, size_t size)
{
	const uint8_t dummy[sizeof(uint32_t)] = { 0 };
	const uint8_t *buff = (txBuff == NULL) ? dummy : txBuff;

	switch (size) {
		case 0:
			return 0;

		case 1:
			*(qspi_common.base + txd01) = buff[0];
			return 1;

		case 2:
			*(qspi_common.base + txd10) = buff[0] | (buff[1] << 8);
			return 2;

		case 3:
			*(qspi_common.base + txd11) = buff[0] | (buff[1] << 8) | (buff[2] << 16);
			return 3;

		case 4:
		default:
			*(qspi_common.base + txd00) = buff[0] | (buff[1] << 8) | (buff[2] << 16) | (buff[3] << 24);
			return 4;
	}
}


static int qspi_txFifoFull(void)
{
	return ((*(qspi_common.base + sr) & (1 << 3)) == 0) ? 0 : 1;
}


static int qspi_txFifoEmpty(void)
{
	return ((*(qspi_common.base + sr) & (1 << 2)) == 0) ? 0 : 1;
}


ssize_t qspi_transfer(const uint8_t *txBuff, uint8_t *rxBuff, size_t size, time_t timeout)
{
	int err;
	size_t tempSz, txSz = size, rxSz = 0;
	time_t now, end;

	/* At the start of each iteration FIFOs are empty.
	   Controller only transfers as much data as inserted onto TxFIFO,
	   and each transmission is started manually. When no data is on TxFIFO SCLK is stopped.
	   Thus, there's no potential of potential data loss. */
	mutexLock(qspi_common.irqLock);
	while (txSz != 0) {
		/* Incomplete word can only be written onto an empty TxFIFO. */
		if (txSz < sizeof(uint32_t)) {
			tempSz = qspi_txData(txBuff, txSz);

			txSz -= tempSz;
			rxSz += tempSz;
		}
		else {
			/* Transmit data */
			while ((txSz >= sizeof(uint32_t)) && (qspi_txFifoFull() == 0)) {
				tempSz = qspi_txData(txBuff, txSz);

				txSz -= tempSz;
				rxSz += tempSz;

				if (txBuff != NULL) {
					txBuff += tempSz;
				}
			}
		}

		/* TX FIFO not full enable irq */
		*(qspi_common.base + ier) |= (1 << 2);
		/* Start data transmission */
		*(qspi_common.base + cr) |= (1 << 16);

		(void)gettime(&now, NULL);
		end = now + (timeout * 1000);

		err = 0;
		/* Wait until TX Fifo is empty */
		while ((now < end) && (qspi_txFifoEmpty() == 0)) {
			err = condWait(qspi_common.cond, qspi_common.irqLock, end - now);
			(void)gettime(&now, NULL);
		}

		/* In case of timeout check for the last time if TX Fifo is empty. */
		/* This check is done to prevent the possibly of starvation. */
		if ((err < 0) && (qspi_txFifoEmpty() == 0)) {
			mutexUnlock(qspi_common.irqLock);
			return err;
		}

		/* Receive data */
		while (rxSz != 0) {
			if (qspi_rxFifoEmpty() == 1) {
				/* Invalid state. */
				mutexUnlock(qspi_common.irqLock);
				return -EIO;
			}

			tempSz = qspi_rxData(rxBuff, rxSz);
			rxSz -= tempSz;

			if (rxBuff != NULL) {
				rxBuff += tempSz;
			}
		}
	}

	mutexUnlock(qspi_common.irqLock);
	return size;
}


/* Linear mode allows only for reading data.
 * 03h command is recommended, otherwise first word = 0 (internal bug) :
 * https://support.xilinx.com/s/article/60803?language=en_US
 */
#if 0
static int qspi_linearMode(void)
{
	/* Disable QSPI */
	*(qspi_common.base + er) &= ~0x1;
	qspi_dataMemoryBarrier();

	/* Disable IRQs */
	*(qspi_common.base + idr) = 0x7d;

	/* Disable linear mode */
	*(qspi_common.base + lqspi_cr) = 0;

	*(qspi_common.base + cr) = (1 << 14);
	*(qspi_common.base + cr) = (1 << 10);

	*(qspi_common.base + cr) &= ~(0x7 << 3);
	*(qspi_common.base + cr) |= (0x3 << 3);

	*(qspi_common.base + cr) |= 0x1;
	*(qspi_common.base + cr) |= (1 << 31);
	*(qspi_common.base + cr) &= ~(1 << 26);

	*(qspi_common.base + cr) |= (0x3 << 6);
	*(qspi_common.base + cr) &= ~(0x3 << 1);
	*(qspi_common.base + cr) |= (0x1 << 19);

	*(qspi_common.base + lqspi_cr) =  0x80000003;

	*(qspi_common.base + er) = 0x1;
	qspi_dataMemoryBarrier();

	return EOK;
}
#endif


static void qspi_IOMode(void)
{
	/* Configure I/O mode */

	/* Disable QSPI */
	*(qspi_common.base + er) &= ~0x1;
	qspi_dataMemoryBarrier();

	/* Disable IRQs */
	*(qspi_common.base + idr) = 0x7d;

	/* Configure controller */

	/* Set master mode, not Legacy mode */
	*(qspi_common.base + cr) = 0x1 | (1u << 31);

	/* Set baud rate to 100 MHz: 200 MHz / 2 */
	*(qspi_common.base + cr) &= ~(0x7 << 3);
	if (QSPI_FCLK < 0) {
		/* Set baud rate to 50 MHz: 200 MHz / 4 */
		*(qspi_common.base + cr) |= (0x1 << 3);
	}

	/* Set little endian */
	*(qspi_common.base + cr) &= ~(1 << 26);
	/* Set FIFO width 32 bits */
	*(qspi_common.base + cr) |= (0x3 << 6);
	/* Set clock phase and polarity */
	*(qspi_common.base + cr) &= ~(0x3 << 1);


	/* Enable manual mode and manual CS */
	*(qspi_common.base + cr) |= (0x3 << 14);

	/* Loopback clock is used for high-speed read data capturing (>40MHz) */
	if (QSPI_FCLK >= 0) {
		*(qspi_common.base + lpbk) = *(qspi_common.base + lpbk) & ~0x3f;
		*(qspi_common.base + lpbk) = (1 << 5);
	}

	/* Disable linear mode */
	*(qspi_common.base + lqspi_cr) = 0;
	qspi_dataMemoryBarrier();
}


static int qspi_setAmbaClk(int dev, unsigned int state)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_ambaclock;

	ctl.ambaclock.dev = dev;
	ctl.ambaclock.state = state;

	return platformctl(&ctl);
}


static int qspi_initClk(void)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devclock;

	ctl.devclock.dev = pctl_ctrl_lqspi_clk;
	ctl.devclock.clkact0 = 0x1;
	ctl.devclock.clkact1 = 0x1;
	ctl.devclock.srcsel = 0;
	ctl.devclock.divisor0 = 5;

	return platformctl(&ctl);
}


static int qspi_setPin(uint32_t pin)
{
	platformctl_t ctl;

	/* Pin should not be configured by the driver */
	if (pin < 0) {
		return EOK;
	}

	if ((pin < pctl_mio_pin_01) && (pin > pctl_mio_pin_08)) {
		return -EINVAL;
	}

	ctl.action = pctl_set;
	ctl.type = pctl_mio;

	ctl.mio.pin = pin;
	ctl.mio.l0 = 0x1;
	ctl.mio.l1 = 0;
	ctl.mio.l2 = 0;
	ctl.mio.l3 = 0;
	ctl.mio.pullup = 0;
	ctl.mio.speed = 0x1;
	ctl.mio.ioType = 0x1;
	ctl.mio.disableRcvr = 0;
	ctl.mio.triEnable = 0;

	if (pin == pctl_mio_pin_01) {
		ctl.mio.pullup = 1;
		ctl.mio.speed = 0x0;
	}

	return platformctl(&ctl);
}


static int qspi_initPins(void)
{
	int res, i;
	static const int pins[] = {
		QSPI_CS,
		QSPI_IO0,
		QSPI_IO1,
		QSPI_IO2,
		QSPI_IO3,
		QSPI_CLK,
		QSPI_FCLK
	};

	for (i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
		/* Pin should not be configured by the driver */
		if (pins[i] < 0) {
			continue;
		}

		res = qspi_setPin(pins[i]);
		if (res < 0) {
			break;
		}
	}

	return res;
}


int qspi_deinit(void)
{
	qspi_stop();

	resourceDestroy(qspi_common.cond);
	resourceDestroy(qspi_common.irqLock);

	munmap((void *)qspi_common.base, _PAGE_SIZE);

	return qspi_setAmbaClk(pctl_amba_lqspi_clk, 0);
}


int qspi_init(void)
{
	int res;

	/* Set IO PLL as source clock and set divider:
	 * IO_PLL / 0x5 :  1000 MHz / 5 = 200 MHz     */
	res = qspi_initClk();
	if (res < 0) {
		return -EIO;
	}

	/* Enable clock */
	res = qspi_setAmbaClk(pctl_amba_lqspi_clk, 1);
	if (res < 0) {
		return -EIO;
	}

	/* Single SS, 4-bit I/O */
	res = qspi_initPins();
	if (res < 0) {
		return -EIO;
	}

	qspi_common.base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0xe000d000);
	if (qspi_common.base == MAP_FAILED) {
		qspi_setAmbaClk(pctl_amba_lqspi_clk, 0);
		return -ENOMEM;
	}

	res = condCreate(&qspi_common.cond);
	if (res < 0) {
		munmap((void *)qspi_common.base, _PAGE_SIZE);
		qspi_setAmbaClk(pctl_amba_lqspi_clk, 0);
		return -ENOENT;
	}

	/* irqLock is only set for the purpose of condWait it doesn't serve synchronization purpose. */
	res = mutexCreate(&qspi_common.irqLock);
	if (res < 0) {
		munmap((void *)qspi_common.base, _PAGE_SIZE);
		resourceDestroy(qspi_common.cond);
		qspi_setAmbaClk(pctl_amba_lqspi_clk, 0);
		return -ENOENT;
	}

	interrupt(51, qspi_irqHandler, NULL, qspi_common.cond, &qspi_common.inth);

	qspi_IOMode();

	return EOK;
}
