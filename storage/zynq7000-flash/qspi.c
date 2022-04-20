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

#include <phoenix/arch/zynq7000.h>


enum { cr = 0, sr, ier, idr, imr, er, dr, txd00, rxd, sicr, txth, rxth, gpio,
	   lpbk = 0xe, txd01 = 0x20, txd10, txd11,
	   lqspi_cr = 0x28, lqspi_sr, modid = 0x3f };


struct {
	volatile uint32_t *base;
	handle_t cond;
	handle_t inth;
	handle_t irqLock;
	handle_t lock;
} qspi_common;


static inline void qspi_dataMemoryBarrier(void)
{
	__asm__ volatile ("dmb");
}


static int qspi_irqHandler(unsigned int n, void *arg)
{
	*(qspi_common.base + idr) |= (1 << 2);

	return 1;
}


static inline time_t qspi_timeMsGet(void)
{
	time_t now;
	gettime(&now, NULL);

	return now / 1000;
}


/* TODO: add timeout exit condition to be compliant with MISRA */
static inline void qspi_cleanFifo(void)
{
	/* Clean up RX Fifo */
	while ((*(qspi_common.base + sr) & (1 << 4))) {
		*(qspi_common.base + rxd);
	}

	*(qspi_common.base + cr) |= (1 << 10);
	qspi_dataMemoryBarrier();
	*(qspi_common.base + er) = 0;
	qspi_dataMemoryBarrier();
}


void qspi_stop(void)
{
	qspi_cleanFifo();
	mutexUnlock(qspi_common.lock);
}


void qspi_start(void)
{
	mutexLock(qspi_common.lock);

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

	return (size < 4) ? size : 4;
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


ssize_t qspi_transfer(const uint8_t *txBuff, uint8_t *rxBuff, size_t size, time_t timeout)
{
	int err;
	size_t tempSz, txSz = size, rxSz = 0;
	time_t interval = 0, start = 0;

	if (timeout != 0) {
		start = qspi_timeMsGet();
	}

	while (txSz || rxSz) {
		/* Transmit data */
		while (txSz) {
			/* Incomplete word has to be send and receive as a last transfer
			 * otherwise there is an undefined behaviour.                   */
			if ((txSz < sizeof(uint32_t)) && (rxSz >= sizeof(uint32_t))) {
				break;
			}

			/* TX Fifo is full */
			if ((*(qspi_common.base + sr) & (1 << 3))) {
				break;
			}

			tempSz = qspi_txData(txBuff, txSz);

			txSz -= tempSz;
			rxSz += tempSz;

			if (txBuff != NULL) {
				txBuff += tempSz;
			}
		}

		/* TX FIFO not full enable irq */
		*(qspi_common.base + ier) |= (1 << 2);
		/* Start data transmission */
		*(qspi_common.base + cr) |= (1 << 16);

		/* Wait until TX Fifo is empty */
		mutexLock(qspi_common.irqLock);
		while ((*(qspi_common.base + sr) & (1 << 2)) == 0) {
			err = condWait(qspi_common.cond, qspi_common.irqLock, (timeout - interval) * 1000);
		}
		mutexUnlock(qspi_common.irqLock);

		if (err < 0) {
			return err;
		}

		/* Receive data */
		while (rxSz) {
			/* RX Fifo is empty */
			if ((*(qspi_common.base + sr) & (1 << 4)) == 0) {
				break;
			}

			tempSz = qspi_rxData(rxBuff, rxSz);
			rxSz -= tempSz;

			if (rxBuff != NULL) {
				rxBuff += tempSz;
			}
		}

		if (timeout != 0) {
			interval = qspi_timeMsGet() - start;
			if (interval > timeout) {
				return -ETIMEDOUT;
			}
		}
	}

	return size - txSz;
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

	/* Set baud rate, master mode, not Legacy mode */
	*(qspi_common.base + cr) &= ~(0x1 << 3);
	*(qspi_common.base + cr) |= (0x1 << 3) | 0x1 | (1 << 31);
	/* Set little endian */
	*(qspi_common.base + cr) &= ~(1 << 26);
	/* Set FIFO width 32 bits */
	*(qspi_common.base + cr) |= (0x3 << 6);
	/* Set clock phase and polarity */
	*(qspi_common.base + cr) &= ~(0x3 << 1);


	/* Enable manual mode and manual CS */
	*(qspi_common.base + cr) |= (0x3 << 14);

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

	if (pin < pctl_mio_pin_01 && pin > pctl_mio_pin_06) {
		return -EINVAL;
	}

	ctl.action = pctl_set;
	ctl.type = pctl_mio;

	ctl.mio.pin = pin;
	ctl.mio.l0 = 0x1;
	ctl.mio.l1 = ctl.mio.l2 = ctl.mio.l3 = 0;
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
		pctl_mio_pin_01, /* Chip Select */
		pctl_mio_pin_02, /* I/O */
		pctl_mio_pin_03, /* I/O */
		pctl_mio_pin_04, /* I/O */
		pctl_mio_pin_05, /* I/O */
		pctl_mio_pin_06  /* CLK */
	};

	for (i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
		res = qspi_setPin(pins[i]);
		if (res < 0) {
			break;
		}
	}

	return res;
}


int qspi_deinit(void)
{
	qspi_cleanFifo();

	resourceDestroy(qspi_common.cond);
	resourceDestroy(qspi_common.lock);
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

	qspi_common.base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, 0xe000d000);
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

	res = mutexCreate(&qspi_common.irqLock);
	if (res < 0) {
		munmap((void *)qspi_common.base, _PAGE_SIZE);
		resourceDestroy(qspi_common.cond);
		qspi_setAmbaClk(pctl_amba_lqspi_clk, 0);
		return -ENOENT;
	}

	res = mutexCreate(&qspi_common.lock);
	if (res < 0) {
		munmap((void *)qspi_common.base, _PAGE_SIZE);
		resourceDestroy(qspi_common.cond);
		resourceDestroy(qspi_common.irqLock);
		qspi_setAmbaClk(pctl_amba_lqspi_clk, 0);
		return -ENOENT;
	}

	interrupt(51, qspi_irqHandler, NULL, qspi_common.cond, &qspi_common.inth);

	qspi_IOMode();

	return EOK;
}
