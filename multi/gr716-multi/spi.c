/*
 * Phoenix-RTOS
 *
 * GR716 SPI driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <sys/debug.h>
#include <sys/minmax.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <phoenix/arch/gr716.h>

#include "common.h"
#include "spi.h"
#include "gr716-multi.h"


#define SPI_CNT 2
#define FIFO_SZ 0x10u

#define WORD_LEN 8u /* bits */

#define SPICTRL0_BASE ((void *)0x80309000)
#define SPICTRL1_BASE ((void *)0x8030A000)

#define SPICTRL0_IRQ 11
#define SPICTRL1_IRQ 12

#define SPI_IOMUX_OPT 0x7u

/* SPI registers */
#define SPI_CAPABILITY 0
#define SPI_MODE       8
#define SPI_EVENT      9
#define SPI_MASK       10
#define SPI_COMMAND    11
#define SPI_TX         12
#define SPI_RX         13
#define SPI_SLVSEL     14
#define SPI_AUTOSLVSEL 15

/* Mode register */
#define SPI_MODE_CORE_EN (1 << 24)
#define SPI_MODE_MASTER  (1 << 25)

/* Event register */
#define SPI_EVENT_TX_NF (1 << 8)
#define SPI_EVENT_RX_NE (1 << 9)
#define SPI_EVENT_LAST  (1 << 14)

/* Command register */
#define SPI_COMM_LAST (1 << 22)

/* Mask register */
#define SPI_MASK_LAST (1 << 14)


static struct {
	volatile uint32_t *base;
	int byteOrder;
	int ready;

	handle_t mutex;
	handle_t irqLock;
	handle_t cond;
	handle_t inth;
} spi_common[SPI_CNT];


static int spi_irqHandler(unsigned int n, void *arg)
{
	int spi = (int)arg;
	volatile uint32_t *spi_base = spi_common[spi].base;

	(void)n;

	if ((*(spi_base + SPI_EVENT) & SPI_EVENT_LAST) != 0) {
		*(spi_base + SPI_EVENT) = SPI_EVENT_LAST;
		spi_common[spi].ready = 1;
		return 1;
	}

	return 0;
}


static int spi_initPins(spi_pins_t *pins)
{
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iocfg = {
			.opt = SPI_IOMUX_OPT,
			.pulldn = 0,
			.pullup = 0,
		}
	};

	pctl.iocfg.pin = pins->sck;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	pctl.iocfg.pin = pins->mosi;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	pctl.iocfg.pin = pins->miso;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	pctl.iocfg.pin = pins->cs;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	return 0;
}


static int spi_configure(int dev, spi_config_t *config)
{
	volatile uint32_t *spi_base = spi_common[dev].base;
	uint8_t cpol = (config->mode & 0x2) >> 1;
	uint8_t cpha = (config->mode & 0x1);
	uint32_t mode = (cpol << 29) | (cpha << 28) | ((config->div16 & 0x1) << 27) | ((config->byteOrder & 0x1) << 26) |
		(SPI_MODE_MASTER) | (((WORD_LEN - 1) & 0xff) << 20) | ((config->prescaler & 0xF) << 16) |
		((config->prescFactor & 0x1) << 13);

#if 0
	/* Loop-back mode (for tests) */
	mode |= (1 << 30);
#endif

	(void)mutexLock(spi_common[dev].mutex);

	spi_common[dev].byteOrder = config->byteOrder;

	spi_common[dev].ready = 0;

	/* Disable core */
	*(spi_base + SPI_MODE) = 0;

	*(spi_base + SPI_MODE) = mode;

	/* Enable core */
	common_atomicOr(spi_base + SPI_MODE, SPI_MODE_CORE_EN);

	/* Enable LAST bit interrupt */
	*(spi_base + SPI_MASK) = SPI_MASK_LAST;

	(void)mutexUnlock(spi_common[dev].mutex);

	return 0;
}


static void spi_txByte(volatile uint32_t *spi_base, uint8_t endian, uint8_t byte)
{
	if (endian == spi_lsb) {
		*(spi_base + SPI_TX) = byte;
	}
	else {
		*(spi_base + SPI_TX) = byte << 24;
	}
}


static void spi_rxByte(volatile uint32_t *spi_base, uint8_t endian, uint8_t *rxBuff)
{
	if (endian == spi_lsb) {
		*rxBuff = (*(spi_base + SPI_RX) >> 8) & 0xFF;
	}
	else {
		*rxBuff = (*(spi_base + SPI_RX) >> 16) & 0xFF;
	}
}


static int spi_xfer(int dev, uint8_t slavemsk, const uint8_t *txBuff, uint8_t *rxBuff, size_t len)
{
	size_t txWords = 0, chunk = 0, wrote = 0;
	volatile uint32_t *spi_base = spi_common[dev].base;

	(void)mutexLock(spi_common[dev].mutex);

	/* Set slave select */
	*(spi_base + SPI_SLVSEL) = ~slavemsk;

	while (txWords < len) {
		(void)mutexLock(spi_common[dev].irqLock);

		chunk = min(len - txWords, FIFO_SZ);

		txWords += chunk;

		/* Write to TX FIFO */
		while (chunk > 0) {
			/* If this is last word to write, set LAST bit */
			if (chunk == 1) {
				*(spi_base + SPI_COMMAND) = SPI_COMM_LAST;
			}
			spi_txByte(spi_base, spi_common[dev].byteOrder, *txBuff++);
			chunk--;
			wrote++;
		}

		/* Wait until LAST bit is cleared by interrupt handler */
		while (spi_common[dev].ready == 0) {
			(void)condWait(spi_common[dev].cond, spi_common[dev].irqLock, 0);
		}

		spi_common[dev].ready = 0;

		(void)mutexUnlock(spi_common[dev].irqLock);

		/* Read from RX FIFO */
		while (wrote > 0) {
			if ((*(spi_base + SPI_EVENT) & SPI_EVENT_RX_NE) != 0) {
				spi_rxByte(spi_base, spi_common[dev].byteOrder, rxBuff++);
				wrote--;
			}
		}
	}
	/* Clear slave select */
	*(spi_base + SPI_SLVSEL) = 0xfu;

	(void)mutexUnlock(spi_common[dev].mutex);

	return 0;
}


static void spi_handleDevCtl(msg_t *msg, int dev)
{
	multi_i_t *idevctl = (multi_i_t *)msg->i.raw;
	multi_o_t *odevctl = (multi_o_t *)msg->o.raw;

	const uint8_t *txBuff = (const uint8_t *)msg->i.data;
	uint8_t *rxBuff = (uint8_t *)msg->o.data;

	dev -= id_spi0;

	if (dev >= SPI_CNT) {
		odevctl->err = -EINVAL;
		return;
	}

	switch (idevctl->spi.type) {
		case spi_set_pins:
			odevctl->err = spi_initPins(&idevctl->spi.pins);
			break;

		case spi_config:
			odevctl->err = spi_configure(dev, &idevctl->spi.config);
			break;

		case spi_transaction:
			odevctl->err = spi_xfer(dev, idevctl->spi.transaction.slaveMsk, txBuff, rxBuff, idevctl->spi.transaction.len);
			break;

		default:
			odevctl->err = -ENOSYS;
			break;
	}
}


void spi_handleMsg(msg_t *msg, int dev)
{
	switch (msg->type) {
		case mtWrite:
		case mtRead:
			msg->o.io.err = EOK;
			break;

		case mtDevCtl:
			spi_handleDevCtl(msg, dev);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


int spi_init(void)
{
	int res = 0;

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.cguctrl = {
			.state = enable,
			.cgu = cgu_primary,
		}
	};

	static const struct {
		volatile uint32_t *base;
		unsigned int irq;
	} spiInfo[] = {
		{ SPICTRL0_BASE, SPICTRL0_IRQ },
		{ SPICTRL1_BASE, SPICTRL1_IRQ },
	};

	for (int i = 0; i < SPI_CNT; i++) {
		spi_common[i].base = spiInfo[i].base;

		/* Enable clock */
		pctl.cguctrl.cgudev = cgudev_spictrl0 + i;
		res = platformctl(&pctl);
		if (res < 0) {
			return res;
		}

		res = condCreate(&spi_common[i].cond);
		if (res < 0) {
			return -ENOENT;
		}

		res = mutexCreate(&spi_common[i].mutex);
		if (res < 0) {
			resourceDestroy(spi_common[i].cond);
			return -ENOENT;
		}

		res = mutexCreate(&spi_common[i].irqLock);
		if (res < 0) {
			resourceDestroy(spi_common[i].cond);
			resourceDestroy(spi_common[i].mutex);
			return -ENOENT;
		}

		/* Disable module */
		*(spi_common[i].base + SPI_MODE) = 0;

		spi_common[i].byteOrder = spi_lsb;

		interrupt(spiInfo[i].irq, spi_irqHandler, (void *)i, spi_common[i].cond, &spi_common[i].inth);
	}

	return 0;
}
