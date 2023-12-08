/*
 * Phoenix-RTOS
 *
 * GRLIB SPI driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>
#include <errno.h>
#include <sys/debug.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <posix/utils.h>

#if defined(__CPU_GR716)
#include <phoenix/arch/gr716.h>
#elif defined(__CPU_GR712RC)
#include <phoenix/arch/gr712rc.h>
#else
#error "Unsupported target"
#endif

#include <phoenix/arch/sparcv8leon3.h>

#include "spi.h"
#include "grlib-multi.h"


#define WORD_LEN 8u /* bits */

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
	struct {
		volatile uint32_t *vbase;
		int byteOrder;
		int ready;
		uint8_t fifosz;

		handle_t mutex;
		handle_t irqLock;
		handle_t cond;
		handle_t inth;
	} dev[SPI_CNT];
} spi_common;


static int spi_irqHandler(unsigned int n, void *arg)
{
	int spi = (int)arg;
	volatile uint32_t *spi_base = spi_common.dev[spi].vbase;

	(void)n;

	if ((*(spi_base + SPI_EVENT) & SPI_EVENT_LAST) != 0) {
		*(spi_base + SPI_EVENT) = SPI_EVENT_LAST;
		spi_common.dev[spi].ready = 1;
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
	volatile uint32_t *spi_base = spi_common.dev[dev].vbase;
	uint8_t cpol = (config->mode & 0x2) >> 1;
	uint8_t cpha = (config->mode & 0x1);
	uint32_t mode = (cpol << 29) | (cpha << 28) | ((config->div16 & 0x1) << 27) | ((config->byteOrder & 0x1) << 26) |
		(SPI_MODE_MASTER) | (((WORD_LEN - 1) & 0xff) << 20) | ((config->prescaler & 0xF) << 16) |
		((config->prescFactor & 0x1) << 13);

#ifdef GRLIB_SPI_LOOPBACK
	/* Loop-back mode (for tests) */
	mode |= (1 << 30);
#endif

	(void)mutexLock(spi_common.dev[dev].mutex);

	spi_common.dev[dev].byteOrder = config->byteOrder;

	spi_common.dev[dev].ready = 0;

	/* Disable core */
	*(spi_base + SPI_MODE) = 0;

	*(spi_base + SPI_MODE) = mode;

	/* Enable core */
	*(spi_base + SPI_MODE) |= SPI_MODE_CORE_EN;

	/* Enable LAST bit interrupt */
	*(spi_base + SPI_MASK) = SPI_MASK_LAST;

	(void)mutexUnlock(spi_common.dev[dev].mutex);

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
	volatile uint32_t *spi_base = spi_common.dev[dev].vbase;

	(void)mutexLock(spi_common.dev[dev].mutex);

	/* Set slave select */
	*(spi_base + SPI_SLVSEL) = ~slavemsk;

	while (txWords < len) {
		(void)mutexLock(spi_common.dev[dev].irqLock);

		chunk = min(len - txWords, spi_common.dev[dev].fifosz);

		txWords += chunk;

		/* Write to TX FIFO */
		while (chunk > 0) {
			/* If this is last word to write, set LAST bit */
			if (chunk == 1) {
				*(spi_base + SPI_COMMAND) = SPI_COMM_LAST;
			}
			spi_txByte(spi_base, spi_common.dev[dev].byteOrder, *txBuff++);
			chunk--;
			wrote++;
		}

		/* Wait until LAST bit is cleared by interrupt handler */
		while (spi_common.dev[dev].ready == 0) {
			(void)condWait(spi_common.dev[dev].cond, spi_common.dev[dev].irqLock, 0);
		}

		spi_common.dev[dev].ready = 0;

		(void)mutexUnlock(spi_common.dev[dev].irqLock);

		/* Read from RX FIFO */
		while (wrote > 0) {
			if ((*(spi_base + SPI_EVENT) & SPI_EVENT_RX_NE) != 0) {
				spi_rxByte(spi_base, spi_common.dev[dev].byteOrder, rxBuff++);
				wrote--;
			}
		}
	}
	/* Clear slave select */
	*(spi_base + SPI_SLVSEL) = 0xfu;

	(void)mutexUnlock(spi_common.dev[dev].mutex);

	return 0;
}


static void spi_handleDevCtl(msg_t *msg, int dev)
{
	multi_i_t *idevctl = (multi_i_t *)msg->i.raw;
	multi_o_t *odevctl = (multi_o_t *)msg->o.raw;

	const uint8_t *txBuff = (const uint8_t *)msg->i.data;
	uint8_t *rxBuff = (uint8_t *)msg->o.data;

	dev -= id_spi0;

	if ((dev >= SPI_CNT) || (dev < 0)) {
		odevctl->err = -EINVAL;
		return;
	}

	switch (idevctl->spi.type) {
		case spi_setPins:
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


static int spi_cguInit(int dev)
{
#if defined(__CPU_GR716)
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.cguctrl = {
			.state = enable,
			.cgu = cgu_primary,
		}
	};

	pctl.cguctrl.cgudev = cgudev_spictrl0 + dev;
	return platformctl(&pctl);
#else
	return 0;
#endif
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


int spi_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < SPI_CNT; i++) {
		char buf[8];
		if (snprintf(buf, sizeof(buf), "spi%u", i) >= sizeof(buf)) {
			return -1;
		}

		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


int spi_init(void)
{
	struct {
		volatile uint32_t *base;
		unsigned int irq;
	} spi_info[SPI_CNT];

	for (unsigned int i = 0; i < SPI_CNT; i++) {
		unsigned int instance = i;
		ambapp_dev_t dev = { .devId = CORE_ID_SPICTRL };
		platformctl_t pctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.ambapp = {
				.dev = &dev,
				.instance = &instance,
			}
		};

		if (platformctl(&pctl) < 0) {
			return -1;
		}

		if (dev.bus != BUS_AMBA_APB) {
			/* SPICTRL should be on APB bus */
			return -1;
		}
		spi_info[i].base = dev.info.apb.base;
		spi_info[i].irq = dev.irqn;

		uintptr_t base = ((uintptr_t)spi_info[i].base & ~(_PAGE_SIZE - 1));
		spi_common.dev[i].vbase = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
		if (spi_common.dev[i].vbase == MAP_FAILED) {
			return -1;
		}

		if (spi_cguInit(i) < 0) {
			munmap((void *)spi_common.dev[i].vbase, _PAGE_SIZE);
			return -1;
		}

		if (condCreate(&spi_common.dev[i].cond) < 0) {
			munmap((void *)spi_common.dev[i].vbase, _PAGE_SIZE);
			return -1;
		}

		if (mutexCreate(&spi_common.dev[i].mutex) < 0) {
			munmap((void *)spi_common.dev[i].vbase, _PAGE_SIZE);
			resourceDestroy(spi_common.dev[i].cond);
			return -1;
		}

		if (mutexCreate(&spi_common.dev[i].irqLock) < 0) {
			munmap((void *)spi_common.dev[i].vbase, _PAGE_SIZE);
			resourceDestroy(spi_common.dev[i].cond);
			resourceDestroy(spi_common.dev[i].mutex);
			return -1;
		}

		spi_common.dev[i].vbase += ((uintptr_t)spi_info[i].base - base) / sizeof(uintptr_t);

		/* Disable module */
		*(spi_common.dev[i].vbase + SPI_MODE) = 0;

		spi_common.dev[i].byteOrder = spi_lsb;

		spi_common.dev[i].fifosz = (*(spi_common.dev[i].vbase + SPI_CAPABILITY) >> 8) & 0xff;

		interrupt(spi_info[i].irq, spi_irqHandler, (void *)i, spi_common.dev[i].cond, &spi_common.dev[i].inth);
	}

	return 0;
}
