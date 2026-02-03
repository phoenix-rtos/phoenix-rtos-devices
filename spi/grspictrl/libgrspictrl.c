/*
 * Phoenix-RTOS
 *
 * GRLIB SPI driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <board_config.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <phoenix/gaisler/ambapp.h>

#if defined(__TARGET_SPARV8LEON)
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#elif defined(__TARGET_RISCV64)
#include <phoenix/arch/riscv64/riscv64.h>
#endif

#include <spi-mst.h>
#include "libgrspictrl.h"


/* clang-format off */
#define TRACE(str, ...) do { if (0) { printf(__FILE__ ":%d: " str "\n", __LINE__, ##__VA_ARGS__); } } while (0)
/* clang-format on */


#define WORD_LEN 8U /* bits */

#define SPI_IOMUX_OPT 0x7U

/* Mode register */
#define SPI_MODE_CORE_EN (1U << 24)
#define SPI_MODE_MASTER  (1U << 25)

#define SPI_MODE_SPEED_MSK ((1U << 27) | (0xfU << 16) | (1U << 13))

/* Event register */
#define SPI_EVENT_TX_NF (1U << 8)
#define SPI_EVENT_RX_NE (1U << 9)
#define SPI_EVENT_LAST  (1U << 14)

/* Command register */
#define SPI_COMM_LAST (1U << 22)

/* Mask register */
#define SPI_MASK_NEE  (1U << 9)
#define SPI_MASK_LAST (1U << 14)


typedef struct {
	int32_t sck;
	int32_t mosi;
	int32_t miso;
	int32_t cs;
} spi_pins_t;


typedef struct spictrl_regs {
	volatile uint32_t cap0; /* 0x00 */
	volatile uint32_t cap1;
	uint32_t reserved1[6];
	volatile uint32_t mode; /* 0x20 */
	volatile uint32_t event;
	volatile uint32_t mask;
	volatile uint32_t cmd;
	volatile uint32_t tx;
	volatile uint32_t rx;
	volatile uint32_t slvsel;
	volatile uint32_t autoslvsel;
} spictrl_regs_t;


__attribute__((section(".interrupt"), aligned(0x1000))) static int irqHandler(unsigned int n, void *arg)
{
	(void)n;
	spi_ctx_t *ctx = (spi_ctx_t *)arg;

	if ((ctx->regs->event & SPI_EVENT_RX_NE) != 0) {
		return 1;
	}

	return -1;
}


static int spi_initPins(unsigned int dev)
{
	static const spi_pins_t pins[] = {
		{ SPI0_SCK, SPI0_MOSI, SPI0_MISO, SPI0_CS },
	};

#if defined(__CPU_GR716)
	const spi_pins_t *cfg = &pins[dev];

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_iomux,
		.task.iocfg = {
			.opt = SPI_IOMUX_OPT,
			.pulldn = 0,
			.pullup = 0,
		}
	};

	pctl.task.iocfg.pin = cfg->sck;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	pctl.task.iocfg.pin = cfg->mosi;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	pctl.task.iocfg.pin = cfg->miso;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	pctl.task.iocfg.pin = cfg->cs;
	if (platformctl(&pctl) < 0) {
		return -1;
	}
#else
	(void)pins;
#endif

	return 0;
}


static uint32_t calcSpeed(uint8_t prescFactor, uint8_t div16, uint8_t prescaler)
{
	return SYSCLK_FREQ / ((4 - 2 * prescFactor) * (prescaler + 1) * ((div16 != 0) ? 16 : 1));
}


static int searchConfig(uint32_t speed, uint8_t *bestPrescFactor, uint8_t *bestDiv16, uint8_t *bestPrescaler)
{
	uint32_t bestSpeed = 0;
	for (uint8_t prescFactor = 0; prescFactor <= 1; prescFactor++) {
		for (uint8_t div16 = 0; div16 <= 1; div16++) {
			for (uint8_t prescaler = 0; prescaler <= 15; prescaler++) {

				uint32_t actualSpeed = calcSpeed(prescFactor, div16, prescaler);

				if ((actualSpeed <= speed) && (actualSpeed > bestSpeed)) {
					bestSpeed = actualSpeed;
					*bestPrescFactor = prescFactor;
					*bestDiv16 = div16;
					*bestPrescaler = prescaler;

					if (bestSpeed == speed) {
						return 0;
					}
				}
			}
		}
	}

	return (bestSpeed == 0) ? -1 : 0;
}


int spi_getSpeed(spi_ctx_t *ctx, uint32_t *speed)
{
	uint32_t cfg = ctx->regs->mode & SPI_MODE_SPEED_MSK;
	uint8_t div16 = (cfg >> 27) & 0x1U;
	uint8_t prescaler = (cfg >> 16) & 0xfU;
	uint8_t prescFactor = (cfg >> 13) & 0x1U;

	*speed = calcSpeed(prescFactor, div16, prescaler);

	return 0;
}


int spi_setSpeed(spi_ctx_t *ctx, uint32_t speed)
{
	if (speed == 0) {
		return -EINVAL;
	}

	uint32_t currSpeed;
	(void)spi_getSpeed(ctx, &currSpeed);
	if (speed == currSpeed) {
		return 0;
	}

	/* SPICLK = SYSCLK_FREQ / ((4 - 2 * prescFactor) * (prescaler + 1) * (div16 ? 16 : 1))
	 * prescFactor = 0 or 1
	 * prescaler = 0..15
	 * div16 = 0 or 1
	 *
	 * Min Divisor: prescFactor=1, div16=0, prescaler=0 -> 2
	 * Max Divisor: prescFactor=0, div16=1, prescaler=15 -> 1024
	 */

	const uint32_t maxSpeed = SYSCLK_FREQ / 2;
	const uint32_t minSpeed = SYSCLK_FREQ / 1024;

	if (speed > maxSpeed) {
		speed = maxSpeed;
	}

	if (speed < minSpeed) {
		return -EINVAL;
	}

	uint8_t bestPrescFactor = 0, bestDiv16 = 0, bestPrescaler = 0;
	if (searchConfig(speed, &bestPrescFactor, &bestDiv16, &bestPrescaler) < 0) {
		return -EINVAL;
	}

	uint32_t reg = ctx->regs->mode;

	uint32_t cfg = (bestDiv16 << 27) | (bestPrescaler << 16) | (bestPrescFactor << 13);

	ctx->regs->mode = 0;

	reg &= ~(SPI_MODE_SPEED_MSK | SPI_MODE_CORE_EN);
	reg |= cfg;

	ctx->regs->mode = reg;

	ctx->regs->mode |= SPI_MODE_CORE_EN;

	return 0;
}


int spi_getMode(spi_ctx_t *ctx, uint8_t *mode)
{
	uint32_t reg = ctx->regs->mode;

	*mode = (reg >> 28) & 0x3U;

	return 0;
}


int spi_setMode(spi_ctx_t *ctx, uint8_t mode)
{
	uint32_t reg = ctx->regs->mode;
	uint32_t currMode = (reg >> 28) & 0x3U;
	if (currMode == mode) {
		return 0;
	}

	ctx->regs->mode = 0;

	reg &= ~(0x3U << 28);
	reg |= ((uint32_t)mode << 28);

	ctx->regs->mode = reg;

	ctx->regs->mode |= SPI_MODE_CORE_EN;

	return 0;
}


int spi_getBitOrder(spi_ctx_t *ctx, uint8_t *bitOrder)
{
	uint32_t reg = ctx->regs->mode;

	*bitOrder = (((reg >> 26) & 0x1U) == 0) ? spi_lsb : spi_msb;

	return 0;
}


int spi_setBitOrder(spi_ctx_t *ctx, uint8_t bitOrder)
{
	uint32_t reg = ctx->regs->mode;
	uint8_t currOrder = (((reg >> 26) & 0x1U) == 0) ? spi_lsb : spi_msb;
	if (currOrder == bitOrder) {
		return 0;
	}

	ctx->bitOrder = bitOrder;

	if (bitOrder == spi_msb) {
		reg |= (0x1U << 26);
	}
	else {
		reg &= ~(0x1U << 26);
	}

	ctx->regs->mode = 0;

	ctx->regs->mode = reg;

	ctx->regs->mode |= SPI_MODE_CORE_EN;

	return 0;
}


static void spi_txByte(spi_ctx_t *ctx, uint8_t byte)
{
	if (ctx->bitOrder == spi_lsb) {
		ctx->regs->tx = byte;
	}
	else {
		ctx->regs->tx = byte << 24;
	}
}


static void spi_rxByte(spi_ctx_t *ctx, uint8_t *rxBuff)
{
	if (ctx->bitOrder == spi_lsb) {
		*rxBuff = (ctx->regs->rx >> 8) & 0xffU;
	}
	else {
		*rxBuff = (ctx->regs->rx >> 16) & 0xffU;
	}
}


static int spi_xfer(spi_ctx_t *ctx, const uint8_t *txBuff, uint8_t *rxBuff, size_t len, bool last)
{
	size_t txWords = 0, chunk = 0, wrote = 0;

	(void)mutexLock(ctx->irqLock);

	while (txWords < len) {

		chunk = min(len - txWords, ctx->fifosz);

		txWords += chunk;

		/* Write to TX FIFO */
		while (chunk > 0) {
			/* If this is last word to write, set LAST bit */
			if (last && (chunk == 1)) {
				ctx->regs->cmd = SPI_COMM_LAST;
			}

			/* Check if we have space in fifo (should basically always be true,
			 * no point in configuring irq for this)
			 */
			if ((ctx->regs->event & SPI_EVENT_TX_NF) != 0) {
				uint8_t txWord = (txBuff != NULL) ? *txBuff++ : 0xffU;
				spi_txByte(ctx, txWord);
				chunk--;
				wrote++;
			}
		}

		/* Read from RX FIFO */
		while (wrote > 0) {
			/* Wait for RX not empty */
			while ((ctx->regs->event & SPI_EVENT_RX_NE) == 0) {
				(void)condWait(ctx->cond, ctx->irqLock, 0);
			}

			uint8_t val;
			spi_rxByte(ctx, &val);
			if (rxBuff != NULL) {
				*rxBuff++ = val;
			}
			wrote--;
		}
	}

	(void)mutexUnlock(ctx->irqLock);

	return 0;
}


static int spi_setSS(spi_ctx_t *ctx, unsigned int ss, bool select)
{
	if (ss == SPI_SS_EXTERNAL) {
		return 0;
	}

	if (ss >= ctx->ssCount) {
		return -EINVAL;
	}

	ctx->regs->slvsel = select ? ~(1U << ss) : 0xffffffffU;

	return 0;
}


int spi_transaction(spi_ctx_t *ctx, uint32_t ss, const spi_transferDesc_t *descriptors, size_t descCnt, const void *txBuf, void *rxBuf)
{
	uint8_t *txPtr = (uint8_t *)txBuf;
	uint8_t *rxPtr = (uint8_t *)rxBuf;

	(void)mutexLock(ctx->mutex);

	int err = spi_setSS(ctx, ss, true);

	for (size_t i = 0; (i < descCnt) && (err == 0); i++) {
		bool last = (i == (descCnt - 1));
		const spi_transferDesc_t *desc = &descriptors[i];

		uint8_t *txSegPtr = NULL;
		uint8_t *rxSegPtr = NULL;

		if ((desc->type == spi_transfer_tx) || (desc->type == spi_transfer_xfer)) {
			if (txPtr == NULL) {
				err = -EINVAL;
				break;
			}
			txSegPtr = txPtr;
			txPtr += desc->len;
		}

		if ((desc->type == spi_transfer_rx) || (desc->type == spi_transfer_xfer)) {
			if (rxPtr == NULL) {
				err = -EINVAL;
				break;
			}
			rxSegPtr = rxPtr;
			rxPtr += desc->len;
		}

		TRACE("xfer type=%u len=%u last=%u\n", desc->type, desc->len, last ? 1 : 0);

		err = spi_xfer(ctx, txSegPtr, rxSegPtr, desc->len, last);

		TRACE("xfer done err=%d\n", err);

		if (desc->delayUs != 0) {
			usleep(desc->delayUs);
		}
	}

	(void)spi_setSS(ctx, ss, false);

	(void)mutexUnlock(ctx->mutex);

	return err;
}


static int spi_cguInit(unsigned int dev)
{
#if defined(__CPU_GR716)
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.task.cguctrl = {
			.v.state = enable,
			.cgu = cgu_primary,
		}
	};

	pctl.task.cguctrl.cgudev = cgudev_spictrl0 + dev;
	return platformctl(&pctl);
#else
	return 0;
#endif
}


int spi_init(unsigned int dev, spi_ctx_t *ctx)
{
	unsigned int instance = dev;
	ambapp_dev_t adev = { .devId = CORE_ID_SPICTRL };
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_ambapp,
		.task.ambapp = {
			.dev = &adev,
			.instance = &instance,
		}
	};

	int ret = platformctl(&pctl);
	if (ret < 0) {
		return ret;
	}

	if (adev.bus != BUS_AMBA_APB) {
		/* SPICTRL should be on APB bus */
		return -EIO;
	}
	ctx->irq = adev.irqn;

	uintptr_t pbase = (uintptr_t)adev.info.apb.base;

	uintptr_t base = (pbase & ~(_PAGE_SIZE - 1));
	ctx->regs = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (ctx->regs == MAP_FAILED) {
		fprintf(stderr, "spi: mmap failed\n");
		return -ENOMEM;
	}

	if (spi_cguInit(dev) < 0) {
		return -EIO;
	}

	if (spi_initPins(dev) < 0) {
		return -EIO;
	}

	ret = condCreate(&ctx->cond);
	if (ret < 0) {
		fprintf(stderr, "spi: condCreate failed\n");
		return ret;
	}

	ret = mutexCreate(&ctx->mutex);
	if (ret < 0) {
		fprintf(stderr, "spi: mutexCreate failed\n");
		return ret;
	}

	ret = mutexCreate(&ctx->irqLock);
	if (ret < 0) {
		fprintf(stderr, "spi: mutexCreate failed\n");
		return ret;
	}

	ctx->regs = (void *)((uintptr_t)ctx->regs + (pbase - base));

	/* Disable core */
	ctx->regs->mode = 0;

	/* clear event */
	ctx->regs->event = 0xffffffffU;

	/* Set common config */
	ctx->regs->mode = SPI_MODE_MASTER | (((WORD_LEN - 1) & 0xffU) << 20);


	/* Enable interrupt on RX Not Empty */
	ctx->regs->mask = SPI_MASK_NEE;

	/* Enable core */
	ctx->regs->mode |= SPI_MODE_CORE_EN;

	ctx->bitOrder = spi_lsb;

	ctx->fifosz = (ctx->regs->cap0 >> 8) & 0xffU;

	ctx->ssCount = (ctx->regs->cap0 >> 24) & 0xffU;

	ret = interrupt(ctx->irq, irqHandler, ctx, ctx->cond, &ctx->inth);
	if (ret < 0) {
		return ret;
	}

	return 0;
}
