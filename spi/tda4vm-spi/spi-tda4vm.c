/*
 * Phoenix-RTOS
 *
 * TDA4VM MCU SPI driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Rafal Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/mman.h>

#include "spi-tda4vm.h"
#include "spi-reg.h"


#define DISABLE 0U
#define ENABLE  1U

#define CHANNEL_STRIDE    5U
#define THREAD_PRIORITY   4U
#define THREAD_STACK_SIZE 128U

#define MCSPI_FIFO_FULL_SIZE 64U
#define MCSPI_FIFO_HALF_SIZE 32U

#define MCSPI_XFER_ONGOING 0U
#define MCSPI_XFER_CMPLT   1U

#define MCSPI_DEVICE_IDLE 0U
#define MCSPI_DEVICE_BUSY 1U

#define MCSPI_IRQENABLE_SET(IRQ, channel) (1 << (IRQ + (channel * 4U)))


static struct mcspi_dev mcspiDev[3] = { 0 };

static struct mcspi_hwctx mcspi_hwctx[] = {
	{ .baseAddr = MCU_MCSPI0_CFG, .irqNum = MCU_MCSPI0_INTR },
	{ .baseAddr = MCU_MCSPI1_CFG, .irqNum = MCU_MCSPI1_INTR },
	{ .baseAddr = MCU_MCSPI2_CFG, .irqNum = MCU_MCSPI2_INTR }
};


static void mcspi_softReset(struct mcspi_dev *dev)
{
	*(dev->base + sysconfig) |= (1U << MCSPI_SYSCONFIG_SOFTRESET);

	while ((*(dev->base + sysstatus) & (1U << MCSPI_SYSSTATUS_RESETDONE)) == 0U) {
		/* busy wait */
	};
}


static void mcspi_modulCtrl(struct mcspi_dev *dev)
{
	struct mcspi_modulctrl *modctrl = &dev->mode;
	uint16_t reg_modctrl = ((modctrl->single & 1U) << MCSPI_MODCTRL_SINGLE) |
			((modctrl->pin34 & 1U) << MCSPI_MODCTRL_PIN34) |
			((modctrl->ms & 1U) << MCSPI_MODCTRL_MS) |
			((modctrl->system_test & 1U) << MCSPI_MODCTRL_SYSTEST) |
			((modctrl->initdly & 7U) << MCSPI_MODCTRL_INITDLY) |
			((modctrl->moa & 1U) << MCSPI_MODCTRL_MOA);

	*(dev->base + modulctrl) = (uint32_t)reg_modctrl;
}


static void mcspi_sysconfig(struct mcspi_dev *dev)
{
	uint16_t reg_sysconf = (DISABLE << MCSPI_SYSCONFIG_AUTOIDLE) |
			(DISABLE << MCSPI_SYSCONFIG_SOFTRESET) |
			(DISABLE << MCSPI_SYSCONFIG_ENWAKEUP) |
			(1U << MCSPI_SYSCONFIG_SIDLE) |
			(3U << MCSPI_SYSCONFIG_CLOCKACT);

	*(dev->base + sysconfig) = (uint32_t)reg_sysconf;
}


static void mcspi_chconf(struct mcspi_dev *dev, struct mcspi_chconf *cnf)
{
	uint32_t reg_chconf = (cnf->pha << MCSPI_CHCONF_PHA) | (cnf->pol << MCSPI_CHCONF_POL) | (cnf->clkd << MCSPI_CHCONF_CLKD) |
			(cnf->epol << MCSPI_CHCONF_EPOL) | (cnf->wl << MCSPI_CHCONF_WL) | (cnf->trm << MCSPI_CHCONF_TRM) |
			(cnf->dmaw << MCSPI_CHCONF_DMAW) | (cnf->dmar << MCSPI_CHCONF_DMAR) | (cnf->dpe0 << MCSPI_CHCONF_DPE0) |
			(cnf->dpe1 << MCSPI_CHCONF_DPE1) | (cnf->is << MCSPI_CHCONF_IS) | (cnf->turbo << MCSPI_CHCONF_TURBO) |
			(cnf->force << MCSPI_CHCONF_FORCE) | (cnf->spienslv << MCSPI_CHCONF_SPIENSLV) | (cnf->sbe << MCSPI_CHCONF_SBE) |
			(cnf->sbol << MCSPI_CHCONF_SBOL) | (cnf->tcs0 << MCSPI_CHCONF_TCS0) | (cnf->ffew << MCSPI_CHCONF_FFEW) |
			(cnf->ffer << MCSPI_CHCONF_FFER) | (cnf->clkg << MCSPI_CHCONF_CLKG);

	*(dev->base + chconf + dev->choffs) = reg_chconf;
}


static void mcspi_resourceDestroy(struct mcspi_dev *dev)
{
	(void)resourceDestroy(dev->irqCond);
	(void)resourceDestroy(dev->devCond);
	(void)resourceDestroy(dev->irqMutex);
	(void)resourceDestroy(dev->devMutex);
	(void)munmap((void *)dev->base, PAGE_SIZE);
}


static void mcspi_thread(void *arg)
{
	struct mcspi_dev *dev = (struct mcspi_dev *)arg;

	mutexLock(dev->irqMutex);
	while (dev->initialized == 1U) {
		condWait(dev->irqCond, dev->irqMutex, 0);

		/* wait for end of transmission event */
		while ((*(dev->base + chstat + dev->choffs) & (1 << MCSPI_CHSTAT_EOT)) == 0U) {
			usleep(10);
		};

		/* disable channel */
		*(dev->base + chctrl + dev->choffs) &= ~(1 << MCSPI_CHCTRL_EN);

		mutexLock(dev->devMutex);
		dev->xfer.cmplt = MCSPI_XFER_CMPLT;
		condSignal(dev->devCond);
		mutexUnlock(dev->devMutex);
	}

	mutexUnlock(dev->irqMutex);


	endthread();
}


static void *mcspi_receiveWord(struct mcspi_dev *dev)
{
	struct mcspi_xfer *xfer = &dev->xfer;
	void *dataBuff = xfer->req->rxBuff;

	if (xfer->req->wordSize <= 8U) {
		*(uint8_t *)dataBuff = (uint8_t) * (dev->base + rx + dev->choffs);
		dataBuff = (void *)((uint8_t *)dataBuff + 1U);
	}
	else if (xfer->req->wordSize <= 16U) {
		*(uint16_t *)dataBuff = (uint16_t) * (dev->base + rx + dev->choffs);
		dataBuff = (void *)((uint16_t *)dataBuff + 1U);
	}
	else {
		*(uint32_t *)dataBuff = (uint32_t) * (dev->base + rx + dev->choffs);
		dataBuff = (void *)((uint32_t *)dataBuff + 1U);
	}

	return dataBuff;
}


static void *mcspi_transmitWord(struct mcspi_dev *dev)
{
	struct mcspi_xfer *xfer = &dev->xfer;
	void *dataBuff = xfer->req->txBuff;

	if (xfer->req->wordSize <= 8U) {
		*(dev->base + tx + dev->choffs) = (uint32_t)(*(uint8_t *)dataBuff);
		dataBuff = (void *)((uint8_t *)dataBuff + 1U);
	}
	else if (xfer->req->wordSize <= 16U) {
		*(dev->base + tx + dev->choffs) = (uint32_t)(*(uint16_t *)dataBuff);
		dataBuff = (void *)((uint16_t *)dataBuff + 1U);
	}
	else {
		*(dev->base + tx + dev->choffs) = (uint32_t)(*(uint32_t *)dataBuff);
		dataBuff = (void *)((uint32_t *)dataBuff + 1U);
	}

	return dataBuff;
}


static int mcspi_intr(unsigned int intr, void *data)
{
	struct mcspi_dev *dev = (struct mcspi_dev *)data;
	struct mcspi_xfer *xfer = &dev->xfer;
	uint32_t irqStatus = *(dev->base + irqstatus);
	uint32_t chStatus;
	uint8_t proceed = 1U;
	uint32_t reg, xferByte;

	/* tx-rx data from FIFO */
	while (((xfer->xferCntRx != 0U) || (xfer->xferCntTx != 0U)) && (proceed == 1U)) {
		proceed = 0U;
		chStatus = *(dev->base + chstat + dev->choffs);
		/* rx */
		if (xfer->xferCntRx != 0U) {
			if (((chStatus & (1 << MCSPI_CHSTAT_RXFFE)) == 0U) || ((chStatus & (1 << MCSPI_CHSTAT_RXS)) != 0U)) {
				xfer->req->rxBuff = mcspi_receiveWord(dev);
				xfer->xferCntRx--;
				proceed = 1U;
			}
		}

		/* tx */
		if (xfer->xferCntTx != 0U) {
			if ((chStatus & (1 << MCSPI_CHSTAT_TXFFF)) == 0U) {
				xfer->req->txBuff = mcspi_transmitWord(dev);
				xfer->xferCntTx--;
				proceed = 1U;
			}
		}
	}

	/* adjust fifo trigger levels */
	xferByte = xfer->xferCntTx << xfer->wordShift;
	if ((xfer->xferCntTx != 0U) && (xferByte <= xfer->txTrig)) {
		xfer->txTrig = xferByte;
	}

	xferByte = xfer->xferCntRx << xfer->wordShift;
	if ((xfer->xferCntRx > 1U) && (xferByte <= xfer->rxTrig)) {
		xfer->rxTrig = xferByte;
	}
	reg = (uint32_t)(((xfer->txTrig - 1U) & 0xFFU) << MCSPI_XFERLEVEL_AEL);
	reg |= (uint32_t)(((xfer->rxTrig - 1U) & 0xFFU) << MCSPI_XFERLEVEL_AFL);
	*(dev->base + xferlevel) &= ~65535UL;
	*(dev->base + xferlevel) |= reg;

	/* EOW interrupt and channel disabling */
	if (((irqStatus & (1 << MCSPI_IRQENABLE_EOW)) != 0U) ||
			((xfer->xferCntTx == 0U) && (xfer->xferCntRx <= 1U))) {
		/* receive last data from FIFO */
		chStatus = *(dev->base + chstat + dev->choffs);
		while (((chStatus & (1 << MCSPI_CHSTAT_RXFFE)) == 0U || (chStatus & (1 << MCSPI_CHSTAT_RXS)) != 0U) && xfer->xferCntRx != 0U) {
			xfer->req->rxBuff = mcspi_receiveWord(dev);
			xfer->xferCntRx--;
			chStatus = *(dev->base + chstat + dev->choffs);
		}

		/* disable interrupts */
		reg = MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_RX_FULL, 0) |
				MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_TX_EMPTY, 0) |
				MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_EOW, 0);

		*(dev->base + irqenable) &= ~reg;
		*(dev->base + irqstatus) = irqStatus;

		return 0;
	}

	*(dev->base + irqstatus) = irqStatus;

	return -1;
}


static void _mcspi_rxInit(struct mcspi_dev *dev)
{
	uint32_t reg;

	/* clear RX FULL interrupt status */
	*(dev->base + irqstatus) = (uint32_t)MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_RX_FULL, 0);

	/* enable RX_FULL IRQ */
	reg = *(dev->base + irqenable);

	reg |= MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_RX_FULL, 0);
	*(dev->base + irqenable) = reg;
}


static void _mcspi_txInit(struct mcspi_dev *dev)
{
	uint32_t reg, wordCnt;
	uint8_t cnt;
	struct mcspi_xfer *xfer = &dev->xfer;

	/* preload tx fifo with data */
	if (dev->type == MCSPI_DEVICE_SLAVE) {
		wordCnt = (xfer->txTrig) >> xfer->wordShift;
		for (cnt = 0; cnt < wordCnt; cnt++) {
			xfer->req->txBuff = mcspi_transmitWord(dev);
			xfer->xferCntTx--;
		}
	}

	*(dev->base + irqstatus) = (uint32_t)MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_TX_EMPTY, 0) |
			(uint32_t)MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_TX_UNDERFLOW, 0);

	/* enable TX EMPTY interrupt */
	reg = *(dev->base + irqenable);

	reg |= MCSPI_IRQENABLE_SET(MCSPI_IRQENABLE_TX_EMPTY, 0);
	*(dev->base + irqenable) = reg;
}


static void _mcspi_txrxInit(struct mcspi_dev *dev)
{
	_mcspi_rxInit(dev);
	_mcspi_txInit(dev);
}


int mcspi_xferStart(struct mcspi_xferReq *xferReq)
{
	struct mcspi_dev *dev = &mcspiDev[xferReq->devnum];
	struct mcspi_xfer *xfer;
	uint32_t fifoState;
	uint16_t xferBytes;
	uint32_t reg;

	mutexLock(dev->devMutex);
	if (dev->xferBusy == MCSPI_DEVICE_BUSY) {
		mutexUnlock(dev->devMutex);
		return -EBUSY;
	}
	memset(&dev->xfer, 0, sizeof(struct mcspi_xfer));

	dev->xfer.req = xferReq;
	xfer = &dev->xfer;
	dev->xferBusy = MCSPI_DEVICE_BUSY;
	xfer->cmplt = MCSPI_XFER_ONGOING;

	/* set the transmission mode */
	*(dev->base + chconf + dev->choffs) &= ~(0x3 << MCSPI_CHCONF_TRM);
	*(dev->base + chconf + dev->choffs) |= ((xferReq->xferType & 0x3U) << MCSPI_CHCONF_TRM);

	/* set the word length */
	if (xferReq->wordSize < 4U || xferReq->wordSize > 32U) {
		return -EINVAL;
	}

	*(dev->base + chconf + dev->choffs) &= ~(0x1F << MCSPI_CHCONF_WL);
	*(dev->base + chconf + dev->choffs) |= (((xferReq->wordSize - 1U) & 0x1FU) << MCSPI_CHCONF_WL);

	/* configure dev params & enable FIFO */
	switch (xferReq->xferType) {
		case MCSPI_XFER_TXRX:
			fifoState = (1 << MCSPI_CHCONF_FFER) | (1 << MCSPI_CHCONF_FFEW);
			xfer->rxTrig = MCSPI_FIFO_HALF_SIZE;
			xfer->txTrig = MCSPI_FIFO_HALF_SIZE;
			xfer->fifoSize = MCSPI_FIFO_HALF_SIZE;
			xfer->xferCntTx = xfer->xferCntRx = xferReq->wordCount;
			break;
		case MCSPI_XFER_RX:
			fifoState = (1 << MCSPI_CHCONF_FFER);
			xfer->rxTrig = MCSPI_FIFO_FULL_SIZE;
			xfer->fifoSize = MCSPI_FIFO_FULL_SIZE;
			xfer->xferCntRx = xferReq->wordCount;
			break;
		case MCSPI_XFER_TX:
			fifoState = (1 << MCSPI_CHCONF_FFEW);
			xfer->txTrig = MCSPI_FIFO_FULL_SIZE;
			xfer->fifoSize = MCSPI_FIFO_FULL_SIZE;
			xfer->xferCntTx = xferReq->wordCount;
			break;
		default:
			return -EINVAL;
			break;
	}

	*(dev->base + chconf + dev->choffs) &= ~(0x3 << MCSPI_CHCONF_FFEW);
	*(dev->base + chconf + dev->choffs) |= fifoState;

	/* assess byte count */
	if (xferReq->wordSize <= 8U) {
		xfer->wordShift = 0U;
	}
	else if (xferReq->wordSize <= 16U) {
		xfer->wordShift = 1U;
	}
	else {
		xfer->wordShift = 2U;
	}
	xferBytes = xferReq->wordCount << xfer->wordShift;

	/* assess fifo xfer levs */
	if (xferBytes <= xfer->txTrig) {
		xfer->txTrig = xferBytes;
	}

	if (xferBytes <= xfer->rxTrig) {
		xfer->rxTrig = xferBytes;
	}

	/* if RX trigger level == FIFO size, decrement RX trigger level
	 * so no TX underrun occurs
	 */
	if (xferReq->xferType != MCSPI_XFER_RX && xfer->rxTrig == xfer->fifoSize) {
		xfer->rxTrig -= 4U;
	}

	reg = (uint32_t)(((xfer->txTrig - 1U) & 0xFFU) << MCSPI_XFERLEVEL_AEL);
	reg |= (uint32_t)(((xfer->rxTrig - 1U) & 0xFFU) << MCSPI_XFERLEVEL_AFL);
	*(dev->base + xferlevel) &= ~65535UL;
	*(dev->base + xferlevel) |= reg;

	/* set the word count */
	*(dev->base + xferlevel) &= ~(65535UL << MCSPI_XFERLEVEL_WCNT);
	*(dev->base + xferlevel) |= (uint32_t)(xferReq->wordCount << MCSPI_XFERLEVEL_WCNT);

	mutexUnlock(dev->devMutex);

	switch (dev->xfer.req->xferType) {
		case MCSPI_XFER_TXRX:
			_mcspi_txrxInit(dev);
			break;
		case MCSPI_XFER_RX:
			_mcspi_rxInit(dev);
			break;
		case MCSPI_XFER_TX:
			_mcspi_txInit(dev);
			break;
		default:
			return -EINVAL;
			break;
	}

	/* enable EOW interrupt */
	*(dev->base + irqenable) |= (1 << MCSPI_IRQENABLE_EOW);

	/* channel enable */
	*(dev->base + chctrl + dev->choffs) |= (1 << MCSPI_CHCTRL_EN);

	/* waiting for cmplt */
	mutexLock(dev->devMutex);
	while (xfer->cmplt == MCSPI_XFER_ONGOING) {
		condWait(dev->devCond, dev->devMutex, 0);
		if (xfer->cmplt == MCSPI_XFER_CMPLT) {
			dev->xferBusy = MCSPI_DEVICE_IDLE;
		}
	}
	mutexUnlock(dev->devMutex);

	return EOK;
}


int mcspi_deinit(uint8_t devnum)
{
	struct mcspi_dev *dev;

	if (devnum > MCU_MCSPI2) {
		return -EINVAL;
	}
	dev = &mcspiDev[devnum];

	mutexLock(dev->devMutex);
	if (dev->xferBusy == MCSPI_DEVICE_BUSY) {
		mutexUnlock(dev->devMutex);
		return -EBUSY;
	}

	mutexUnlock(dev->devMutex);

	dev->initialized = 0;

	(void)condSignal(dev->irqCond);
	(void)threadJoin(-1, 0);

	mcspi_resourceDestroy(dev);

	return EOK;
}


int mcspi_init(uint8_t devnum, uint8_t chnum, uint8_t devtype, struct mcspi_modulctrl *modctrl, struct mcspi_chconf *chconf)
{
	int ret;
	struct mcspi_dev *dev;

	if (((devnum > MCU_MCSPI2)) ||
			((devtype & (MCSPI_DEVICE_SLAVE | MCSPI_DEVICE_MASTER)) == 0U) ||
			((chnum > MCU_MCSPI_CH3))) {
		return -EINVAL;
	}
	dev = &mcspiDev[devnum];
	dev->hwctx = &mcspi_hwctx[devnum];

	if (dev->initialized != 0U) {
		return -EBUSY;
	}
	dev->base = mmap(NULL, PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)dev->hwctx->baseAddr);

	if (dev->base == MAP_FAILED) {
		return -ENOMEM;
	}

	ret = condCreate(&dev->irqCond);
	if (ret != EOK) {
		(void)munmap((void *)dev->base, PAGE_SIZE);
		return ret;
	}

	ret = condCreate(&dev->devCond);
	if (ret != EOK) {
		mcspi_resourceDestroy(dev);
		return ret;
	}

	ret = mutexCreate(&dev->irqMutex);
	if (ret != EOK) {
		mcspi_resourceDestroy(dev);
		return ret;
	}

	ret = mutexCreate(&dev->devMutex);
	if (ret != EOK) {
		mcspi_resourceDestroy(dev);
		return ret;
	}

	/* clear IRQ status reg */
	*(dev->base + irqstatus) = UINT32_MAX;

	dev->stack = malloc((size_t)THREAD_STACK_SIZE);
	if (dev->stack == NULL) {
		mcspi_resourceDestroy(dev);
		return -ENOMEM;
	}

	dev->threadNum = beginthread(mcspi_thread, THREAD_PRIORITY, dev->stack, THREAD_STACK_SIZE, (void *)dev);
	if (dev->threadNum < 0) {
		mcspi_resourceDestroy(dev);
		free(dev->stack);
		return -EIO;
	}

	ret = interrupt(dev->hwctx->irqNum, mcspi_intr, (void *)dev, dev->irqCond, &dev->irqHandle);
	if (ret != EOK) {
		mcspi_resourceDestroy(dev);
		free(dev->stack);
		return ret;
	}

	/* hardware init */
	dev->type = devtype;
	dev->choffs = (uint32_t)(chnum * CHANNEL_STRIDE);

	/* default clocks config: MCU_MCSPI0_ICLK = 166MHz, MCU_MCSPI0_FCLK(SPIREF) = 50MHz */
	mcspi_softReset(dev);

	/* configure MCSPI controller mode */
	(void)memcpy(&dev->mode, modctrl, sizeof(struct mcspi_modulctrl));
	mcspi_modulCtrl(dev);

	/* configure MCSPI_SYSCONFIG */
	mcspi_sysconfig(dev);

	/* configure MCSPI_CHCONFIG */
	mcspi_chconf(dev, chconf);

	dev->initialized = 1;

	return ret;
}
