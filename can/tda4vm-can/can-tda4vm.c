/*
 * Phoenix-RTOS
 *
 * TDA4VM MCU CAN 2.0 driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Rafal Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/interrupt.h>
#include <unistd.h>

#include "can-tda4vm.h"
#include "phy.h"


#define DEFAULT_TASK_PRIORITY 4U
#define DEFAULT_TASK_STACK    512UL

#define MCU_MCAN0_SS        (0x40520000)
#define MCU_MCAN1_SS        (0x40560000)
#define MCU_MCAN0_CFG       (0x40528000)
#define MCU_MCAN1_CFG       (0x40568000)
#define MCU_MCAN0_RAM       (0x40500000)
#define MCU_MCAN1_RAM       (0x40540000)
#define MCU_MCAN_REG_STRIDE (0x40000)

#define MCU_MCAN0_MCANSS_MCAN_LVL_INT_0            0U
#define MCU_MCAN0_MCANSS_MCAN_LVL_INT_1            1U
#define MCU_MCAN1_MCANSS_MCAN_LVL_INT_0            2U
#define MCU_MCAN1_MCANSS_MCAN_LVL_INT_1            3U
#define MCU_MCAN0_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0 4U
#define MCU_MCAN1_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0 5U

#define MCU_MCAN_MODULE0 0U
#define MCU_MCAN_MODULE1 1U

#define CAN_MAX_BITRATE 1000000UL


static struct mcu_mcan_module gMcan[2];


static void mcan_freeResources(uint8_t idx)
{
	struct mcu_mcan_module *mcan = &gMcan[idx];

	if (mcan->threadNum > 0) {
		mutexLock(mcan->moduleLock);
		mcan->init = 0U;

		condSignal(mcan->irqCond);

		(void)threadJoin(-1, 0);

		mutexUnlock(mcan->moduleLock);
	}

	resourceDestroy(mcan->moduleLock);
	resourceDestroy(mcan->irqLock);
	resourceDestroy(mcan->rxCond);
	resourceDestroy(mcan->txCond);
	resourceDestroy(mcan->irqHandle0);

	if (mcan->stack != NULL) {
		free(mcan->stack);
	}
	if (mcan->baseCore != MAP_FAILED) {
		(void)munmap((void *)mcan->baseCore, PAGE_SIZE);
	}
	if (mcan->baseSS != MAP_FAILED) {
		(void)munmap((void *)mcan->baseSS, PAGE_SIZE);
	}
	if (mcan->baseRAM != MAP_FAILED) {
		(void)munmap((void *)mcan->baseRAM, PAGE_SIZE * 8U);
	}
}


static int mcan_irqHandler(unsigned int intr, void *data)
{
	struct mcu_mcan_module *mcan = (struct mcu_mcan_module *)data;
	uint32_t irqStatus = *(mcan->baseCore + ir);
	uint32_t irqMask = *(mcan->baseCore + ie);
	int ret = -1;

	irqStatus &= irqMask;
	*(mcan->baseCore + ir) = irqStatus;
	mcan->irqStatus |= irqStatus;

	if (((irqStatus >> MCAN_IR_TC) & 1U) == 1U) {
		ret = 0;
	}

	if (((irqStatus >> MCAN_IR_RF0N) & 1U) == 1U) {
		mcan_readRxFIFO(mcan, 0U);
		ret = 0;
	}

	if (((irqStatus >> MCAN_IR_RF1N) & 1U) == 1U) {
		if (mcan->filterByID == MCU_MCAN_FILTERING_OFF) {
			mcan_readRxFIFO(mcan, 1U);
		}
		ret = 0;
	}

	/* TODO: handle timestamp rollover */
	return ret;
}


static void mcan_irqThread(void *arg)
{
	struct mcu_mcan_module *mcan = (struct mcu_mcan_module *)arg;

	mutexLock(mcan->irqLock);
	while (mcan->init == 1U) {
		condWait(mcan->irqCond, mcan->irqLock, 0);

		if (((mcan->irqStatus >> MCAN_IR_TC) & 1U) == 1U) {
			mcan->irqStatus &= ~(1UL << MCAN_IR_TC);
			condSignal(mcan->txCond);
		}

		if (((mcan->irqStatus >> MCAN_IR_RF0N) & 1U) == 1U) {
			mcan->irqStatus &= ~(1UL << MCAN_IR_RF0N);
			mutexLock(mcan->moduleLock);
			if (mcan->rx.rxBusy == 1U) {
				if (mcan->rx.xferCount == mcan->rx.xferSize) {
					condSignal(mcan->rxCond);
				}
			}
			mutexUnlock(mcan->moduleLock);
		}

		if (((mcan->irqStatus >> MCAN_IR_RF1N) & 1U) == 1U) {
			mcan->irqStatus &= ~(1UL << MCAN_IR_RF1N);
			mutexLock(mcan->moduleLock);
			if (mcan->rx.rxBusy == 1U) {
				if (mcan->rx.xferCount == mcan->rx.xferSize) {
					condSignal(mcan->rxCond);
				}
			}
			mutexUnlock(mcan->moduleLock);
		}
	}

	mutexUnlock(mcan->irqLock);

	endthread();
}


static void mcan_parseTxMsg(struct mcan_txMessage *txMsg, struct mcu_mcan_module *mcan)
{
	txMsg->dlc = mcan->tx.data->dlc;
	if (txMsg->dlc > MCAN_DATA_FIELD_LENGTH) {
		txMsg->dlc = MCAN_DATA_FIELD_LENGTH;
	}
	txMsg->brs = 0U; /* no bit rate switching */
	txMsg->fdf = mcan->protocol;
	txMsg->efc = 1U; /* store TX events */
	txMsg->mm = mcan->txMarker++;
	if (mcan->extendedID == MCU_MCAN_EXTENDEDID) {
		txMsg->id = mcan->tx.msgID;
	}
	else {
		txMsg->id = (mcan->tx.msgID << 18U);
	}
	txMsg->rtr = mcan->tx.rtr;
	txMsg->xtd = 0U; /* TODO: add extended identifier */
	txMsg->esi = 0U;
	txMsg->data = mcan->tx.data->payload;
}


int mcan_getTxEvt(struct mcan_txEvent *txEvt)
{
	struct mcu_mcan_module *mcan = &gMcan[txEvt->idx];

	return mcan_readRAMTxEvt(mcan, txEvt);
}


int mcan_txData(struct mcan_txReq *txReq)
{
	int ret;
	struct mcan_txMessage txMsg;
	struct mcu_mcan_module *mcan = &gMcan[txReq->moduleIdx];
	uint8_t errCnt = (uint8_t)(*(mcan->baseCore + ecr) & 0xFFU);

	mutexLock(mcan->moduleLock);
	if (mcan->tx.txBusy == 1U) {
		mutexUnlock(mcan->moduleLock);
		return -EBUSY;
	}
	mcan->tx.txBusy = 1U;
	mcan->tx.data = txReq->data;
	mcan->tx.xferCount = 0;
	mcan->tx.xferSize = txReq->cnt;
	mcan->tx.msgID = txReq->msgID;
	mcan->tx.rtr = txReq->rtr;

	/* enable interrupts */
	mcan_enableTxIRQ(mcan);

	while (mcan->tx.txBusy == 1U) {
		mcan_parseTxMsg(&txMsg, mcan);

		/* send msg to RAM */
		ret = mcan_writeRAM(mcan, &txMsg);
		if (ret != EOK) {
			mutexUnlock(mcan->moduleLock);
			mcan->tx.txBusy = 0U;
			return ret;
		}

		/* submit Tx request */
		*(mcan->baseCore + txbar) |= (1UL << mcan->tx.txPutIdx);

		condWait(mcan->txCond, mcan->moduleLock, 0);
		if (mcan->tx.xferCount == mcan->tx.xferSize) {
			mcan->tx.txBusy = 0U;
		}
	}
	mutexUnlock(mcan->moduleLock);

	/* disable interrupts */
	mcan_disableTxIRQ(mcan);

	/* check protocol reg for errors */
	if ((uint8_t)(*(mcan->baseCore + ecr) & 0xFFU) != errCnt) {
		return -EIO;
	}

	return mcan->tx.xferCount;
}


int mcan_rxData(struct mcan_rxReq *rxReq)
{
	struct mcu_mcan_module *mcan = &gMcan[rxReq->moduleIdx];
	uint8_t errCnt = (uint8_t)((*(mcan->baseCore + ecr) >> MCAN_ECR_REC) & 0x3FU);

	mutexLock(mcan->moduleLock);
	if (mcan->rx.rxBusy == 1U) {
		mutexUnlock(mcan->moduleLock);
		return -EBUSY;
	}

	mcan->rx.rxBusy = 1U;
	mcan->rx.data = rxReq->data;
	mcan->rx.xferSize = rxReq->cnt;
	mcan->rx.xferCount = 0;
	/* enable interrupts */
	mcan_enableRxIRQ(mcan);

	while (mcan->rx.rxBusy == 1U) {
		condWait(mcan->rxCond, mcan->moduleLock, 0);

		if (mcan->rx.xferSize == mcan->rx.xferCount) {
			mcan->rx.rxBusy = 0U;
		}
	}
	mutexUnlock(mcan->moduleLock);

	/* disable interrupts */
	mcan_disableRxIRQ(mcan);

	/* check protocol errors */
	if ((uint8_t)((*(mcan->baseCore + ecr) >> MCAN_ECR_REC) & 0x3FU) != errCnt) {
		return -EIO;
	}

	return mcan->rx.xferCount;
}


static int mcan_moduleInitRsrc(uint8_t idx)
{
	int ret;
	uint32_t reg;
	struct mcu_mcan_module *mcan = &gMcan[idx];

	memset(mcan, 0, sizeof(struct mcu_mcan_module));
	mcan->baseCore = MAP_FAILED;
	mcan->baseSS = MAP_FAILED;
	mcan->baseRAM = MAP_FAILED;
	mcan->stack = NULL;

	mcan->init = 1U;

	ret = mutexCreate(&mcan->moduleLock);
	if (ret != EOK) {
		return ret;
	}

	ret = mutexCreate(&mcan->irqLock);
	if (ret != EOK) {
		return ret;
	}

	ret = condCreate(&mcan->txCond);
	if (ret != EOK) {
		return ret;
	}

	ret = condCreate(&mcan->rxCond);
	if (ret != EOK) {
		return ret;
	}

	ret = condCreate(&mcan->irqCond);
	if (ret != EOK) {
		return ret;
	}

	reg = (uint32_t)(MCU_MCAN0_CFG + (idx * MCU_MCAN_REG_STRIDE));
	mcan->baseCore = mmap(NULL, PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)reg);
	if (mcan->baseCore == MAP_FAILED) {
		return -ENOMEM;
	}

	reg = (uint32_t)(MCU_MCAN0_SS + (idx * MCU_MCAN_REG_STRIDE));
	mcan->baseSS = mmap(NULL, PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)reg);
	if (mcan->baseSS == MAP_FAILED) {
		return -ENOMEM;
	}

	reg = (uint32_t)(MCU_MCAN0_RAM + (idx * MCU_MCAN_REG_STRIDE));
	mcan->baseRAM = mmap(NULL, PAGE_SIZE * 8U, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)reg);
	if (mcan->baseRAM == MAP_FAILED) {
		return -ENOMEM;
	}

	mcan->stack = malloc((size_t)DEFAULT_TASK_STACK);
	if (mcan->stack == NULL) {
		return -ENOMEM;
	}

	mcan->threadNum = beginthread(mcan_irqThread, DEFAULT_TASK_PRIORITY, mcan->stack, DEFAULT_TASK_STACK, (void *)mcan);
	if (mcan->threadNum < 0) {
		return -ENOMEM;
	}

	ret = interrupt(MCU_MCAN0_MCANSS_MCAN_LVL_INT_0 + idx, mcan_irqHandler, (void *)mcan, mcan->irqCond, &mcan->irqHandle0);
	if (ret != 0) {
		return ret;
	}

	return EOK;
}


void mcan_moduleDeinit(uint8_t idx)
{
	mcan_freeResources(idx);
}


int mcan_moduleInit(struct mcu_mcanModuleInit *init)
{
	struct mcu_mcan_module *mcan;
	int ret;

	if ((init->idx != MCU_MCAN_MODULE0) && (init->idx != MCU_MCAN_MODULE1)) {
		return -EINVAL;
	}
	mcan = &gMcan[init->idx];

	if (mcan->init == 1U) {
		return -EBUSY;
	}

	if ((init->canMode < MCU_MCAN_MODE_NORMAL) || (init->canMode > MCU_MCAN_MODE_LOOPBACK)) {
		return -EINVAL;
	}

	if ((init->protocol != MCAN_MCAN_CC) && (init->protocol != MCAN_MCAN_FD)) {
		return -EINVAL;
	}

	if (init->dataBitrate > CAN_MAX_BITRATE) {
		return -EINVAL;
	}

	ret = mcan_moduleInitRsrc(init->idx);
	if (ret != EOK) {
		mcan_freeResources(init->idx);
		return ret;
	}
	mcan->canMode = init->canMode;
	mcan->protocol = init->protocol;
	mcan->dataBitrate = init->dataBitrate;
	mcan->lowID = init->lowID;
	mcan->highID = init->highID;
	mcan->idx = init->idx;
	mcan->extendedID = init->extendedID;
	mcan->filterByID = init->filterByID;

	ret = mcan_phyInit(mcan);
	if (ret != EOK) {
		return -EIO;
	}

	return EOK;
}
