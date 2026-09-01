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

#include <unistd.h>
#include <sys/platform.h>
#include <phoenix/arch/armv7r/tda4vm/tda4vm.h>
#include <phoenix/arch/armv7r/tda4vm/tda4vm_pins.h>

#include "phy.h"


#define BASE_TQ_COUNT      16UL
#define FSYS_CLK           80000000UL
#define TIMESTAMP_PRESC    166666UL
#define TQ_MIN_COUNT       4UL
#define TQ_MAX_COUNT       385UL
#define MAX_PRESCALER      512UL
#define RX_FIFO_REG_STRIDE 4U

/* Memory RAM limits */
#define RAM_STD_ID_FILT_START_ADDR 0UL
#define RAM_STD_ID_ELEM_SIZE       4UL
#define RAM_STD_ID_ELEM_NUM        1UL
#define RAM_STD_ID_FILT_END_ADDR   (uint32_t)(RAM_STD_ID_FILT_START_ADDR + (RAM_STD_ID_ELEM_SIZE * RAM_STD_ID_ELEM_NUM))

#define RAM_EXT_ID_FILT_START_ADDR (RAM_STD_ID_FILT_END_ADDR)
#define RAM_EXT_ID_ELEM_SIZE       8UL
#define RAM_EXT_ID_ELEM_NUM        1UL
#define RAM_EXT_ID_FILT_END_ADDR   (uint32_t)(RAM_EXT_ID_FILT_START_ADDR + (RAM_EXT_ID_ELEM_SIZE * RAM_EXT_ID_ELEM_NUM))

#define RAM_TX_EVENT_FIFO_START_ADDR (RAM_EXT_ID_FILT_END_ADDR)
#define RAM_TX_EVENT_FIFO_ELEM_SIZE  8UL
#define RAM_TX_EVENT_FIFO_ELEM_NUM   32UL
#define RAM_TX_EVENT_FIFO_END_ADDR   (uint32_t)(RAM_TX_EVENT_FIFO_START_ADDR + (RAM_TX_EVENT_FIFO_ELEM_SIZE * RAM_TX_EVENT_FIFO_ELEM_NUM))

#define RAM_RX_ELEM_SIZE         16UL
#define RAM_RX_ELEM_NUM          32UL
#define RAM_RX_FIFO0_START_ADDR  (RAM_TX_EVENT_FIFO_END_ADDR)
#define RAM_RX_FIFO0_END_ADDR    (uint32_t)(RAM_RX_FIFO0_START_ADDR + (RAM_RX_ELEM_SIZE * RAM_RX_ELEM_NUM))
#define RAM_RX_FIFO1_START_ADDR  (RAM_RX_FIFO0_END_ADDR)
#define RAM_RX_FIFO1_END_ADDR    (uint32_t)(RAM_RX_FIFO1_START_ADDR + (RAM_RX_ELEM_SIZE * RAM_RX_ELEM_NUM))
#define RAM_RX_BUFFER_START_ADDR (RAM_RX_FIFO1_END_ADDR)
#define RAM_RX_BUFFER_END_ADDR   (uint32_t)(RAM_RX_BUFFER_START_ADDR + (RAM_RX_ELEM_SIZE * RAM_RX_ELEM_NUM))

#define RAM_TX_QUEUE_START_ADDR (RAM_RX_BUFFER_END_ADDR)
#define RAM_TX_QUEUE_ELEM_SIZE  16UL
#define RAM_TX_QUEUE_ELEM_NUM   32UL
#define RAM_TX_QUEUE_END_ADDR   (uint32_t)(RAM_TX_QUEUE_START_ADDR + (RAM_TX_QUEUE_ELEM_SIZE * RAM_TX_QUEUE_ELEM_NUM))

#define MCAN_TX_IRQS (uint32_t)((1UL << MCAN_IR_TC) | (1UL << MCAN_IR_TCF) | (1UL << MCAN_IR_TFE) | \
		(1UL << MCAN_IR_TEFW) | (1UL << MCAN_IR_TEFF) | \
		(1UL << MCAN_IR_TEFL) | (1UL << MCAN_IR_TSW))

#define MCAN_RX_IRQS (uint32_t)((1UL << MCAN_IR_RF0N) | (1UL << MCAN_IR_RF0W) | (1UL << MCAN_IR_RF0F) | \
		(1UL << MCAN_IR_RF0L) | (1UL << MCAN_IR_RF1N) | (1UL << MCAN_IR_RF1W) | \
		(1UL << MCAN_IR_RF1F) | (1UL << MCAN_IR_RF1L))


static platformctl_t pinConfig[] = {
	{ .action = pctl_set, .type = pctl_pinconfig, .pin_config.pin_num = pin_mcu_mcan0_tx, .pin_config.debounce_idx = 0U, .pin_config.mux = 0U, .pin_config.flags = (TDA4VM_GPIO_PULL_DISABLE & ~(TDA4VM_GPIO_TX_DIS)) },
	{ .action = pctl_set, .type = pctl_pinconfig, .pin_config.pin_num = pin_mcu_mcan0_rx, .pin_config.debounce_idx = 0U, .pin_config.mux = 0U, .pin_config.flags = (TDA4VM_GPIO_PULL_DISABLE | TDA4VM_GPIO_RX_EN) },
};


int mcan_writeRAM(struct mcu_mcan_module *mcan, struct mcan_txMessage *txMsg)
{
	uint8_t txPutId, txFull;
	uint32_t startAddr, offset;
	uint32_t val;
	uint8_t count32b, cnt;

	val = *(mcan->baseCore + txfqs);
	txPutId = (val >> MCAN_TXFQS_TFQPI) & 0x1FU;
	txFull = (val >> MCAN_TXFQS_TFQF) & 0x1U;

	if (txFull == 1U || (txPutId > RAM_TX_QUEUE_ELEM_SIZE)) {
		return -EBUSY;
	}
	mcan->tx.txPutIdx = txPutId;

	/* calculate Tx RAM address */
	startAddr = (*(mcan->baseCore + txbc) >> MCAN_TXBC_TBSA) & 0x3FFFUL;
	startAddr = startAddr << 2U;

	offset = (uint32_t)(RAM_TX_QUEUE_ELEM_SIZE * txPutId);
	offset += (startAddr);

	/* submit Tx buffer */
	val = 0;
	val = (uint32_t)(txMsg->esi << TXBUFF_ESI) | (uint32_t)(txMsg->xtd << TXBUFF_XTD) |
			(uint32_t)(txMsg->rtr << TXBUFF_RTR) | (uint32_t)(txMsg->id << TXBUFF_ID);
	*(mcan->baseRAM + (offset >> 2U)) = val;
	offset += 4U;

	val = 0;
	val = (uint32_t)(txMsg->mm << TXBUFF_MM) | (uint32_t)(txMsg->efc << TXBUFF_EFC) |
			(uint32_t)(txMsg->fdf << TXBUFF_FDF) | (uint32_t)(txMsg->brs << TXBUFF_BRS) |
			(uint32_t)(txMsg->dlc << TXBUFF_DLC);
	*(mcan->baseRAM + (offset >> 2U)) = val;
	offset += 4U;

	/* TODO: add CAN FD data transmission */
	count32b = (((uint8_t)txMsg->dlc) >> 2U);
	for (cnt = 0U; cnt < count32b; cnt++) {
		val = 0;
		val = (uint32_t)txMsg->data[0U] | ((uint32_t)txMsg->data[1U] << 8U) |
				((uint32_t)txMsg->data[2U] << 16U) | ((uint32_t)txMsg->data[3U] << 24U);

		*(mcan->baseRAM + (offset >> 2U)) = val;
		offset += 4U;
		txMsg->data += 4U;
		txMsg->dlc -= 4U;
	}

	val = 0;
	for (cnt = 0U; cnt < (uint8_t)txMsg->dlc; cnt++) {
		val |= ((uint32_t)txMsg->data[cnt] << (cnt * 8U));
		;
		txMsg->data += 1U;
	}
	*(mcan->baseRAM + (offset >> 2U)) = val;

	mcan->tx.xferCount += 1U;
	mcan->tx.data += 1U;

	return EOK;
}


int mcan_readRAMTxEvt(struct mcu_mcan_module *mcan, struct mcan_txEvent *txEvt)
{
	uint8_t txFill, txGetId, txFull;
	uint32_t txfifoState;
	uint32_t startAddr, offset;
	uint32_t val;

	/* get fifo status */
	txfifoState = *(mcan->baseCore + txefs);
	txFill = (txfifoState >> MCAN_TXEFS_EFFL) & 0x3FU;
	txGetId = (txfifoState >> MCAN_TXEFS_EFGI) & 0x1FU;
	txFull = (txfifoState >> MCAN_TXEFS_EFF) & 0x1U;

	if (txFill == 0U) {
		return -ENOENT;
	}
	if (txFull == 1U) {
		fprintf(stderr, "[MCAN] TX Event FIFO full\n");
	}

	/* read tx event from RAM */
	startAddr = (*(mcan->baseCore + txefc) >> MCAN_TXEFC_EFSA) & 0x3FFFUL;
	startAddr = startAddr << 2U;
	offset = (uint32_t)(RAM_TX_EVENT_FIFO_ELEM_SIZE * txGetId);
	offset += (startAddr);

	val = *(mcan->baseRAM + (offset >> 2U));
	txEvt->esi = (val >> TXEVT_ESI) & 0x1U;
	txEvt->xtd = (val >> TXEVT_XTD) & 0x1U;
	txEvt->rtr = (val >> TXEVT_RTR) & 0x1U;
	txEvt->id = val & 0x1FFFFFFFUL;
	if (txEvt->xtd == MCU_MCAN_STANDARDID) {
		txEvt->id >>= 18U;
	}

	offset += 4U;
	val = *(mcan->baseRAM + (offset >> 2U));
	txEvt->mm = (val >> TXEVT_MM) & 0xFFU;
	txEvt->et = (val >> TXEVT_EFC) & 0x3U;
	txEvt->fdf = (val >> TXEVT_FDF) & 0x1U;
	txEvt->brs = (val >> TXEVT_BRS) & 0x1U;
	txEvt->dlc = (val >> TXEVT_DLC) & 0xFU;
	txEvt->txts = val & 0xFFFFUL;

	/* write ack to tx evt fifo */
	*(mcan->baseCore + txefa) = (uint32_t)txGetId;

	return EOK;
}


void mcan_readRxFIFO(struct mcu_mcan_module *mcan, uint8_t rxFifoID)
{
	uint8_t fillLvl, fifoFull;
	uint8_t rxGetIdx, rxPutIdx;
	uint32_t val;
	uint32_t startAddr, offset;
	uint8_t cnt, count32b;
	uint8_t regStride = rxFifoID * RX_FIFO_REG_STRIDE;
	struct mcan_rxMessage *rxFrame;
	uint8_t tail;

	/* iterate through all requested frames */
	while ((mcan->rx.xferSize - mcan->rx.xferCount) > 0UL) {
		rxFrame = mcan->rx.data;
		val = *(mcan->baseCore + rxf0s + regStride);
		fillLvl = (val & 0x3FU);
		rxGetIdx = (val >> MCAN_RXF0S_F0GI) & 0x3FU;
		rxPutIdx = (val >> MCAN_RXF0S_FOPI) & 0x3FU;
		fifoFull = (val >> MCAN_RXF0S_F0F) & 0x1U;

		if (fillLvl == 0U) {
			return;
		}
		if (fifoFull == 1U) {
			/* In overwrite mode, skip two elements to avoid
			 * reading an element potentially being written by MCAN.
			 */
			rxGetIdx = (rxPutIdx + 2U) & (RAM_RX_ELEM_NUM - 1U);
		}

		/* calculate RX RAM address */
		startAddr = (*(mcan->baseCore + rxf0c + regStride) >> MCAN_RXF0C_F0SA) & 0x3FFFUL;
		startAddr = startAddr << 2U;

		offset = (uint32_t)(RAM_RX_ELEM_SIZE * rxGetIdx);
		offset += (startAddr);

		val = *(mcan->baseRAM + (offset >> 2U));
		rxFrame->esi = (val >> RXBUFF_ESI) & 0x1U;
		rxFrame->xtd = (val >> RXBUFF_XTD) & 0x1U;
		rxFrame->rtr = (val >> RXBUFF_RTR) & 0x1U;
		rxFrame->id = val & 0x1FFFFFFFUL;
		if (rxFrame->xtd == MCU_MCAN_STANDARDID) {
			rxFrame->id >>= 18U;
		}

		offset += 4U;
		val = *(mcan->baseRAM + (offset >> 2U));
		rxFrame->anmf = (val >> RXBUFF_ANMF) & 0x1U;
		rxFrame->fidx = (val >> RXBUFF_FIDX) & 0x7FU;
		rxFrame->fdf = (val >> RXBUFF_FDF) & 0x1U;
		rxFrame->brs = (val >> RXBUFF_BRS) & 0x1U;
		rxFrame->dlc = (val >> RXBUFF_DLC) & 0xFU;
		rxFrame->rxts = val & 0xFFFFUL;

		offset += 4U;
		/* TODO: add CAN FD frames reception (dlc is not literal)*/
		count32b = (((uint8_t)rxFrame->dlc) >> 2U);
		for (cnt = 0U; cnt < count32b; cnt++) {
			val = *(mcan->baseRAM + (offset >> 2U));
			rxFrame->data[cnt * 4U] = (uint8_t)(val & 0xFFU);
			rxFrame->data[(cnt * 4U) + 1U] = (uint8_t)((val >> 8U) & 0xFFU);
			rxFrame->data[(cnt * 4U) + 2U] = (uint8_t)((val >> 16U) & 0xFFU);
			rxFrame->data[(cnt * 4U) + 3U] = (uint8_t)((val >> 24U) & 0xFFU);

			offset += 4U;
		}

		tail = rxFrame->dlc - (cnt * 4U);
		if (tail != 0U) {
			val = *(mcan->baseRAM + (offset >> 2U));
			for (cnt = 0U; cnt < tail; cnt++) {
				rxFrame->data[(count32b * 4U) + cnt] = (uint8_t)((val >> (8U * cnt)) & 0xFFU);
			}
		}
		mcan->rx.xferCount += 1U;
		mcan->rx.data += 1U;

		/* write RX FIFO ack */
		*(mcan->baseCore + rxf0a + regStride) = (uint32_t)rxGetIdx;
	}
}


void mcan_enableTxIRQ(struct mcu_mcan_module *mcan)
{
	int ret;
	uint32_t cnt;

	/* enable MCAN_IE */
	*(mcan->baseCore + ie) |= MCAN_TX_IRQS;
	/* enable interrupt lines */
	ret = *(mcan->baseCore + ils);
	ret &= ~(MCAN_TX_IRQS);
	*(mcan->baseCore + ils) |= ret;
	/* enable tx buffer interrupts */
	ret = 0U;
	for (cnt = 0UL; cnt < RAM_TX_QUEUE_ELEM_NUM; cnt++) {
		ret |= (1UL << cnt);
	}
	*(mcan->baseCore + txbtie) |= ret;
}


void mcan_disableTxIRQ(struct mcu_mcan_module *mcan)
{
	int ret;
	uint32_t cnt;

	/* disable MCAN_IE */
	*(mcan->baseCore + ie) &= ~(MCAN_TX_IRQS);
	/* disable tx buffer interrupts */
	ret = 0U;
	for (cnt = 0UL; cnt < RAM_TX_QUEUE_ELEM_NUM; cnt++) {
		ret |= (1UL << cnt);
	}
	*(mcan->baseCore + txbtie) &= ~ret;
}


void mcan_enableRxIRQ(struct mcu_mcan_module *mcan)
{
	int ret;
	/* enable MCAN_IE */
	*(mcan->baseCore + ie) |= MCAN_RX_IRQS;
	/* enable interrupt lines */
	ret = *(mcan->baseCore + ils);
	ret &= ~(MCAN_RX_IRQS);
	*(mcan->baseCore + ils) |= ret;
}


void mcan_disableRxIRQ(struct mcu_mcan_module *mcan)
{
	/* disable MCAN_IE */
	*(mcan->baseCore + ie) &= ~MCAN_RX_IRQS;
}


static void mcan_setMemoryRAM(struct mcu_mcan_module *mcan)
{
	if (mcan->filterByID == MCU_MCAN_FILTERING_ON) {
		/* standard ID filter list */
		if (mcan->extendedID == MCU_MCAN_STANDARDID) {
			*(mcan->baseCore + sidfc) |= ((RAM_STD_ID_ELEM_NUM << MCAN_SIDFC_LSS) | ((RAM_STD_ID_FILT_START_ADDR >> 2U) << MCAN_SIDFC_FLSSA));
		}
		/* extended ID filter list */
		else if (mcan->extendedID == MCU_MCAN_EXTENDEDID) {
			*(mcan->baseCore + xidfc) |= ((RAM_EXT_ID_ELEM_NUM << MCAN_XIDFC_LSE) | ((RAM_EXT_ID_FILT_START_ADDR >> 2U) << MCAN_XIDFC_FLESA));
		}
	}

	/* Tx event FIFO */
	*(mcan->baseCore + txefc) |= ((RAM_TX_EVENT_FIFO_ELEM_NUM << MCAN_TXEFC_EFS) | ((RAM_TX_EVENT_FIFO_START_ADDR >> 2U) << MCAN_TXEFC_EFSA));
	/* Rx FIFO_0 */
	*(mcan->baseCore + rxf0c) |= ((RAM_RX_ELEM_SIZE << MCAN_RXF0C_F0S) | (((RAM_RX_FIFO0_START_ADDR) >> 2U) << MCAN_RXF0C_F0SA));
	/* Rx FIFO_1 */
	*(mcan->baseCore + rxf1c) |= ((RAM_RX_ELEM_SIZE << MCAN_RXF1C_F1S) | (((RAM_RX_FIFO1_START_ADDR) >> 2U) << MCAN_RXF1C_F1SA));
	/* Rx Buffer */
	*(mcan->baseCore + rxbc) |= ((RAM_RX_BUFFER_START_ADDR >> 2U) << MCAN_RXBC_RBSA);
	/* Tx Queue */
	*(mcan->baseCore + txbc) |= ((0x1UL << MCAN_TXBC_TFQM) | (RAM_TX_QUEUE_ELEM_NUM << MCAN_TXBC_TFQS) | ((RAM_TX_QUEUE_START_ADDR >> 2U) << MCAN_TXBC_TBSA));

	/* TODO: set MCAN_RXESC if CAN FD data lengths needed */

	/* TODO: set MCAN_TXESC if CAN FD data lengths needed */
}


static int incrementPresc(uint32_t base, uint16_t *scaler)
{
	uint32_t temp = (uint32_t)(*scaler) * base;

	while ((temp < FSYS_CLK) && (*scaler < MAX_PRESCALER)) {
		*scaler = (uint16_t)(*scaler + 1U);
		temp = (uint32_t)(*scaler) * base;
	}

	return (temp == FSYS_CLK) ? 0 : -1;
}

/* calculations made on bitrates instead of time quantas to explicitly avoid divisions.
 * for basic bitrates (eg. 125kb/s, 250kb/s, 500kb/s...) calculation is ready in first incrementPresc() call.
 */
static int mcan_calcBitTiming(uint32_t dataBitrate, uint16_t *clkPrescale, uint16_t *tqCount)
{
	int ret;
	uint16_t cnt = BASE_TQ_COUNT;
	uint16_t scaler = 1U;
	uint8_t decrement = 0U;
	uint32_t candidate;

	if (dataBitrate > (FSYS_CLK / 4U)) {
		return -1;
	}

	candidate = dataBitrate * (uint32_t)cnt;
	while ((cnt >= TQ_MIN_COUNT) && (cnt <= TQ_MAX_COUNT)) {
		if ((candidate > (FSYS_CLK / 2U)) && decrement == 0U) {
			decrement = 1U;
			cnt = (uint16_t)(BASE_TQ_COUNT - 1U);
		}
		candidate = dataBitrate * (uint32_t)cnt;

		ret = incrementPresc(candidate, &scaler);
		if (ret == 0) {
			*tqCount = cnt;
			*clkPrescale = scaler;
			return 0;
		}
		cnt = (decrement == 1U) ? (cnt - 1U) : (cnt + 1U);
		scaler = 1U;
	}

	return -1;
}


static void mcan_setBitTimingReg(struct mcu_mcan_module *mcan, uint16_t clkPrescaler, uint16_t tqCount)
{
	/* SP must be between 75% and 87.5% of bit time.
	 * Integer division / 4 ensures SP >= 75%.
	 */
	uint16_t ntseg1, ntseg2;

	ntseg2 = tqCount / 4U;
	ntseg1 = tqCount - ntseg2 - 1; /* 1 bit for SYNC_SEG */

	*(mcan->baseCore + nbtp) &= ~((0x7FUL << MCAN_NBTP_NSJW) | (0x1FFUL << MCAN_NBTP_NBRP) | (0xFFUL << MCAN_NBTP_NTSEG1) | (0x7FUL << MCAN_NBTP_NTSEG2));
	*(mcan->baseCore + nbtp) |= (((ntseg2 - 1UL) << MCAN_NBTP_NSJW) | ((clkPrescaler - 1UL) << MCAN_NBTP_NBRP) | ((ntseg1 - 1UL) << MCAN_NBTP_NTSEG1) | ((ntseg2 - 1UL) << MCAN_NBTP_NTSEG2));
}


static void mcan_setMessageFilter(struct mcu_mcan_module *mcan)
{
	uint32_t offset, val = 0;

	if (mcan->filterByID == MCU_MCAN_FILTERING_ON) {
		if (mcan->extendedID == MCU_MCAN_STANDARDID) {
			offset = (*(mcan->baseCore + sidfc) >> MCAN_SIDFC_FLSSA) & (0x3FFFUL);
			val |= ((1UL << STDIDX_SFEC) | (mcan->lowID << STDIDX_SFID1) | (mcan->highID << STDIDX_SFID2));
			*(mcan->baseRAM + offset) = val;
		}
		/* TODO: add extended filter if required */
	}
}


int mcan_phyInit(struct mcu_mcan_module *mcan)
{
	int ret, cnt;
	uint16_t clkPrescaler, tqCount;

	/* default clock config: MCU_MCAN_ICLK = 166MHz, MCU_MCAN_FCLK = 80MHz */

	/* set initialization mode */
	*(mcan->baseCore + cccr) |= (1UL << MCAN_CCCR_INIT);
	while ((*(mcan->baseCore + cccr) & ((1UL << MCAN_CCCR_INIT))) != 1U) {
		/* Busy wait */
	}
	*(mcan->baseCore + cccr) |= (1UL << MCAN_CCCR_CCE);
	*(mcan->baseCore + cccr) |= (mcan->protocol << MCAN_CCCR_FDOE);

	/* configure bit timing */
	ret = mcan_calcBitTiming(mcan->dataBitrate, &clkPrescaler, &tqCount);
	if (ret < 0) {
		fprintf(stderr, "[CAN]: given bitrate can not be configured...\n");
		return -EINVAL;
	}
	mcan_setBitTimingReg(mcan, clkPrescaler, tqCount);

	/* set device mode */
	if (mcan->canMode != MCU_MCAN_MODE_NORMAL) {
		*(mcan->baseCore + cccr) |= (1UL << MCAN_CCCR_MON);
	}
	if (mcan->canMode == MCU_MCAN_MODE_LOOPBACK) {
		*(mcan->baseCore + cccr) |= (1UL << MCAN_TEST_TESTEN);
		*(mcan->baseCore + test) |= (1UL << MCAN_TEST_LPBCK);
	}

	/* set External Timestamp */
	*(mcan->baseCore + tscc) &= ~(0x3UL);
	*(mcan->baseCore + tscc) |= 0x2UL;

	/* pre-determined prescaler value (timestamps every 1ms), value for ICLK=166MHz */
	*(mcan->baseSS + ext_ts_prescaler) &= ~(0xFFFFFFUL);
	*(mcan->baseSS + ext_ts_prescaler) |= TIMESTAMP_PRESC;

	*(mcan->baseSS + ctrl) |= (1UL << MCANSS_CTRL_EXT_EN);

	/* configure global filter settings */
	*(mcan->baseCore + gfc) |= (1UL << MCAN_GFC_ANFE) | (1UL << MCAN_GFC_ANFS);

	/* Memory RAM config */
	mcan_setMemoryRAM(mcan);

	/* set RXFIFOs to overwrite mode */
	*(mcan->baseCore + rxf0c) |= (1UL << MCAN_RXF0C_F0OM);
	*(mcan->baseCore + rxf1c) |= (1UL << MCAN_RXF1C_F1OM);

	/* MessageID filters */
	mcan_setMessageFilter(mcan);

	/* enable interrupt line 0*/
	*(mcan->baseCore + ile) |= 1U;

	/* leave initialization mode */
	*(mcan->baseCore + cccr) &= ~(1UL << MCAN_CCCR_CCE);
	*(mcan->baseCore + cccr) &= ~(1UL << MCAN_CCCR_INIT);
	while ((*(mcan->baseCore + cccr) & ((1UL << MCAN_CCCR_INIT))) != 0U) {
		/* Busy wait */
	}

	/* TODO: add MCAN1 GPIO config if needed */
	if (mcan->canMode != MCU_MCAN_MODE_LOOPBACK) {
		for (cnt = 0U; cnt < 2U; cnt++) {
			ret = platformctl((void *)&pinConfig[cnt]);
			if (ret != EOK) {
				fprintf(stderr, "MCAN pin %d configuration failed\n", cnt);
				return ret;
			}
		}
	}

	return EOK;
}
