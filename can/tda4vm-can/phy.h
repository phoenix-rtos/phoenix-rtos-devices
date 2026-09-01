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

#ifndef _CAN_PHY_TDA4VM_
#define _CAN_PHY_TDA4VM_


#include "can-tda4vm.h"


/* MCAN_IR & MCAN_IE bitfields */
#define MCAN_IR_RF0N 0U
#define MCAN_IR_RF0W 1U
#define MCAN_IR_RF0F 2U
#define MCAN_IR_RF0L 3U
#define MCAN_IR_RF1N 4U
#define MCAN_IR_RF1W 5U
#define MCAN_IR_RF1F 6U
#define MCAN_IR_RF1L 7U
#define MCAN_IR_HPM  8U
#define MCAN_IR_TC   9U
#define MCAN_IR_TCF  10U
#define MCAN_IR_TFE  11U
#define MCAN_IR_TEFN 12U
#define MCAN_IR_TEFW 13U
#define MCAN_IR_TEFF 14U
#define MCAN_IR_TEFL 15U
#define MCAN_IR_TSW  16U
#define MCAN_IR_MRAF 17U
#define MCAN_IR_TOO  18U
#define MCAN_IR_DRX  19U
#define MCAN_IR_BEC  20U
#define MCAN_IR_BEU  21U
#define MCAN_IR_ELO  22U
#define MCAN_IR_EP   23U
#define MCAN_IR_EW   24U
#define MCAN_IR_BO   25U
#define MCAN_IR_WDI  26U
#define MCAN_IR_PEA  27U
#define MCAN_IR_PED  28U
#define MCAN_IR_ARA  29U


/* MCAN_CCCR bitfields */
#define MCAN_CCCR_INIT 0U
#define MCAN_CCCR_CCE  1U
#define MCAN_CCCR_MON  5U
#define MCAN_CCCR_FDOE 8U

/*  MCAN_NBTP bitfields */
#define MCAN_NBTP_NTSEG2 0U
#define MCAN_NBTP_NTSEG1 8U
#define MCAN_NBTP_NBRP   16U
#define MCAN_NBTP_NSJW   25U

/* MCAN_TEST bitfields */
#define MCAN_TEST_LPBCK  4U
#define MCAN_TEST_TESTEN 7U

/* MCANSS_CTRL bitfields */
#define MCANSS_CTRL_EXT_EN 6U

/* MCAN_GFC bitfields */
#define MCAN_GFC_RRFE 0U
#define MCAN_GFC_RRFS 1U
#define MCAN_GFC_ANFE 2U
#define MCAN_GFC_ANFS 4U

/* MCAN_SIDFC bitfields */
#define MCAN_SIDFC_FLSSA 2U
#define MCAN_SIDFC_LSS   16U

/* MCAN_XIDFC bitfields */
#define MCAN_XIDFC_FLESA 2U
#define MCAN_XIDFC_LSE   16U

/* MCAN_TXEFC bitfields */
#define MCAN_TXEFC_EFSA 2U
#define MCAN_TXEFC_EFS  16U
#define MCAN_TXEFC_EFWM 24U

/* MCAN_RXF0S bitfields */
#define MCAN_RXF0S_FOFL 0U
#define MCAN_RXF0S_F0GI 8U
#define MCAN_RXF0S_FOPI 16U
#define MCAN_RXF0S_F0F  24U

/* MCAN_RXF0C bitfields */
#define MCAN_RXF0C_F0SA 2U
#define MCAN_RXF0C_F0S  16U
#define MCAN_RXF0C_F0WM 24U
#define MCAN_RXF0C_F0OM 31U

/* MCAN_RXF1C bitfields */
#define MCAN_RXF1C_F1SA 2U
#define MCAN_RXF1C_F1S  16U
#define MCAN_RXF1C_F1WM 24U
#define MCAN_RXF1C_F1OM 31U

/* MCAN_RXESC bitfields */
#define MCAN_RXESC_F0DS 2U
#define MCAN_RXESC_F1DS 4U
#define MCAN_RXESC_RBDS 8U

/* MCAN_RXBC bitfields */
#define MCAN_RXBC_RBSA 2U

/* MCAN_TXBC bitfields */
#define MCAN_TXBC_TBSA 2U
#define MCAN_TXBC_NDTB 16U
#define MCAN_TXBC_TFQS 24U
#define MCAN_TXBC_TFQM 30U

/* MCAN_TXFQS bitfields */
#define MCAN_TXFQS_TFFL  0U
#define MCAN_TXFQS_TFGI  8U
#define MCAN_TXFQS_TFQPI 16U
#define MCAN_TXFQS_TFQF  21U

/* MCAN_TXEFS bitfields */
#define MCAN_TXEFS_EFFL  0U
#define MCAN_TXEFS_EFGI  8U
#define MCAN_TXEFS_EFQPI 16U
#define MCAN_TXEFS_EFF   24U

/* MCAN_ECT bitfields */
#define MCAN_ECR_TEC 0U
#define MCAN_ECR_REC 8U
#define MCAN_ECR_RP  15U
#define MCAN_ECR_CEL 16U


struct mcan_txSession {
	uint8_t txBusy;
	struct mcan_txBuff *data;
	uint32_t msgID;
	uint8_t rtr;
	uint8_t txPutIdx;
	uint32_t xferSize;  /* total frames count */
	uint32_t xferCount; /* frames sent so far */
};


struct mcan_rxSession {
	uint8_t rxBusy;
	struct mcan_rxMessage *data;
	uint32_t xferSize;
	uint32_t xferCount; /* data received so far */
};


#define STDIDX_SFT   30U
#define STDIDX_SFEC  27U
#define STDIDX_SFID1 16U
#define STDIDX_SFID2 0U

#define EXTIDX_EFEC  29U
#define EXTIDX_EFID1 0U
#define EXTIDX_EFT   30U
#define EXTIDX_EFID2 0U


struct mcu_mcan_module {
	uint8_t idx;
	uint8_t canMode;
	uint8_t protocol;
	uint8_t extendedID;
	uint32_t dataBitrate;
	uint8_t filterByID;
	uint32_t lowID;
	uint32_t highID;
	uint8_t txMarker;

	int threadNum;
	uint8_t init;
	void *stack;
	volatile uint32_t *baseSS;
	volatile uint32_t *baseCore;
	volatile uint32_t *baseRAM;
	handle_t moduleLock, irqLock;
	handle_t rxCond, txCond;
	handle_t irqCond;
	handle_t irqHandle0;

	struct mcan_txSession tx;
	struct mcan_rxSession rx;
	uint32_t irqStatus;
};


/* MCANSS registers */
enum {
	pid = 0,
	ctrl,
	stat,
	ics,
	irs,
	iecs,
	iess,
	ies,
	eoi,
	ext_ts_prescaler,
	ext_ts_unserviced_intr_cntr
};

/* MCAN CORE registers */
enum {
	crel = 0,
	endn,
	dbtp = 3,
	test,
	rwd,
	cccr,
	nbtp,
	tscc,
	tscv,
	tocc,
	tocv,
	ecr = 16,
	psr,
	tdcr,
	ir = 20,
	ie,
	ils,
	ile,
	gfc = 32,
	sidfc,
	xidfc,
	xidam = 36,
	hpms,
	ndat1,
	ndat2,
	rxf0c,
	rxf0s,
	rxf0a,
	rxbc,
	rxf1c,
	rxf1s,
	rxf1a,
	rxesc,
	txbc,
	txfqs,
	txesc,
	txbrp,
	txbar,
	txbcr,
	txbto,
	txbcf,
	txbtie,
	txbcie,
	txefc = 60,
	txefs,
	txefa
};


int mcan_phyInit(struct mcu_mcan_module *mcan);


void mcan_enableTxIRQ(struct mcu_mcan_module *mcan);


void mcan_disableTxIRQ(struct mcu_mcan_module *mcan);


void mcan_enableRxIRQ(struct mcu_mcan_module *mcan);


void mcan_disableRxIRQ(struct mcu_mcan_module *mcan);


int mcan_writeRAM(struct mcu_mcan_module *mcan, struct mcan_txMessage *txMsg);


void mcan_readRxFIFO(struct mcu_mcan_module *mcan, uint8_t rxFifoID);


int mcan_readRAMTxEvt(struct mcu_mcan_module *mcan, struct mcan_txEvent *txEvt);

#endif
