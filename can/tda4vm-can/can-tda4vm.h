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

#ifndef _CAN_TDA4VM_
#define _CAN_TDA4VM_

#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <limits.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>


/* user macros */
#define MCU_MCAN_MODULE0 0U
#define MCU_MCAN_MODULE1 1U

#define MCU_MCAN_MODE_NORMAL   0U
#define MCU_MCAN_MODE_MONITOR  1U
#define MCU_MCAN_MODE_LOOPBACK 2U

#define MCAN_MCAN_CC 0U
#define MCAN_MCAN_FD 1U

#define MCU_MCAN_STANDARDID 0U
#define MCU_MCAN_EXTENDEDID 1U

#define MCU_MCAN_FILTERING_OFF 0U
#define MCU_MCAN_FILTERING_ON  1U

#define MCAN_DATA_FIELD_LENGTH 8U


struct mcan_txBuff {
	uint8_t dlc;
	uint8_t payload[MCAN_DATA_FIELD_LENGTH];
};


struct mcan_txReq {
	uint8_t moduleIdx;
	uint32_t msgID;
	uint8_t rtr;              /* remote transmission request: 0 - not RTR, 1 - RTR */
	struct mcan_txBuff *data; /* tx frames payload */
	uint8_t cnt;              /* tx frames count */
};


struct mcan_rxReq {
	uint8_t moduleIdx;
	struct mcan_rxMessage *data; /* array of rx frames */
	uint8_t cnt;                 /* rx frames count */
};


struct mcu_mcanModuleInit {
	uint8_t idx;          /* 0 - MCU_MCAN0, 1 - MCU_MCAN1 */
	uint8_t canMode;      /* 0 - Normal mode, 1 - Bus Monitor, 2 - Loopback */
	uint8_t protocol;     /* 0 - CAN 2.0, 1 - CAN FD */
	uint8_t extendedID;   /* 0 - standard ID, 1 - extended ID */
	uint32_t dataBitrate; /* data phase bitrate in bps */
	uint8_t filterByID;   /* Message filtering by ID: 0 - not applied, 1 - applied */
	uint32_t lowID;       /* lowest accepted Message ID (only if filterById = 1) */
	uint32_t highID;      /* highest accepted Message ID (only if filterById = 1) */
};


struct mcan_txMessage {
	uint32_t dlc : 4; /* Data Length Code */
	uint32_t brs : 1;
	uint32_t fdf : 1;
	uint32_t efc : 2; /* Event FIFO control */
	uint32_t mm : 8;  /* Message Marker */
	uint32_t id : 29; /* Message ID */
	uint32_t rtr : 1; /* Remote Transmission Request */
	uint32_t xtd : 1; /* Extended ID */
	uint32_t esi : 1; /* Error State */
	uint8_t *data;
};
#define TXBUFF_ESI 31U
#define TXBUFF_XTD 30U
#define TXBUFF_RTR 29U
#define TXBUFF_ID  0U

#define TXBUFF_MM  24U
#define TXBUFF_EFC 23U
#define TXBUFF_FDF 21U
#define TXBUFF_BRS 20U
#define TXBUFF_DLC 16U


struct mcan_rxMessage {
	uint32_t rxts : 16; /* RX timestamp */
	uint32_t dlc : 4;   /* Data Length Code */
	uint32_t brs : 1;
	uint32_t fdf : 1;
	uint32_t fidx : 8; /* Acceptance filter index */
	uint32_t anmf : 1; /* Accepted not matching frame */
	uint32_t id : 29;  /* Message ID */
	uint32_t rtr : 1;  /* Remote Transmission Request */
	uint32_t xtd : 1;  /* Extended ID */
	uint32_t esi : 1;  /* Error State */
	uint8_t data[MCAN_DATA_FIELD_LENGTH];
};
#define RXBUFF_ESI 31U
#define RXBUFF_XTD 30U
#define RXBUFF_RTR 29U
#define RXBUFF_ID  0U

#define RXBUFF_ANMF 31U
#define RXBUFF_FIDX 24U
#define RXBUFF_FDF  21U
#define RXBUFF_BRS  20U
#define RXBUFF_DLC  16U
#define RXBUFF_RXTS 0U


struct mcan_txEvent {
	uint32_t txts : 16; /* TX timestamp */
	uint32_t dlc : 4;   /* Data Length Code */
	uint32_t brs : 1;
	uint32_t fdf : 1;
	uint32_t et : 2;  /* Transmission Event */
	uint32_t mm : 8;  /* Message Marker */
	uint32_t id : 29; /* Message ID */
	uint32_t rtr : 1; /* Remote Transmission Request */
	uint32_t xtd : 1; /* Extended ID */
	uint32_t esi : 1; /* Error State */
	/* filled by the user */
	uint8_t idx;    /* MCAN module ID*/
	uint8_t evtCnt; /* how many Tx Events to pop from RAM */
};
#define TXEVT_ESI 31U
#define TXEVT_XTD 30U
#define TXEVT_RTR 29U
#define TXEVT_ID  0U

#define TXEVT_MM   24U
#define TXEVT_EFC  22U
#define TXEVT_FDF  21U
#define TXEVT_BRS  20U
#define TXEVT_DLC  16U
#define TXEVT_TXTS 0U


int mcan_moduleInit(struct mcu_mcanModuleInit *init);


void mcan_moduleDeinit(uint8_t idx);


int mcan_getTxEvt(struct mcan_txEvent *txEvt);


/* returns n.of frames transmitted */
int mcan_txData(struct mcan_txReq *txReq);


/* returns n.of frames received */
int mcan_rxData(struct mcan_rxReq *rxReq);

#endif
