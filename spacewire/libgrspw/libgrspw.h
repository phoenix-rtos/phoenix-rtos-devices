/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2023, 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBGRSPW_H_
#define LIBGRSPW_H_


#include <board_config.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <string.h>
#include <stdbool.h>


#define SPW_RX_MIN_BUFSZ 4
#define SPW_TX_MIN_BUFSZ 8

/* DMA configuration */
#define SPW_DMA_CFG_LE  (1U << 16) /* Disable TX when link error occurs */
#define SPW_DMA_CFG_SP  (1U << 15) /* Remove 2nd (and 1st) byte (protocol id) of each packet */
#define SPW_DMA_CFG_SA  (1U << 14) /* Remove 1st byte (address) of each packet */
#define SPW_DMA_CFG_ENA (1U << 13) /* Enable separate node address for channel */
#define SPW_DMA_CFG_NS  (1U << 12) /* No spill */

/* TX packet flags */
#define SPW_TX_FLG_DCRC (1U << 17) /* Append data CRC */
#define SPW_TX_FLG_HCRC (1U << 16) /* Append header CRC */

/* clang-format off */
#define SPW_TX_FLG_NON_CRC(n) (((n) & 0xfU) << 8) /* Number of bytes from start that will not be included in CRC calculation */
#define SPW_TX_FLG_HDR_LEN(n) ((n) & 0xffU)       /* Number of bytes in header */
/* clang-format on */

/* RX packet flags */
#define SPW_RX_FLG_TR   (1U << 31) /* Packet truncated */
#define SPW_RX_FLG_DCRC (1U << 30) /* Data CRC error */
#define SPW_RX_FLG_HCRC (1U << 29) /* Header CRC error */
#define SPW_RX_FLG_EEP  (1U << 28) /* EEP termination */
#define SPW_RX_LEN_MSK  0x1ffffffU /* Packet length mask */


/* Default maximum value - can be overridden by board_config */
#ifndef SPW_MAX_PACKET_LEN
#define SPW_MAX_PACKET_LEN 1024
#endif


typedef struct {
	const uint8_t *hdr;
	const uint8_t *data;
	uint32_t dataLen;
	uint32_t flags;
} spw_txPacket_t;


typedef struct {
	uint8_t *buf;
	uint32_t flags;
} spw_rxPacket_t;


#define SPW_RX_DESC_CNT 128
#define SPW_TX_DESC_CNT 64


typedef struct {
	volatile uint32_t ctrl;
	uint32_t addr; /* RX buff phy addr - must be word aligned */
} spw_rxDesc_t;


typedef struct {
	volatile uint32_t ctrl;
	uint32_t hdrAddr;   /* TX header buff phy addr - does not have to be word aligned */
	uint32_t packetLen; /* TX packet length in bytes (without header) */
	uint32_t dataAddr;  /* TX data buff phy addr - does not have to be word aligned */
} spw_txDesc_t;


typedef struct {
	volatile uint32_t *vbase;
	unsigned int irq;
	uint8_t addr;
	uint8_t txDescFree;

	size_t sentDesc;
	size_t lastTxDesc;
	size_t nextRxDesc;

	bool rxAcknowledged[SPW_RX_DESC_CNT];

	handle_t ctrlLock;
	handle_t txLock;
	handle_t txIrqLock;
	handle_t rxLock;
	handle_t rxConfLock;
	handle_t cond;
	handle_t rxAckCond;

	volatile uint8_t (*txBuff)[SPW_MAX_PACKET_LEN];
	volatile uint8_t (*rxBuff)[SPW_MAX_PACKET_LEN];
	volatile spw_txDesc_t *txDesc;
	volatile spw_rxDesc_t *rxDesc;
} spw_dev_t;


typedef struct {
	struct {
		uint8_t addr;
		uint8_t mask;
	} node;
	struct {
		uint8_t addr;
		uint8_t mask;
		uint32_t flags;
	} dma;
} spw_config_t;


typedef struct {
	size_t nPackets;
} spw_rxConfig_t;


typedef struct {
	size_t firstDesc;
	size_t nPackets;
	uint32_t timeoutUs;
} spw_rx_t;


typedef struct {
	size_t nPackets;
	bool async;
} spw_tx_t;


typedef struct {
	/* Configure, RX, TX in one op */
	size_t nTxPackets;
	size_t nRxPackets;
	uint32_t rxTimeoutUs;
} spw_xfer_t;


static inline size_t spw_serializeTxMsg(uint32_t flags, uint32_t dataLen, const uint8_t *hdr, const uint8_t *data, uint8_t *buf, size_t bufsz)
{
	/* TX msg layout (single packet):
	 * | flags (inc. hdrLen) | dataLen |  hdr   |  data   |
	 * |         4 B         |   4 B   | hdrLen | dataLen |
	 */

	uint8_t hdrLen = flags & 0xffU;

	if (SPW_TX_MIN_BUFSZ + hdrLen + dataLen > bufsz) {
		return 0;
	}

	buf[0] = flags >> 24;
	buf[1] = (flags >> 16) & 0xffU;
	buf[2] = (flags >> 8) & 0xffU;
	buf[3] = flags & 0xffU;
	buf[4] = dataLen >> 24;
	buf[5] = (dataLen >> 16) & 0xffU;
	buf[6] = (dataLen >> 8) & 0xffU;
	buf[7] = dataLen & 0xffU;

	if (hdrLen != 0) {
		memcpy(&buf[SPW_TX_MIN_BUFSZ], hdr, hdrLen);
	}
	if (dataLen != 0) {
		memcpy(&buf[SPW_TX_MIN_BUFSZ] + hdrLen, data, dataLen);
	}

	return SPW_TX_MIN_BUFSZ + hdrLen + dataLen;
}


static inline size_t spw_deserializeRxMsg(const uint8_t *buf, spw_rxPacket_t *packet)
{
	/* RX msg layout (single packet):
	 * | flags (inc. dataLen)  |  data   |
	 * |        4 B            | dataLen |
	 */

	packet->flags = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	packet->buf = (uint8_t *)&buf[SPW_RX_MIN_BUFSZ];

	return SPW_RX_MIN_BUFSZ + ((packet->flags) & SPW_RX_LEN_MSK);
}


int spw_transmit(spw_dev_t *dev, const uint8_t *buf, size_t bufsz, const spw_tx_t *tx);


int spw_rxConfigure(spw_dev_t *dev, size_t *firstDesc, const size_t nPackets);


int spw_rxRead(spw_dev_t *dev, uint8_t *buf, size_t bufsz, size_t *readCnt, const spw_rx_t *rx);


int spw_configure(spw_dev_t *dev, const spw_config_t *config);


ssize_t spw_xferOp(spw_dev_t *dev, const uint8_t *txbuf, size_t txbufsz, uint8_t *rxbuf, size_t rxbufsz, const spw_xfer_t *xfer, size_t *readCnt);


int spw_initDev(unsigned int instance, spw_dev_t *spwdev);


#endif /* LIBGRSPW_H_ */
