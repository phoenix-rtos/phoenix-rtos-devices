/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MULTI_SPACEWIRE_H_
#define _MULTI_SPACEWIRE_H_


#include <phoenix/msg.h>
#include <string.h>
#include <sys/msg.h>
#include <stdbool.h>

enum {
	id_spw0 = 0u,
	id_spw1,
	id_spw2,
	id_spw3,
	id_spw4,
	id_spw5,
};


#define SPW_RX_MIN_BUFSZ 4
#define SPW_TX_MIN_BUFSZ 8

/* DMA configuration */
#define SPW_DMA_CFG_LE  (1u << 16) /* Disable TX when link error occurs */
#define SPW_DMA_CFG_SP  (1u << 15) /* Remove 2nd (and 1st) byte (protocol id) of each packet */
#define SPW_DMA_CFG_SA  (1u << 14) /* Remove 1st byte (address) of each packet */
#define SPW_DMA_CFG_ENA (1u << 13) /* Enable separate node address for channel */
#define SPW_DMA_CFG_NS  (1u << 12) /* No spill */

/* TX packet flags */
#define SPW_TX_FLG_DCRC (1u << 17) /* Append data CRC */
#define SPW_TX_FLG_HCRC (1u << 16) /* Append header CRC */

/* clang-format off */
#define SPW_TX_FLG_NON_CRC(n) (((n) & 0xfu) << 8) /* Number of bytes from start that will not be included in CRC calculation */
#define SPW_TX_FLG_HDR_LEN(n) ((n) & 0xffu)       /* Number of bytes in header */
/* clang-format on */

/* RX packet flags */
#define SPW_RX_FLG_TR   (1u << 31) /* Packet truncated */
#define SPW_RX_FLG_DCRC (1u << 30) /* Data CRC error */
#define SPW_RX_FLG_HCRC (1u << 29) /* Header CRC error */
#define SPW_RX_FLG_EEP  (1u << 28) /* EEP termination */
#define SPW_RX_LEN_MSK  0x1ffffffu /* Packet length mask */


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
	/* clang-format off */
	enum { spw_config = 0, spw_rxConfig, spw_rx, spw_tx } type;
	/* clang-format on */
	union {
		spw_config_t config;
		struct {
			size_t nPackets;
		} rxConfig;
		struct {
			size_t firstDesc;
			size_t nPackets;
		} rx;
		struct {
			size_t nPackets;
			bool async;
		} tx;
	} task;

} spw_t;


_Static_assert(sizeof(spw_t) <= sizeof(((msg_t *)0)->i.raw), "spw_t exceeds size of msg.i.raw");


typedef struct {
	size_t val;
} spw_o_t;


_Static_assert(sizeof(spw_o_t) <= sizeof(((msg_t *)0)->o.raw), "spw_t exceeds size of msg.o.raw");


static inline size_t spw_serializeTxMsg(uint32_t flags, uint32_t dataLen, const uint8_t *hdr, const uint8_t *data, uint8_t *buf, size_t bufsz)
{
	/* TX msg layout (single packet):
	 * | flags (inc. hdrLen) | dataLen |  hdr   |  data   |
	 * |         4 B         |   4 B   | hdrLen | dataLen |
	 */

	uint8_t hdrLen = flags & 0xffu;

	if (SPW_TX_MIN_BUFSZ + hdrLen + dataLen > bufsz) {
		return 0;
	}

	buf[0] = flags >> 24;
	buf[1] = (flags >> 16) & 0xffu;
	buf[2] = (flags >> 8) & 0xffu;
	buf[3] = flags & 0xffu;
	buf[4] = dataLen >> 24;
	buf[5] = (dataLen >> 16) & 0xffu;
	buf[6] = (dataLen >> 8) & 0xffu;
	buf[7] = dataLen & 0xffu;

	memcpy(&buf[SPW_TX_MIN_BUFSZ], hdr, hdrLen);
	memcpy(&buf[SPW_TX_MIN_BUFSZ] + hdrLen, data, dataLen);

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


void spw_handleMsg(msg_t *msg, int dev);


int spw_createDevs(oid_t *oid);


int spw_init(void);


#endif
