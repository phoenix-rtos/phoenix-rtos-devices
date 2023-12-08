/*
 * Phoenix-RTOS
 *
 * GRLIB multi driver main
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _GRLIB_MULTI_H_
#define _GRLIB_MULTI_H_


#include <stdbool.h>
#include <string.h>
#include <sys/types.h>


/* clang-format off */
/* IDs of special files OIDs */
enum { id_gpio0 = 0, id_gpio1, id_spi0, id_spi1, id_uart0, id_uart1, id_uart2, id_uart3, id_uart4,
	id_uart5, id_console, id_adc0, id_adc1, id_adc2, id_adc3, id_adc4, id_adc5, id_adc6, id_adc7,
	id_spw0, id_spw1, id_spw2, id_spw3, id_spw4, id_spw5,
	id_pseudoNull, id_pseudoZero, id_pseudoFull, id_pseudoRandom, id_kmsgctrl };
/* clang-format on */


#pragma pack(push, 8)


/* GPIO */


typedef struct {
	/* clang-format off */
	enum { gpio_setPort = 0, gpio_getPort, gpio_setDir, gpio_getDir } type;
	/* clang-format on */
	union {
		struct {
			uint32_t mask;
			uint32_t val;
		} port;

		struct {
			uint32_t mask;
			uint32_t val;
		} dir;
	};
} gpio_t;


/* SPI */


typedef struct {
	uint8_t sck;
	uint8_t mosi;
	uint8_t miso;
	uint8_t cs;
} spi_pins_t;


/* clang-format off */
enum { spi_lsb = 0, spi_msb };

enum { spi_mode0 = 0, spi_mode1, spi_mode2, spi_mode3 };
/* clang-format on */


/* SPI_FREQ = SYSCLK_FREQ / ((4 - (2 * prescFactor)) * (prescaler + 1))
 * Setting div16 further divides the frequency by 16.
 */


typedef struct {
	uint8_t byteOrder;
	uint8_t mode;
	uint8_t prescFactor;
	uint8_t prescaler; /* 0x0 - 0xf */
	uint8_t div16;
} spi_config_t;


typedef struct {
	/* clang-format off */
	enum { spi_setPins = 0, spi_config, spi_transaction } type;
	/* clang-format on */

	union {
		spi_pins_t pins;
		spi_config_t config;

		struct {
			uint8_t slaveMsk; /* 0x1 - 0xf */
			size_t len;
		} transaction;
	};
} spi_t;


/* ADC */

/* clang-format off */
enum { adc_modeDiff = 0, adc_modeSingle };
enum { adc_sampleCnt1 = 0, adc_sampleCnt2, adc_sampleCnt3, adc_sampleCnt4 };
/* clang-format on */


typedef struct {
	uint32_t sampleRate;
	uint8_t mode;
	uint8_t sampleCnt;
} adc_config_t;


typedef struct {
	/* clang-format off */
	enum { adc_config = 0 } type;
	/* clang-format on */

	union {
		adc_config_t config;
	};
} adc_t;


/* SpaceWire */

#define SPW_RX_MIN_BUFSZ 4
#define SPW_TX_MIN_BUFSZ 8

/* DMA configuration */
#define SPW_DMA_CFG_LE  (1 << 16) /* Disable TX when link error occurs */
#define SPW_DMA_CFG_SP  (1 << 15) /* Remove 2nd (and 1st) byte (protocol id) of each packet */
#define SPW_DMA_CFG_SA  (1 << 14) /* Remove 1st byte (address) of each packet */
#define SPW_DMA_CFG_ENA (1 << 13) /* Enable separate node address for channel */
#define SPW_DMA_CFG_NS  (1 << 12) /* No spill */

/* TX packet flags */
#define SPW_TX_FLG_DCRC (1 << 17) /* Append data CRC */
#define SPW_TX_FLG_HCRC (1 << 16) /* Append header CRC */

/* clang-format off */
#define SPW_TX_FLG_NON_CRC(n) (((n) & 0xfu) << 8) /* Number of bytes from start that will not be included in CRC calculation */
#define SPW_TX_FLG_HDR_LEN(n) ((n) & 0xffu)       /* Number of bytes in header */
/* clang-format on */

/* RX packet flags */
#define SPW_RX_FLG_TR   (1 << 31)  /* Packet truncated */
#define SPW_RX_FLG_DCRC (1 << 30)  /* Data CRC error */
#define SPW_RX_FLG_HCRC (1 << 29)  /* Header CRC error */
#define SPW_RX_FLG_EEP  (1 << 28)  /* EEP termination */
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


/* MULTI */


typedef struct {
	id_t id;

	union {
		gpio_t gpio;
		spi_t spi;
		adc_t adc;
		spw_t spw;
	};
} multi_i_t;


typedef struct {
	int err;
	unsigned int val;
} multi_o_t;


#pragma pack(pop)


static inline size_t multi_spwSerializeTxMsg(uint32_t flags, uint32_t dataLen, const uint8_t *hdr, const uint8_t *data, uint8_t *buf, size_t bufsz)
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


static inline size_t multi_spwDeserializeRxMsg(const uint8_t *buf, spw_rxPacket_t *packet)
{
	/* RX msg layout (single packet):
	 * | flags (inc. dataLen)  |  data   |
	 * |        4 B            | dataLen |
	 */

	packet->flags = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	packet->buf = (uint8_t *)&buf[SPW_RX_MIN_BUFSZ];

	return SPW_RX_MIN_BUFSZ + ((packet->flags) & SPW_RX_LEN_MSK);
}


#endif
