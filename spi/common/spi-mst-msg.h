/*
 * Phoenix-RTOS
 *
 * SPI master message interface
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/*
 * SPI Master Message Interface
 * =====================================
 *
 * This API allows multiple SPI transactions to be executed atomically
 * within a single message (inspired by Linux kernel).
 *
 * The message interface uses a descriptor-based approach, where each
 * transaction segment is defined by a descriptor specifying its type
 * (TX, RX, or XFER) and length. The actual data buffers for transmission
 * and reception are packed into contiguous sections in the message buffers.
 * Descriptor type XFER indicates TX and RX of the same length.
 *
 * The SPI config is passed in the `i.raw` field of the message,
 * while the descriptors and data are packed and passed either in the `i.raw`
 * field (if they fit) or in the `i.data` buffer. The received data is returned
 * packed in the `o.raw` field (if it fits) or in the `o.data` buffer.
 * The user application is responsible for packing and unpacking the buffers
 * according to the layout defined by the descriptors. Helpers are provided
 * to facilitate this process.
 *
 * =====================================
 * Buffer Layout
 * =====================================
 *
 * Input Buffer Layout
 * -------------------------------------
 * The input buffer contains the list of transaction descriptors and the raw
 * data to be transmitted. The buffer can be assembled manually or by using
 * `spimsg_pack()` helper function.
 *
 *
 * +----------------------------+
 * | spimsg_desc_t[0]           |  Example: (Type=TX, Len=1)
 * | spimsg_desc_t[1]           |  Example: (Type=RX, Len=4)
 * | spimsg_desc_t[2]           |  Example: (Type=XFER, Len=2)
 * | ...                        |
 * | spimsg_desc_t[N-1]         |
 * +----------------------------+
 * | TX Payload Segment[0]      |  (Data for Desc[0])
 * +----------------------------+
 * | TX Payload Segment[2]      |  (Data for Desc[2]) - Desc[1] was RX, so it has no TX data
 * +----------------------------+
 * | ...                        |
 * +----------------------------+
 *
 * NOTE: The TX Payload section is contiguous. If a segment is type RX,
 * it contributes 0 bytes to this section.
 *
 *
 * Output Buffer Layout
 * -------------------------------------
 * The output buffer is populated by the driver with received data.
 *
 * +----------------------------+
 * | RX Payload Segment[1]      |  (Data from Desc[1]) - Desc[0] was TX, so it produced no output
 * +----------------------------+
 * | RX Payload Segment[2]      |  (Data from Desc[2])
 * +----------------------------+
 * | ...                        |
 * +----------------------------+
 *
 * NOTE: The RX Payload section is contiguous. If a segment is type TX,
 * it contributes 0 bytes to this section.
 *
 *
 * Example: Read Sensor (Write 1 byte Addr, Read 4 bytes Data)
 * -----------------------------------------------------------
 * Segment 0: TX, Len=1, Data=[0x50]
 * Segment 1: RX, Len=4
 *
 * [Input Buffer]  :: [Header] [Desc0(TX,1), Desc1(RX,4)] [0x50]
 * [Output Buffer] :: [0xAA, 0xBB, 0xCC, 0xDD]
 *
 *
 * Example: XFER Command (Write 2 bytes, Read 2 bytes)
 * -----------------------------------------------------------
 * Segment 0: XFER, Len=2, Data=[0x12, 0x34]
 * [Input Buffer]  :: [Header] [Desc0(XFER,2)] [0x12, 0x34]
 * [Output Buffer] :: [0x56, 0x78]
 *
 *
 * =====================================
 * Usage Patterns
 * =====================================
 *
 * Option 1: Using Helper Functions (Recommended)
 * ----------------------------------------------
 *
 * Define transfers using spimsg_transfer_t:
 *   uint8_t cmd[] = { 0x50 };
 *   spimsg_transfer_t ops[] = {
 *       { .type = spi_transfer_tx, .len = 1, .txBuf = cmd, .rxBuf = NULL },
 *       { .type = spi_transfer_rx, .len = 4, .txBuf = NULL, .rxBuf = NULL }
 *   };
 *
 * Pack into message buffer:
 *   uint8_t ibuf[128];
 *   size_t packedLen;
 *   spimsg_pack(ops, 2, ibuf, sizeof(ibuf), &packedLen);
 *
 * Send transaction:
 *   uint8_t obuf[64];
 *   spimsg_transaction(&oid, &cfg, ibuf, packedLen, obuf, sizeof(obuf));
 *
 * Unpack received data:
 *   spimsg_unpack(ops, 2, ibuf, packedLen, obuf, rxLen);
 *
 * Now ops[1].rxBuf points to received data in obuf
 *
 *
 * Option 2: Manual Buffer Management
 * ----------------------------------------------
 *
 * Manually construct input buffer with spi_transferDesc_t:
 *   struct {
 *       spi_transferDesc_t desc[2];
 *       uint8_t txData[1];
 *   } __attribute__((packed)) ibuf = {
 *       .desc = {
 *           { .type = spi_transfer_tx, .len = 1 },
 *           { .type = spi_transfer_rx, .len = 4 }
 *       },
 *       .txData = { 0x50 }
 *   };
 *
 * Send transaction and manually parse output:
 *   uint8_t obuf[4];
 *   spimsg_transaction(&oid, &cfg, &ibuf, sizeof(ibuf), obuf, sizeof(obuf));
 *
 * Manually extract: obuf[0..3] contains RX data from desc[1]
 *
 */


#ifndef PHOENIX_SPI_MST_MSG_H_
#define PHOENIX_SPI_MST_MSG_H_

#include <stddef.h>

#include <sys/msg.h>
#include <sys/types.h>

#include <spi-mst.h>


typedef struct {
	uint8_t bitOrder; /* SPI endianness (spi_lsb/msb) */
	uint8_t mode;     /* SPI clock mode (phase and polarity) */
	uint16_t descCnt; /* Number of descriptors */
	uint32_t speed;   /* SPI clock speed */
} spimsg_config_t;


typedef struct {
	spimsg_config_t cfg; /* SPI transaction configuration */
	uint8_t payload[0];
} spi_devctl_t;


typedef struct {
	uint32_t type;     /* spi_op_xfer/tx/rx */
	uint32_t len;      /* Length of the buffer, for XFER it's both TX and RX length */
	const void *txBuf; /* Source data (NULL for RX) */
	void *rxBuf;       /* RX data pointer - written by `spimsg_unpack()` for RX segments */
} spimsg_transfer_t;


_Static_assert(sizeof(spi_devctl_t) <= sizeof(((msg_t *)0)->i.raw), "spi_devctl_t too large to fit in msg.i.raw");


/* Packs the list of spimsg_transfer_t descriptors and associated TX buffers into a contiguous input buffer
 * according to the layout defined in this header.
 *
 * The caller is responsible for providing a sufficiently large buffer to hold the packed data.
 * The required buffer size can be calculated as:
 *   (sizeof(spi_transferDesc_t) * opsCnt) + totalTxDataLen
 * where totalTxDataLen is the sum of lengths of all TX and XFER segments.
 *
 * Params:
 * `ops`       : Array of transfer descriptors defining the transaction segments.
 * `opsCnt`    : Number of descriptors in the `ops` array.
 * `ibuf`      : Buffer to receive the packed data.
 * `ibufLen`   : Length of the input buffer.
 * `packedLen` : Output parameter to receive the actual size of the packed data.
 */
int spimsg_pack(const spimsg_transfer_t *ops, size_t opsCnt, void *ibuf, size_t ibufLen, size_t *packedLen);


/* Assigns RX buffer pointers in `ops` based on the provided input buffer and descriptors.
 * The input buffer is expected to be in the layout defined by `spimsg_pack()`.
 * The output buffer is the buffer where the RX data was stored by the driver.
 * This function does not copy any data, it only sets the `spimsg_transfer_t.rxBuf` pointers in the `ops` array
 * to point to the correct locations in the `obuf` where the RX data was stored by the driver.
 *
 * Params:
 * `ops`     : Array of transfer descriptors for which the `rxBuf` pointers will be assigned.
 * `opsCnt`  : Number of descriptors in the `ops` array.
 * `ibuf`    : Input buffer containing the packed descriptors and TX data.
 * `ibufLen` : Length of the input buffer.
 * `obuf`    : Output buffer containing the RX data received from the SPI driver.
 * `obufLen` : Length of the output buffer.
 */
int spimsg_unpack(spimsg_transfer_t *ops, size_t opsCnt, const void *ibuf, size_t ibufLen, const void *obuf, size_t obufLen);


/* Performs SPI transaction.
 * The function sends a message to the SPI driver with the specified configuration and data buffers.
 *
 * Params:
 * `oid`     : SPI device OID obtained from `spimsg_open()`.
 * `cfg`     : SPI transaction configuration (clock mode, speed, bit order, etc.)
 * `ibuf`    : Packed input buffer containing transaction descriptors and TX data (packed manually or using `spimsg_pack()`).
 * `ibufLen` : Length of the input buffer.
 * `obuf`    : Buffer to receive the RX data. The actual layout of the RX data is defined by the descriptors in the input buffer.
 * `obufLen` : Length of the RX buffer. The required size can be calculated as the sum of lengths of all RX and XFER segments.
 */
int spimsg_transaction(const oid_t *oid, const spimsg_config_t *cfg, const void *ibuf, size_t ibufLen, void *obuf, size_t obufLen);


/* Closes SPI message context pointed by `oid`. */
int spimsg_close(const oid_t *oid);


/* Opens SPI message context for the given SPI device and slave select line.
 * The returned `oid` can be used in subsequent calls to `spimsg_transaction()`.
 */
int spimsg_open(unsigned int dev, unsigned int ss, oid_t *oid);


#endif
