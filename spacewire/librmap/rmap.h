/*
 * Phoenix-RTOS
 *
 * RMAP packet building/parsing
 * Ref. ECSS‐E‐ST‐50‐52C
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SPACEWIRE_RMAP_H_
#define SPACEWIRE_RMAP_H_


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>


#define RMAP_STATUS_SUCCESS            0x00U
#define RMAP_STATUS_GENERAL_ERROR      0x01U
#define RMAP_STATUS_UNUSED_PACKET_TYPE 0x02U
#define RMAP_STATUS_INVALID_KEY        0x03U
#define RMAP_STATUS_INVALID_DATA_CRC   0x04U
#define RMAP_STATUS_EARLY_EOP          0x05U
#define RMAP_STATUS_TOO_MUCH_DATA      0x06U
#define RMAP_STATUS_EEP                0x07U
/* 0x08 reserved */
#define RMAP_STATUS_VERIFY_BUF_OVERRUN   0x09U
#define RMAP_STATUS_NOTIMPL_NOTAUTH      0x0aU
#define RMAP_STATUS_RMW_DATA_LEN_ERR     0x0bU
#define RMAP_STATUS_INV_TGT_LOGICAL_ADDR 0x0cU


/* Configuration for building an RMAP Command */
typedef struct {
	uint8_t targetLogicalAddr;
	uint8_t initiatorLogicalAddr;
	uint8_t key;
	uint16_t transactionId;

	uint8_t extendedAddr;
	uint32_t memoryAddr;

	uint8_t *targetPath;
	uint8_t targetPathLen;

	uint8_t *replyPath;
	uint8_t replyPathLen; /* Max 12 */

	bool verifyData;    /* True = Verify before write (not applicable for Read/RMW). */
	bool requireAck;    /* True = Request reply from target */
	bool incrementAddr; /* True = Increment address (block transfer) */
} rmap_params_t;


/* Parsed RMAP Reply */
typedef struct {
	uint8_t initiatorLogicalAddr;
	uint8_t targetLogicalAddr;
	uint8_t status;
	uint16_t transactionId;

	bool isReadReply;
	uint32_t dataLen;    /* Only valid for read/rmw reply */
	const uint8_t *data; /* Zero-copy pointer to payload in the Rx buffer */
} rmap_reply_t;


ssize_t rmap_buildWriteCmd(const rmap_params_t *params, uint8_t *header, size_t headerLen, size_t dataLen);


ssize_t rmap_buildReadCmd(const rmap_params_t *params, uint8_t *header, size_t headerLen, size_t readLen);


ssize_t rmap_buildReadModifyWriteCmd(const rmap_params_t *params, uint8_t *header, size_t headerLen, size_t dataLen);


ssize_t rmap_parseReply(const uint8_t *packet, size_t packetLen, rmap_reply_t *reply);


#endif /* SPACEWIRE_RMAP_H_ */
