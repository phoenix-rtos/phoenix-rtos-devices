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

#include <errno.h>
#include <string.h>

#include "rmap.h"

#define RMAP_MIN_HEADER_SIZE     15
#define RMAP_MIN_READ_REPLY_SIZE 12
#define RMAP_MIN_REPLY_SIZE      8

#define RMAP_PROTOCOL_ID 0x01U

#define RMAP_MAX_DATA_LEN 0x00ffffffU

#define INSTR_PACKET_TYPE_MASK  (0x3U << 6)
#define INSTR_PACKET_TYPE_CMD   (0x1U << 6)
#define INSTR_PACKET_TYPE_REPLY (0x0U << 6)
#define INSTR_CMD_WRITE         (1U << 5)
#define INSTR_CMD_VERIFY        (1U << 4)
#define INSTR_CMD_REPLY_REQ     (1U << 3)
#define INSTR_CMD_INC_ADDR      (1U << 2)
#define INSTR_REPLY_MASK        0x3U
#define INSTR_REPLY_12B         0x3U
#define INSTR_REPLY_8B          0x2U
#define INSTR_REPLY_4B          0x1U
#define INSTR_REPLY_0B          0x0U


typedef enum {
	rmap_write = 0,
	rmap_read,
	rmap_rmw,
} rmap_op_t;


static ssize_t buildCmd(const rmap_params_t *params, uint8_t *header, size_t headerLen, size_t dataLen, rmap_op_t opType)
{
	if ((params->replyPathLen > 12) || (dataLen > RMAP_MAX_DATA_LEN)) {
		return -EINVAL;
	}

	uint8_t instruction = 0;
	instruction |= INSTR_PACKET_TYPE_CMD;
	instruction |= (opType == rmap_write) ? INSTR_CMD_WRITE : 0;
	instruction |= ((opType == rmap_rmw) || params->verifyData) ? INSTR_CMD_VERIFY : 0;

	instruction |= params->requireAck ? INSTR_CMD_REPLY_REQ : 0;
	instruction |= params->incrementAddr ? INSTR_CMD_INC_ADDR : 0;
	instruction |= (params->replyPathLen + 3) / 4; /* 2-bit reply path size */

	uint8_t zeroPad = (4 - (params->replyPathLen % 4)) % 4;

	if (headerLen < RMAP_MIN_HEADER_SIZE + zeroPad + params->replyPathLen) {
		return -ENOMEM;
	}

	size_t offs = 0;
	header[offs++] = params->targetLogicalAddr;
	header[offs++] = RMAP_PROTOCOL_ID;
	header[offs++] = instruction;
	header[offs++] = params->key;

	/* Reply address */
	memset(&header[offs], 0, zeroPad);
	offs += zeroPad;
	memcpy(&header[offs], params->replyPath, params->replyPathLen);
	offs += params->replyPathLen;

	header[offs++] = params->initiatorLogicalAddr;
	header[offs++] = (params->transactionId >> 8) & 0xffU;
	header[offs++] = params->transactionId & 0xffU;
	header[offs++] = params->extendedAddr;

	/* Address - MSB first */
	header[offs++] = (params->memoryAddr >> 24) & 0xffU;
	header[offs++] = (params->memoryAddr >> 16) & 0xffU;
	header[offs++] = (params->memoryAddr >> 8) & 0xffU;
	header[offs++] = params->memoryAddr & 0xffU;

	/* Data length - 24 bits, MSB first */
	header[offs++] = (dataLen >> 16) & 0xffU;
	header[offs++] = (dataLen >> 8) & 0xffU;
	header[offs++] = dataLen & 0xffU;
	/* CRC added by HW */

	return offs;
}


ssize_t rmap_buildWriteCmd(const rmap_params_t *params, uint8_t *header, size_t headerLen, size_t dataLen)
{
	return buildCmd(params, header, headerLen, dataLen, rmap_write);
}


ssize_t rmap_buildReadCmd(const rmap_params_t *params, uint8_t *header, size_t headerLen, size_t readLen)
{
	if (!params->requireAck || params->verifyData) {
		/* Read needs a response and no data verification */
		return -EINVAL;
	}
	return buildCmd(params, header, headerLen, readLen, rmap_read);
}


ssize_t rmap_buildReadModifyWriteCmd(const rmap_params_t *params, uint8_t *header, size_t headerLen, size_t dataLen)
{
	if ((dataLen != 0x00U) && (dataLen != 0x02U) && (dataLen != 0x04U) && (dataLen != 0x06U) && (dataLen != 0x08U)) {
		return -EINVAL;
	}
	if (!params->requireAck || !params->incrementAddr) {
		/* RMW needs a response and address increment */
		return -EINVAL;
	}

	return buildCmd(params, header, headerLen, dataLen, rmap_rmw);
}


ssize_t rmap_parseReply(const uint8_t *packet, size_t packetLen, rmap_reply_t *reply)
{
	if (packetLen < RMAP_MIN_REPLY_SIZE) {
		return -EINVAL;
	}

	size_t offs = 0;
	/* Assuming packet is stripped of the SpW addresses */
	reply->initiatorLogicalAddr = packet[offs++];
	if (packet[offs++] != RMAP_PROTOCOL_ID) {
		return -EPROTO;
	}
	uint8_t instruction = packet[offs++];
	if ((instruction & INSTR_PACKET_TYPE_MASK) != INSTR_PACKET_TYPE_REPLY) {
		return -EPROTO;
	}
	reply->isReadReply = (instruction & INSTR_CMD_WRITE) == 0;
	reply->status = packet[offs++];
	reply->targetLogicalAddr = packet[offs++];
	reply->transactionId = packet[offs++] << 8;
	reply->transactionId |= packet[offs++];

	if (reply->isReadReply) {
		if (packetLen < RMAP_MIN_READ_REPLY_SIZE) {
			return -EINVAL;
		}
		/* reserved byte */
		++offs;
		reply->dataLen = packet[offs++] << 16;
		reply->dataLen |= packet[offs++] << 8;
		reply->dataLen |= packet[offs++];
		/* Skip CRC byte */
		reply->data = &packet[offs + 1];
	}
	else {
		reply->dataLen = 0;
		reply->data = NULL;
	}
	/* CRC verified by HW */
	++offs;

	return offs;
}
