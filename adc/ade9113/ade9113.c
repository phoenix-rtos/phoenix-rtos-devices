/*
 * Phoenix-RTOS
 *
 * ADE9113 command/response handling (4 chained chips)
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define LOG_TAG "ADE9113: "

#include "log.h"
#include "ade9113_regs.h"
#include "ade9113.h"


static uint8_t crc8(const uint8_t *data, size_t len)
{
	const uint8_t poly = 0x07;
	const uint8_t xorOut = 0x55;
	uint8_t v = 0x00;
	for (size_t i = 0; i < len; ++i) {
		v = v ^ data[i];
		for (size_t b = 0; b < 8; ++b) {
			v = (v >= 0x80) ? (v << 1) ^ poly : (v << 1);
		}
	}
	return v ^ xorOut;
}


static uint16_t crc16(const uint8_t *data, size_t len)
{
	const uint16_t poly = 0x1021;
	const uint16_t xorOut = 0x0000;
	uint16_t v = 0xffff;
	for (size_t i = 0; i < len; ++i) {
		v = v ^ data[i] << 8;
		for (size_t b = 0; b < 8; ++b) {
			v = (v >= 0x8000) ? (v << 1) ^ poly : (v << 1);
		}
	}
	return v ^ xorOut;
}


static inline int prepareRead(uint8_t reg, uint8_t *data, size_t size)
{
	if (size != 16) {
		return -1;
	}
	memset(data, 0, 16);
	data[12] = 0xC0;
	data[13] = reg;
	data[14] = 0x00;
	data[15] = crc8(&data[12], 3);
	return 16;
}


static int prepareWrite(uint8_t reg, uint8_t value, uint8_t *data, size_t size)
{
	if (size != 16) {
		return -1;
	}
	memset(data, 0, 16);
	data[12] = 0x40;
	data[13] = reg;
	data[14] = value;
	data[15] = crc8(&data[12], 3);
	return 16;
}


const char *ade9113_checkResponse(const uint8_t *data, size_t len)
{
	if (len != 16) {
		return "invalid length";
	}
	uint16_t crcA = crc16(data, 14);
	uint16_t crcB = data[14] | (data[15] << 8);
	if (crcA != crcB) {
		return "CRC16 mismatch";
	}
	if ((data[0] >> 7) != 0) {
		/* read response */
		if ((data[0] & 0x02) != 0) {
			return "read response CRC error bit set";
		}
	}
	else {
		/* write response */
		if ((data[0] & 0x02) != 0) {
			return "write response CRC error bit set";
		}
	}
	return NULL;
}


int ade9113_writeRegsDifferent(struct ade9113_ctx *ctx, uint8_t reg, uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	/* reverse order as first chip in chain receives last message */
	const uint8_t values[4] = { d, c, b, a };

	uint8_t req[16 * 4] = { 0 };
	uint8_t rsp[16 * 4] = { 0 };

	for (int i = 0; i < 4; ++i) {
		prepareWrite(reg, values[i], &req[16 * i], 16);
	}
	ctx->spiExchange(ctx->userData, req, rsp, sizeof(req));

	/* SCRATCH read is used here as NOP command to read result from actual write */
	for (int i = 0; i < 4; ++i) {
		prepareRead(ADE9113_SCRATCH, &req[16 * i], 16);
	}
	ctx->spiExchange(ctx->userData, req, rsp, sizeof(req));

	int ret = 0;
	for (int i = 0; i < 4; ++i) {
		const char *error = ade9113_checkResponse(&rsp[16 * i], 16);
		if (error != NULL) {
			log_error("write[%d] failed: %s", i, error);
			ret = -1;
		}
	}
	return ret;
}


int ade9113_readRegs(struct ade9113_ctx *ctx, uint8_t reg, uint8_t *values, uint8_t size)
{
	if (size != 4) {
		return -1;
	}

	uint8_t req[16 * 4] = { 0 };
	uint8_t rsp[16 * 4] = { 0 };

	for (int i = 0; i < 4; ++i) {
		prepareRead(reg, &req[16 * i], 16);
	}
	ctx->spiExchange(ctx->userData, req, rsp, sizeof(req));

	/* SCRATCH read is used here as NOP command to read result from actual read */
	for (int i = 0; i < 4; ++i) {
		prepareRead(ADE9113_SCRATCH, &req[16 * i], 16);
	}
	ctx->spiExchange(ctx->userData, req, rsp, sizeof(req));

	int ret = 0;
	for (int i = 0; i < 4; ++i) {
		const char *error = ade9113_checkResponse(&rsp[16 * i], 16);
		if (error != NULL) {
			log_error("read[%d] failed: %s", i, error);
			ret = -1;
		}
	}
	for (int i = 0; i < 4; ++i) {
		values[i] = rsp[(16 * (3 - i)) + 13];
	}
	return ret;
}
