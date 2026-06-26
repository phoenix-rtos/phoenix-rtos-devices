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


struct ade9113_ctx {
	int (*spiExchange)(void *userData, const uint8_t *dataIn, uint8_t *dataOut, size_t len);
	void *userData;
};


const char *ade9113_checkResponse(const uint8_t *data, size_t len);


int ade9113_readRegs(struct ade9113_ctx *ctx, uint8_t reg, uint8_t *values, uint8_t size);


int ade9113_writeRegsDifferent(struct ade9113_ctx *ctx, uint8_t reg, uint8_t a, uint8_t b, uint8_t c, uint8_t d);


static inline int ade9113_writeRegs(struct ade9113_ctx *ctx, uint8_t reg, uint8_t a)
{
	return ade9113_writeRegsDifferent(ctx, reg, a, a, a, a);
}
