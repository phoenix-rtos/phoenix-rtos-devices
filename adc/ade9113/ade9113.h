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

#ifndef ADE9113_H
#define ADE9113_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>


struct ade9113_ctx {
	int (*spiExchange)(void *userData, const uint8_t *dataIn, uint8_t *dataOut, size_t len);
	void *userData;
};


const char *ade9113_checkResponse(const uint8_t *data, size_t len);


int ade9113_readRegs(struct ade9113_ctx *ctx, uint8_t reg, uint8_t *values, uint8_t size);


/* read 16 byte value from HI, LO register pair. `reg` contains address of HI register. */
int ade9113_readRegsU16(struct ade9113_ctx *ctx, uint8_t reg, uint16_t *values, uint8_t size);


int ade9113_writeRegsDifferent(struct ade9113_ctx *ctx, uint8_t reg, uint8_t a, uint8_t b, uint8_t c, uint8_t d);


static inline int ade9113_writeRegs(struct ade9113_ctx *ctx, uint8_t reg, uint8_t a)
{
	return ade9113_writeRegsDifferent(ctx, reg, a, a, a, a);
}

#endif /* end of include guard: ADE9113_H */
