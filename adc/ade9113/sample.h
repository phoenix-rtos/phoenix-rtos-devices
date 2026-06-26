/*
 * Phoenix-RTOS
 *
 * Simple ring buffer with multiple readers and fixed sample size
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef SAMPLE_H
#define SAMPLE_H

#include <sys/types.h>
#include <sys/threads.h>
#include <stddef.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>


struct sampleCtx {
	/* static configuration */
	unsigned sampleSize;
	uint64_t mask; /* mask used for calculation of block size modulo (size is power of 2) */

	/* modified under lock */
	bool writeInProgress;
	uint64_t written;
	uint64_t nextWakeup;

	handle_t lock;
	handle_t cond;
	uint8_t *buffer;
};


/*
 * Write sample data chunk/part
 *
 * This has to be called inside transaction that starts with `sample_writeStart` and ends with `sample_writeEnd`.
 * Multiple chunks of data can be written in single transaction. Whole transaction is atomic. Blocked readers are
 * notified when sample_writeEnd is called.
 */
inline void sample_write(struct sampleCtx *ctx, const uint8_t *data, size_t size)
{
	assert(ctx->writeInProgress);
	for (size_t i = 0; i < size; ++i) {
		ctx->buffer[ctx->written & ctx->mask] = data[i];
		ctx->written += 1;
	}
}
void sample_writeStart(struct sampleCtx *ctx);
void sample_writeEnd(struct sampleCtx *ctx);


struct sampleReader {
	struct sampleCtx *ctx;
	uint64_t offs;
	bool open;
};

/* initialize reader. First read will start with first sample received after readerInit is called */
void sample_readerInit(struct sampleReader *reader, struct sampleCtx *ctx);
void sample_readerOpen(struct sampleReader *reader);
void sample_readerClose(struct sampleReader *reader);
ssize_t sample_readBlock(struct sampleReader *reader, uint8_t *data, size_t size);
ssize_t sample_read(struct sampleReader *reader, uint8_t *data, size_t size);

/*
 * Initialize and allocate sample buffer
 *
 * @param[in]  sampleSize  size in bytes of single sample. New readers will start with offset aligned to this value
 * @param[in]  expSize     size of ring buffer as exponent of two (size = 2^expSize)
 */
int sample_init(struct sampleCtx *ctx, unsigned expSize, unsigned sampleSize);


#endif /* end of include guard: SAMPLE_H */
