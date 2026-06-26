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


#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

#define LOG_TAG "ADE9113-SAMPLE: "

#include "log.h"
#include "sample.h"


void sample_writeStart(struct sampleCtx *ctx)
{
	assert(!ctx->writeInProgress);
	mutexLock(ctx->lock);
	ctx->writeInProgress = true;
}


/* non static inline in sample.h */
extern inline void sample_write(struct sampleCtx *ctx, const uint8_t *data, size_t size);


void sample_writeEnd(struct sampleCtx *ctx)
{
	assert(ctx->writeInProgress);
	if (ctx->nextWakeup <= ctx->written) {
		ctx->nextWakeup = UINT64_MAX;
		condBroadcast(ctx->cond);
	}
	ctx->writeInProgress = false;
	mutexUnlock(ctx->lock);
}


void sample_readerInit(struct sampleReader *reader, struct sampleCtx *ctx)
{
	mutexLock(ctx->lock);
	*reader = (struct sampleReader) {
		.ctx = ctx,
		.offs = 0,
		.open = false
	};
	mutexUnlock(ctx->lock);
}


void sample_readerOpen(struct sampleReader *reader)
{
	assert(reader->ctx != NULL);
	mutexLock(reader->ctx->lock);
	reader->open = true;
	reader->offs = reader->ctx->written;
	/* align initial read offset with sample start */
	reader->offs -= reader->offs % reader->ctx->sampleSize;
	mutexUnlock(reader->ctx->lock);
}


void sample_readerClose(struct sampleReader *reader)
{
	struct sampleCtx *ctx = reader->ctx;
	assert(ctx != NULL);
	mutexLock(ctx->lock);
	if (reader->open) {
		reader->offs = 0;
		reader->open = false;
		condBroadcast(ctx->cond);
	}
	mutexUnlock(ctx->lock);
}


static ssize_t sampleRead(struct sampleReader *reader, uint8_t *data, size_t size, bool block)
{
	const struct sampleCtx *ctx = reader->ctx;
	mutexLock(ctx->lock);

	if (block) {
		uint64_t needed = reader->offs + size;
		while ((reader->ctx->written < needed) && (reader->open)) {
			if (reader->ctx->nextWakeup > needed) {
				reader->ctx->nextWakeup = needed;
			}
			condWait(reader->ctx->cond, reader->ctx->lock, 0);
		}
	}
	if (!reader->open) {
		mutexUnlock(ctx->lock);
		return -1;
	}

	uint64_t readySize = ctx->written - reader->offs;
	if (readySize > ctx->mask) {
		/* reader too slow - whole buffer was filled */
		mutexUnlock(ctx->lock);
		return -1;
	}
	size = (size > readySize) ? readySize : size;

	uint32_t start = reader->offs & ctx->mask;
	uint32_t end = (reader->offs + size) & ctx->mask;
	if (start == end) {
		// pass
	}
	else if (start < end) {
		memcpy(data, (void *)ctx->buffer + start, end - start);
	}
	else {
		memcpy(data, (void *)ctx->buffer + start, size - end);
		memcpy(data + (size - end), (void *)ctx->buffer, end);
	}

	mutexUnlock(ctx->lock);
	reader->offs += size;
	return size;
}


ssize_t sample_readBlock(struct sampleReader *reader, uint8_t *data, size_t size)
{
	return sampleRead(reader, data, size, true);
}


ssize_t sample_read(struct sampleReader *reader, uint8_t *data, size_t size)
{
	return sampleRead(reader, data, size, false);
}


static void ctxCleanup(struct sampleCtx *ctx)
{
	/* NOTE: all function here are are safe (no-op) on default values */
	free(ctx->buffer);
	resourceDestroy(ctx->cond);
	resourceDestroy(ctx->lock);
	*ctx = (struct sampleCtx) { .buffer = NULL };
}


int sample_init(struct sampleCtx *ctx, unsigned expSize, unsigned sampleSize)
{
	if ((expSize > 31) || (((uint32_t)sampleSize >> expSize) > 0)) {
		return -1;
	}

	*ctx = (struct sampleCtx) {
		.written = 0,
		.sampleSize = sampleSize,
		.mask = ((uint32_t)1 << expSize) - 1,
		.buffer = malloc((uint32_t)1 << expSize)
	};

	if (ctx->buffer == NULL) {
		log_error("failed to allocate sample buffer");
		ctxCleanup(ctx);
		return -1;
	}

	if (mutexCreate(&ctx->lock) < 0) {
		log_error("mutex create failed");
		assert(false);
		ctxCleanup(ctx);
		return -1;
	}

	if (condCreate(&ctx->cond) < 0) {
		log_error("cond create failed");
		assert(false);
		ctxCleanup(ctx);
		return -1;
	}

	return 0;
}
