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
#include <errno.h>
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
		.refs = 0,
		.state = sample_stateFree
	};
	mutexUnlock(ctx->lock);
}


int sample_readerOpen(struct sampleReader *reader)
{
	assert(reader->ctx != NULL);
	mutexLock(reader->ctx->lock);
	int ret = -1;
	if (reader->state == sample_stateFree) {
		ret = 0;
		reader->state = sample_stateOpen;
		reader->offs = reader->ctx->written;

		/* align initial read offset with sample start */
		reader->offs -= reader->offs % reader->ctx->sampleSize;
	}
	mutexUnlock(reader->ctx->lock);
	return ret;
}


void sample_readerClose(struct sampleReader *reader, bool release)
{
	struct sampleCtx *ctx = reader->ctx;
	assert(ctx != NULL);
	mutexLock(ctx->lock);
	if (reader->state == sample_stateOpen) {
		reader->offs = 0;
		reader->state = sample_stateClosing;
		condBroadcast(ctx->cond);

		/* wait for all blocking operations to be aborted */
		while (reader->refs > 0) {
			mutexUnlock(reader->ctx->lock);
			usleep(1000);
			mutexLock(reader->ctx->lock);
		}
	}
	if (release && (reader->state == sample_stateClosing)) {
		reader->state = sample_stateFree;
	}
	mutexUnlock(ctx->lock);
}

int checkReadable(struct sampleReader *reader)
{
	switch (reader->state) {
		case sample_stateFree:
			return -EBADF;
		case sample_stateClosing:
			return -EBADFD;
		case sample_stateOpen:
			return 0;
		default:
			assert(false);
	}
}


static ssize_t sampleRead(struct sampleReader *reader, uint8_t *data, size_t size, bool block)
{
	struct sampleCtx *ctx = reader->ctx;
	mutexLock(ctx->lock);

	int err = checkReadable(reader);
	if (err < 0) {
		mutexUnlock(ctx->lock);
		return err;
	}

	if (block) {
		uint64_t needed = reader->offs + size;
		reader->refs += 1;
		while ((ctx->written < needed) && (reader->state == sample_stateOpen)) {
			if (ctx->nextWakeup > needed) {
				ctx->nextWakeup = needed;
			}
			condWait(ctx->cond, ctx->lock, 0);
		}
		reader->refs -= 1;

		int err = checkReadable(reader);
		if (err < 0) {
			mutexUnlock(ctx->lock);
			return err;
		}
	}

	uint64_t readySize = ctx->written - reader->offs;
	if (readySize > ctx->mask) {
		/* reader too slow - whole buffer was filled */
		mutexUnlock(ctx->lock);
		return -EPIPE;
	}
	size = (size > readySize) ? readySize : size;

	uint32_t start = reader->offs & ctx->mask;
	uint32_t end = (reader->offs + size) & ctx->mask;
	if (start == end) {
		/* pass */
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


ssize_t sample_readPoll(struct sampleReader *reader)
{
	const struct sampleCtx *ctx = reader->ctx;
	mutexLock(ctx->lock);
	size_t readySize = ctx->written - reader->offs;
	mutexUnlock(ctx->lock);
	return readySize;
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
	/* NOTE: all calls here are are safe (no-op) on default values */
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
