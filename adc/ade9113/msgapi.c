/*
 * Phoenix-RTOS
 *
 * Basic sample server over messages
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <stddef.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <posix/utils.h>
#include <sys/platform.h>
#include <sys/threads.h>

#define LOG_TAG "ADE9113-API: "

#include "log.h"
#include "sample.h"
#include "msgapi.h"


int msgapi_init(struct msgapiCtx *ctx, struct sampleCtx *sample, const char *name)
{
	*ctx = (struct msgapiCtx) {
		.port = UINT32_MAX,
	};

	mutexCreate(&ctx->lock);

	for (unsigned ch = 0; ch < MSGAPI_READER_COUNT; ++ch) {
		sample_readerInit(&ctx->readers[ch], sample);
	}

	int res = portCreate(&ctx->port);
	if (res != EOK) {
		log_error("could not create port: %d", res);
		return -1;
	}

	oid_t dev = { .id = 0, .port = ctx->port };
	res = create_dev(&dev, name);
	if (res != EOK) {
		log_error("could not create device ade9113");
		return -1;
	}

	return 0;
}


int msgapi_serve(struct msgapiCtx *ctx)
{
	while (true) {
		msg_rid_t rid;
		msg_t msg;
		if (msgRecv(ctx->port, &msg, &rid) < 0) {
			continue;
		}
		struct sampleReader *reader = NULL;
		if ((msg.oid.id > 0) && (msg.oid.id <= MSGAPI_READER_COUNT)) {
			reader = &ctx->readers[msg.oid.id - 1];
		}

		switch (msg.type) {
			case mtOpen: {
				if (msg.oid.id != 0) {
					msg.o.err = -EBADF;
					break;
				}
				msg.o.err = -EBUSY;

				if (mutexLock(ctx->lock) < 0) {
					msg.o.err = -EIO;
					break;
				}
				for (unsigned ch = 0; ch < MSGAPI_READER_COUNT; ++ch) {
					if (!ctx->readers[ch].open) {
						reader = &ctx->readers[ch];
						/*
						 * TODO: consider adding option to read already queued samples
						 * For now only new samples (collected after open) can be read
						 */
						sample_readerOpen(reader);
						ctx->opened += 1;
						msg.o.err = ch + 1; /* nonnegative err from open is to pass oid.id */
						log_info("new msgapi session: ch=%d pid=%d opened=%u\n", ch + 1, msg.pid, ctx->opened);
						break;
					}
				}
				mutexUnlock(ctx->lock);

				if (msg.o.err == -EBUSY) {
					log_error("failed to allocate msgapi session: pid=%d opened=%u\n", msg.pid, ctx->opened);
				}
				break;
			}

			case mtClose: {
				if (reader == NULL) {
					msg.o.err = -EBADF;
					break;
				}

				if (mutexLock(ctx->lock) < 0) {
					msg.o.err = -EIO;
					break;
				}
				sample_readerClose(reader);
				msg.o.err = EOK;
				ctx->opened -= 1;
				log_info("closing msgapi session: ch=%d pid=%d opened=%u\n", (int)msg.oid.id, msg.pid, ctx->opened);
				mutexUnlock(ctx->lock);
				break;
			}

			case mtGetAttr:
			case mtSetAttr:
				msg.o.err = EOK;
				break;

			case mtRead: {
				if (reader == NULL) {
					msg.o.err = -EBADF;
					break;
				}
				/* TODO: create library for implementing async blocking queue reads (see klog reading) and use it here */
				msg.o.err = sample_readBlock(reader, msg.o.data, msg.o.size);
				break;
			}

			case mtWrite: {
				msg.o.err = -ENOSYS;
				break;
			}

			default:
				msg.o.err = -ENOSYS;
				break;
		}
		msgRespond(ctx->port, &msg, rid);
	}
}
