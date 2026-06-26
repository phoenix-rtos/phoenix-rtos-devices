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


#ifndef MSGAPI_H
#define MSGAPI_H

#include "sample.h"

#define MSGAPI_READER_COUNT 4

struct msgapiCtx {
	uint32_t port;
	handle_t lock;

	struct sampleReader readers[MSGAPI_READER_COUNT];
	unsigned opened;
};

int msgapi_init(struct msgapiCtx *msgapi, struct sampleCtx *ctx, const char *name);


int msgapi_serve(struct msgapiCtx *ctx);


#endif /* end of include guard: MSGAPI_H */
