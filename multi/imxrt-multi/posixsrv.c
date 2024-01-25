/*
 * Phoenix-RTOS
 *
 * i.MX RT builtin posix server (libposixsrv)
 *
 * Copyright 2023-2024 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>
#include "config.h"

#if BUILTIN_POSIXSRV

#include <posixsrv.h>
#include <sys/threads.h>

#ifndef POSIXSRV_PRIO
#define POSIXSRV_PRIO IMXRT_MULTI_PRIO
#endif

#ifndef POSIXSRV_THREADS_NO
#define POSIXSRV_THREADS_NO 1
#endif


struct {
	char stacks[2 + POSIXSRV_THREADS_NO][_PAGE_SIZE] __attribute__((aligned(8)));
	unsigned srvPort;
	unsigned eventPort;
} libposixsrv_common;


int posixsrv_start(void)
{
	if (posixsrv_init(&libposixsrv_common.srvPort, &libposixsrv_common.eventPort) < 0) {
		return -1;
	}

	beginthread(posixsrv_threadRqTimeout, POSIXSRV_PRIO, libposixsrv_common.stacks[0], sizeof(libposixsrv_common.stacks[0]), NULL);
	beginthread(posixsrv_threadMain, POSIXSRV_PRIO, libposixsrv_common.stacks[1], sizeof(libposixsrv_common.stacks[1]), (void *)libposixsrv_common.eventPort);

	for (int i = 2; i < sizeof(libposixsrv_common.stacks) / sizeof(libposixsrv_common.stacks[0]); ++i) {
		beginthread(posixsrv_threadMain, POSIXSRV_PRIO, libposixsrv_common.stacks[i], sizeof(libposixsrv_common.stacks[i]), (void *)libposixsrv_common.srvPort);
	}

	return 0;
}

#endif
