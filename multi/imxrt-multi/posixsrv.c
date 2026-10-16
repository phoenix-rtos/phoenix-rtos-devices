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
	char stackRqTimeout[2 * _PAGE_SIZE] __attribute__((aligned(8)));
	char stacksMain[1 + POSIXSRV_THREADS_NO][3 * _PAGE_SIZE] __attribute__((aligned(8)));
	unsigned srvPort;
	unsigned eventPort;
} libposixsrv_common;


int posixsrv_start(void)
{
	if (posixsrv_init(&libposixsrv_common.srvPort, &libposixsrv_common.eventPort) < 0) {
		return -1;
	}

	beginthread(posixsrv_threadRqTimeout, POSIXSRV_PRIO, libposixsrv_common.stackRqTimeout, sizeof(libposixsrv_common.stackRqTimeout), NULL);
	beginthread(posixsrv_threadMain, POSIXSRV_PRIO, libposixsrv_common.stacksMain[0], sizeof(libposixsrv_common.stacksMain[0]), (void *)libposixsrv_common.eventPort);

	for (size_t i = 1; i < sizeof(libposixsrv_common.stacksMain) / sizeof(libposixsrv_common.stacksMain[0]); ++i) {
		beginthread(posixsrv_threadMain, POSIXSRV_PRIO, libposixsrv_common.stacksMain[i], sizeof(libposixsrv_common.stacksMain[i]), (void *)libposixsrv_common.srvPort);
	}

	return 0;
}

#endif
