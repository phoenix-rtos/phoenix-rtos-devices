/*
 * Phoenix-RTOS
 *
 * Simple tracing helpers
 *
 * Copyright 2024 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MULTI_TRACE_H_
#define _MULTI_TRACE_H_

#include <stdio.h>

#ifdef TRACE_ENABLE
#define TRACE(fmt, ...) printf("%s:" fmt "\n", __FUNCTION__, ##__VA_ARGS__)
#define TRACE_IRQ()     /* TODO: use rtt_write from librtt */
#else
#define TRACE(fmt, ...)
#define TRACE_IRQ()
#endif


#endif /* end of _MULTI_TRACE_H_ */
