/*
 * Phoenix-RTOS
 *
 * Tracing utility
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef SPW_TRACE_H
#define SPW_TRACE_H

#include <stdint.h>

#define TRACE_BUF_SIZE 256U

typedef struct {
    uint64_t cycle;
    const char *tag;
    uint32_t val;
} trace_entry_t;

extern volatile trace_entry_t traceBuf[TRACE_BUF_SIZE];
extern volatile uint32_t traceIdx;

static inline uint64_t cpu_cycles(void)
{
    uint64_t c;
    __asm__ volatile("rdcycle %0" : "=r"(c) : : "memory");
    return c;
}

static inline void trace_log(const char *tag, uint32_t val)
{
    uint32_t idx = __atomic_fetch_add(&traceIdx, 1, __ATOMIC_RELAXED) & (TRACE_BUF_SIZE - 1U);
    traceBuf[idx].cycle = cpu_cycles();
    traceBuf[idx].tag = tag;
    traceBuf[idx].val = val;
}

void spw_dumpTrace(void);
void spw_resetTrace(void);

#endif