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


#include "spw-trace.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

volatile trace_entry_t traceBuf[TRACE_BUF_SIZE];
volatile uint32_t traceIdx = 0;

void spw_dumpTrace(void)
{
    uint32_t count = traceIdx;
    if (count > TRACE_BUF_SIZE) {
        count = TRACE_BUF_SIZE;
    }

    if (count == 0) {
        printf("TRACE: no events \n");
        return;
    }

    trace_entry_t sorted[TRACE_BUF_SIZE];
    for (uint32_t i = 0; i < count; i++) {
        sorted[i].cycle = traceBuf[i].cycle;
        sorted[i].tag = traceBuf[i].tag;
        sorted[i].val = traceBuf[i].val;
    }

    for (uint32_t i = 0; i + 1 < count; i++) {
        for (uint32_t j = 0; j + 1 < count - i; j++) {
            if (sorted[j].cycle > sorted[j + 1].cycle) {
                trace_entry_t tmp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = tmp;
            }
        }
    }

    printf("\n=== TRACE DUMP (%u / %u events, 1 cycle = 12.5 ns @ 80 MHz) ===\n", count, traceIdx);
    uint64_t baseCycle = sorted[0].cycle;
    uint64_t prevCycle = baseCycle;
    uint64_t startCycle = 0;
    uint64_t irq13Cycle = 0;

    for (uint32_t i = 0; i < count; i++) {
        uint64_t rel = sorted[i].cycle - baseCycle;
        uint64_t delta = sorted[i].cycle - prevCycle;
        printf("[%3u] +%9" PRIu64 " cyc (+%6" PRIu64 " cyc / %6.2f us) | %-15s | val=%u\n",
               i, rel, delta, (double)delta / 80.0, sorted[i].tag, sorted[i].val);
        prevCycle = sorted[i].cycle;

        if ((startCycle == 0) && (strcmp(sorted[i].tag, "transmission start") == 0)) {
            startCycle = sorted[i].cycle;
        }
        else if ((strcmp(sorted[i].tag, "irq_dma start") == 0) && (sorted[i].val == 13)) {
            irq13Cycle = sorted[i].cycle;
        }
    }

    if ((startCycle != 0) && (irq13Cycle >= startCycle)) {
        uint64_t diff = irq13Cycle - startCycle;
		printf(" \n");
        printf("transmission start -> irq13: %" PRIu64 " cyc (%.2f us)\n", diff, (double)diff / 80.0);
		printf(" \n");
    }

    printf("=== END OF TRACE DUMP ===\n\n");
}

void spw_resetTrace(void)
{
    __atomic_store_n(&traceIdx, 0, __ATOMIC_SEQ_CST);
}