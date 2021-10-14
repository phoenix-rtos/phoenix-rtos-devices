/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL SDMA lib
 *
 * Copyright 2018 Phoenix Systems
 * Author: Krystian Wasik
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef IMX6ULL_SDMA_LIB_H
#define IMX6ULL_SDMA_LIB_H

#include "sdma-api.h"

typedef struct {
	oid_t oid;
} sdma_t;

int sdma_open(sdma_t *s, const char *dev_name);
int sdma_close(sdma_t *s);

int sdma_channel_configure(sdma_t *s, sdma_channel_config_t *cfg);

int sdma_data_mem_write(sdma_t *s, void *data, size_t size, addr_t addr);
int sdma_data_mem_read(sdma_t *s, void *data, size_t size, addr_t addr);

int sdma_context_dump(sdma_t *s, sdma_context_t *ctx);
int sdma_context_set(sdma_t *s, const sdma_context_t *ctx);

int sdma_enable(sdma_t *s);
int sdma_disable(sdma_t *s);
int sdma_trigger(sdma_t *s);

/* cnt - number of interrupts for this channel registered up until this point */
int sdma_wait_for_intr(sdma_t *s, uint32_t *cnt);

void *sdma_alloc_uncached(sdma_t *s, size_t size, addr_t *paddr, int ocram);
int sdma_free_uncached(void *vaddr, size_t size);

#endif /* IMX6ULL_SDMA_LIB_H */
