/*
 * Phoenix-RTOS
 *
 * ADE9113 driver DMA for IMX6ULL
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ADE9113_DMA_H
#define ADE9113_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


int dma_init(size_t size, size_t count, bool (*rxCb)(const uint8_t *data, size_t size));


int dma_reset(void);


void dma_free(void);


void dma_enable(void);


void dma_disable(void);


#endif
