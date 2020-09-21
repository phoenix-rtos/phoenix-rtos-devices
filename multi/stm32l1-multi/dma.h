/*
 * Phoenix-RTOS
 *
 * STM32L1 DMA driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#ifndef _DMA_H_
#define _DMA_H_

#include <stddef.h>


enum { dma_per2mem = 0, dma_mem2per };


int dma_configure_spi(int num, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc);


int dma_transfer_spi(int num, void *rx_maddr, void *tx_maddr, size_t len);


int dma_init(void);


#endif
