/*
 * Phoenix-RTOS
 *
 * STM32L4 DMA driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Daniel Sawka, Aleksander Kaminski
 *
 * %LICENSE%
 */

#ifndef LIBDMA_H_
#define LIBDMA_H_

#include <stddef.h>


enum { dma_per2mem = 0, dma_mem2per };


int libdma_configureSpi(int num, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc);


int libdma_transferSpi(int num, void *rx_maddr, const void *tx_maddr, size_t len);


int libdma_init(void);


#endif
