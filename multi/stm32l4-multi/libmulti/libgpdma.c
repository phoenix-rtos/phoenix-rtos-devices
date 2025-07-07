/*
 * Phoenix-RTOS
 *
 * STM32N6 DMA driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Daniel Sawka, Aleksander Kaminski
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/interrupt.h>
#include <sys/time.h>
#include <sys/pwman.h>
#include <stdbool.h>

#include "../common.h"
#include "libmulti/libdma.h"


#define DMA_NUM_CONTROLLERS 1
#define DMA_NUM_CHANNELS    16

struct {
	volatile unsigned int *base;
	struct {
		handle_t irqLock;
		handle_t cond;
		int taken;
	} channels[DMA_NUM_CHANNELS];
	handle_t takenLock;
} dma_common[DMA_NUM_CONTROLLERS];


int libdma_configurePeripheral(const struct libdma_per *per, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, handle_t *cond)
{
	return -ENOSYS;
}


int libdma_transfer(const struct libdma_per *per, void *rxMAddr, const void *txMAddr, size_t len)
{
	return -ENOSYS;
}


int libdma_tx(const struct libdma_per *per, const void *txMAddr, size_t len, int mode, time_t timeout)
{
	return -ENOSYS;
}


int libdma_rx(const struct libdma_per *per, void *rxMAddr, size_t len, int mode, time_t timeout)
{
	return -ENOSYS;
}


int libdma_txAsync(const struct libdma_per *per, const void *txMAddr, size_t len, volatile int *doneFlag)
{
	return -ENOSYS;
}


int libdma_rxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, volatile int *doneFlag)
{
	return -ENOSYS;
}


int libdma_infiniteRxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, void fn(void *arg, int type), void *arg)
{
	return -ENOSYS;
}


uint16_t libdma_leftToRx(const struct libdma_per *per)
{
	return 0;
}


int libdma_acquirePeripheral(int per, unsigned int num, const struct libdma_per **perP)
{
	return -ENOSYS;
}


int libdma_init(void)
{
	return -ENOSYS;
}
