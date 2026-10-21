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
#include <stdint.h>
#include <sys/threads.h>
#include <time.h>


#define DMA_MAX_LEN ((1u << 16) - 1u)


enum {
	dma_per2mem = 0,
	dma_mem2per,
};


enum libdma_peripheral_type {
	dma_spi = 0,
	dma_uart,
};

/* clang-format off */
enum libdma_interrupt_flags { dma_ht = (1 << 0), dma_tc = (1 << 1) };


enum libdma_transfer_priority {dma_priorityLow = 0, dma_priorityMedium, dma_priorityHigh, dma_priorityVeryHigh };


enum { dma_modeNormal = 0, dma_modeNoBlock };
/* clang-format on */


typedef void libdma_callback_t(void *arg, int type);


/* NOTE: Synchronization on an individual channel must be done externally by the user. */


struct libdma_per;


/* If cond == NULL only sync function may be used, otherwise only async functions. */
int libdma_configurePeripheral(const struct libdma_per *per, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, handle_t *cond);


/* SYNC FUNCTIONS */


int libdma_transfer(const struct libdma_per *per, void *rxMAddr, const void *txMAddr, size_t len);


int libdma_tx(const struct libdma_per *per, const void *txMAddr, size_t len, int mode, time_t timeout);


int libdma_rx(const struct libdma_per *per, void *rxMAddr, size_t len, int mode, time_t timeout);


/* ASYNC FUNCTIONS */


/* Only one tx request per channel may be issued at a time. */
int libdma_txAsync(const struct libdma_per *per, const void *txMAddr, size_t len, volatile int *doneFlag);


/* Only one rx request per channel may be issued at a time. */
int libdma_rxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, volatile int *doneFlag);


/* Receive infinite rx into circular buffer. Fn is called at each Transfer Complete and Half Transfer interrupt. */
int libdma_infiniteRxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, libdma_callback_t *fn, void *arg);


/* UTILITY FUNCTIONS */


uint16_t libdma_leftToRx(const struct libdma_per *per);


/* Return -EINVAL on invalid combination of per/num, -EBUSY if channel user by peripheral is already acquired by other. */
int libdma_acquirePeripheral(int per, unsigned int num, const struct libdma_per **perP);


int libdma_init(void);


/* Allocate DMA-capable (uncached) memory.
 * For now we assume DMA buffers will be allocated once at the start of the program as needed
 * and for this reason no corresponding `libdma_free` function exists.
 */
void *libdma_malloc(size_t size, size_t alignment);

#endif
