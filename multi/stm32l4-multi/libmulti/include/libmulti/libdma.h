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


/* clang-format off */
enum { dma_per2mem = 0, dma_mem2per };


enum { dma_spi = 0, dma_uart, dma_tim_upd };


enum { dma_ht = (1 << 0), dma_tc = (1 << 1) };


enum {dma_priorityLow = 0, dma_priorityMedium, dma_priorityHigh, dma_priorityVeryHigh };


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
int libdma_infiniteRxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, void fn(void *arg, int type), void *arg);


/* UTILITY FUNCTIONS */


uint16_t libdma_leftToRx(const struct libdma_per *per);


/* Return -EINVAL on invalid combination of per/num, -EBUSY if channel user by peripheral is already acquired by other. */
int libdma_acquirePeripheral(int per, unsigned int num, const struct libdma_per **perP);


int libdma_init(void);

typedef struct {
	void *buf;
	size_t bufSize;
	uint8_t elSize;
	uint8_t burstSize;
	uint8_t increment;
	uint8_t isCached;
} dma_transfer_buffer_t;
typedef struct {
	void *addr;
	uint8_t elSize;
	uint8_t burstSize;
	uint8_t increment;
} dma_peripheral_config_t;
int libxpdma_acquirePeripheral(int per, unsigned int num, const struct libdma_per **perP);
int libxpdma_configureChannel(const struct libdma_per *per, int dir, handle_t *cond);
int libxpdma_configurePeripheral(const struct libdma_per *per, int dir, const dma_peripheral_config_t *cfg);
int libxpdma_configureMemory(const struct libdma_per *per, int dir, int isCircular, const dma_transfer_buffer_t *buffers, size_t n_buffers);
void libxpdma_DEBUGPrintTransaction(const struct libdma_per *per, int dir);

#endif
