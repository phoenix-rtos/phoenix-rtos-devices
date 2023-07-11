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


#define DMA_MAX_LEN ((1u << 16) - 1u)


/* clang-format off */
enum { dma_per2mem = 0, dma_mem2per };


enum { dma_spi = 0, dma_uart };


enum { dma_ht = 0, dma_tc };
/* clang-format on */


struct libdma_per;


/* If cond == NULL only sync function maybe used, otherwise only async functions. */
int libdma_configurePeripheral(const struct libdma_per *per, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, handle_t *cond);


/* SYNC FUNCTIONS */


int libdma_transfer(const struct libdma_per *per, void *rx_maddr, const void *tx_maddr, size_t len);


/* ASYNC FUNCTIONS */


/* Only one tx request per channel may be issued at a time. */
int libdma_txAsync(const struct libdma_per *per, const void *tx_maddr, size_t len, volatile int *done_flag);


/* Only one rx request per channel may be issued at a time. */
int libdma_rxAsync(const struct libdma_per *per, void *rx_maddr, size_t len, volatile int *done_flag);


/* Receive infinite rx into circular buffer. Fn is called at each Transfer Complete and Half Transfer interrupt. */
int libdma_infiniteRxAsync(const struct libdma_per *per, void *rx_maddr, size_t len, void fn(void *arg, int type), void *arg);


/* UTILITY FUNCTIONS */


uint16_t libdma_leftToRx(const struct libdma_per *per);


const struct libdma_per *libdma_getPeripheral(int per, unsigned int num);


int libdma_init(void);


#endif
