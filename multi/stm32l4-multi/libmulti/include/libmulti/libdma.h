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


enum libdma_peripheral_type { dma_spi = 0, dma_uart, dma_tim_upd };


enum libdma_interrupt_flags { dma_ht = (1 << 0), dma_tc = (1 << 1) };


enum libdma_transfer_priority {dma_priorityLow = 0, dma_priorityMedium, dma_priorityHigh, dma_priorityVeryHigh };


enum { dma_modeNormal = 0, dma_modeNoBlock };
/* clang-format on */


typedef void libdma_callback_t(void *arg, int type);


#define LIBXPDMA_ACQUIRE_FORCE_HPDMA (1 << 0) /* Force acquire channels on a HPDMA controller */
#define LIBXPDMA_ACQUIRE_FORCE_GPDMA (1 << 1) /* Force acquire channels on a GPDMA controller */
#define LIBXPDMA_ACQUIRE_2D_PER2MEM  (1 << 2) /* Acquire a 2D DMA channel for per2mem direction */
#define LIBXPDMA_ACQUIRE_2D_MEM2PER  (1 << 3) /* Acquire a 2D DMA channel for mem2per direction */


#define LIBXPDMA_TRANSFORM_ALIGNR0 (0 << 1) /* Right-align data, zero-extend if necessary */
#define LIBXPDMA_TRANSFORM_ALIGNRS (1 << 1) /* Right-align data, sign-extend if necessary */
#define LIBXPDMA_TRANSFORM_PACK    (2 << 1) /* Treat data as packed and output at destination width */
#define LIBXPDMA_TRANSFORM_ALIGNL  (3 << 1) /* Left-align data */
#define LIBXPDMA_TRANSFORM_SSWAPB  (1 << 3) /* Swap middle bytes in word in source (e.g. 0xAABBCCDD => 0xAACCBBDD) */
#define LIBXPDMA_TRANSFORM_SWAPB   (1 << 4) /* Swap bytes in destination (e.g. 0xAABBCCDD => 0xDDCCBBAA) */
#define LIBXPDMA_TRANSFORM_SWAPH   (1 << 5) /* Swap half-words in destination (e.g. 0xAABBCCDD => 0xCCDDBBAA)*/
#define LIBXPDMA_TRANSFORM_SWAPW   (1 << 6) /* Swap words in destination (for 64-bit destinations only) */


typedef struct {
	void *buf;          /* Pointer to buffer in memory */
	size_t bufSize;     /* Size of buffer */
	uint8_t elSize_log; /* log2 of size of each element in bytes */
	uint8_t burstSize;  /* Number of elements in transfer burst */
	uint8_t increment;  /* Increment buffer pointer after each burst (1 - true, 0 - false) */
	uint8_t isCached;   /* Buffer is in cached memory */
	uint16_t transform; /* Bitfield of LIBXPDMA_TRANSFORM_* flags */
} libdma_transfer_buffer_t;


typedef struct {
	void *addr;         /* Pointer to peripheral's memory */
	uint8_t elSize_log; /* log2 of size of each element in bytes */
	uint8_t burstSize;  /* Number of elements in transfer burst */
	uint8_t increment;  /* Increment peripheral pointer after each burst (1 - true, 0 - false) */
} libdma_peripheral_config_t;


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


/* Reserve DMA channels for the given peripheral.
 * * per - one of `enum libdma_peripheral_type`
 * * num - number of peripheral
 * * flags - bitfield of `LIBXPDMA_ACQUIRE_*`
 * * perP - pointer to where the peripheral's handle will be stored.
 * Returns -EINVAL on invalid combination of per/num, -EBUSY if requested channels are not available
 */
int libxpdma_acquirePeripheral(int per, unsigned int num, uint32_t flags, const struct libdma_per **perP);


/* Configure DMA channel for use
 * * per - peripheral's handle
 * * dir - direction of transfer
 * * priority - priority of transfers on the channel, one of `libdma_transfer_priority`.
 * * cond - pointer to conditional that will be signaled upon interrupt.
 *   If NULL, waiting for transfer condition can be handled by the driver.
 *   Functions `libxpdma_startTransferWithFlag` and `libxpdma_waitForTransaction` will be usable.
 *   Otherwise, the user will be responsible for synchronization.
 *   `libxpdma_startTransferWithFlag` and `libxpdma_waitForTransaction` cannot be used.
 */
int libxpdma_configureChannel(const struct libdma_per *per, int dir, int priority, handle_t *cond);


/* Configure peripheral side of the transfer */
int libxpdma_configurePeripheral(const struct libdma_per *per, int dir, const libdma_peripheral_config_t *cfg);


/* Configure memory side of the transfer
 * * per - peripheral's handle
 * * dir - direction of transfer
 * * isCircular - create a circular transfer
 * * buffers - pointer to array of buffers
 * * n_buffers - number of elements in `buffers` array
 */
int libxpdma_configureMemory(const struct libdma_per *per, int dir, int isCircular, const libdma_transfer_buffer_t *buffers, size_t n_buffers);


/* Start transaction, notify of progress using a callback
 * * per - peripheral's handle
 * * dir - direction of transfer
 * * intrFlags - bitfield of `enum libdma_interrupt_flags`
 * * cb - callback function
 * * cb_arg - argument for callback function
 */
int libxpdma_startTransferWithCallback(const struct libdma_per *per, int dir, int intrFlags, libdma_callback_t *cb, void *cb_arg);


/* Start transaction, notify of progress using a flag in memory
 * * per - peripheral's handle
 * * dir - direction of transfer
 * * doneFlag - pointer to flag in memory. Will be set to 0 upon entry, != 0 when transfer is complete.
 */
int libxpdma_startTransferWithFlag(const struct libdma_per *per, int dir, volatile int *doneFlag);


/* Wait for transaction to finish. Can wait on both transfer directions at once.
 * If either flag pointer is NULL, the function won't wait on transfer in that direction.
 * * per - peripheral's handle
 * * flagMem2Per - pointer to flag in memory for `dma_per2mem` direction. Can be NULL.
 * * flagPer2Mem - pointer to flag in memory for `dma_mem2per` direction. Can be NULL.
 * * timeout - ms to wait for transfer to finish or 0 to wait indefinitely
 */
int libxpdma_waitForTransaction(const struct libdma_per *per, volatile int *flagMem2Per, volatile int *flagPer2Mem, time_t timeout);


/* Cancel a transfer in progress and reset the channel.
 * Can be used to stop infinite transfer or to clean up in case of an error.
 */
int libxpdma_cancelTransfer(const struct libdma_per *per, int dir);


/* Returns size of data remaining in the current buffer.
 * NOTE: During circular transfers, once a buffer is finished the next one will be loaded.
 * For this reason, once transfer complete interrupt occurs this function will return the size of the buffer instead of 0.
 */
ssize_t libxpdma_bufferRemaining(const struct libdma_per *per, int dir);


/* Allocate DMA-capable (uncached) memory.
 * For now to we assume DMA buffers will be allocated once at the start of the program as needed
 * and for this reason no corresponding `libdma_free` function exists.
 */
void *libdma_malloc(size_t size);

#endif
