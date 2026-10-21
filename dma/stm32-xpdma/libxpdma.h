/*
 * Phoenix-RTOS
 *
 * STM32 xPDMA driver API
 *
 * Copyright 2020-2026 Phoenix Systems
 * Author: Daniel Sawka, Aleksander Kaminski, Jacek Maksymowicz
 *
 * %LICENSE%
 */

#ifndef _LIBXPDMA_H_
#define _LIBXPDMA_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/threads.h>
#include <time.h>


/*
 * The API below contains libdma_* functions and dma_* enums for compatibility with code written for STM32L4 libdma.
 * All of them are fully functional and can be interleaved with libxpdma_* calls **except for libdma_init**.
 * Actual initialization must be performed with libxpdma_init before any calls to libdma_init or any other API calls.
 */


#if defined(__CPU_STM32N6)
#define LIBDMA_NUM_CONTROLLERS 2
/* Channels 12..15 are capable of 2D transfers, others can only do linear transfers */
#define LIBDMA_NUM_CHANNELS    16
#define LIBDMA_2D_CAPABLE_MASK ((1 << 16) - (1 << 12))

#define LIBDMA_HAS_DATA_CACHE 1
#elif defined(__CPU_STM32U3)
#define LIBDMA_NUM_CONTROLLERS 1
/* Channels 10..11 are capable of 2D transfers, others can only do linear transfers */
#define LIBDMA_NUM_CHANNELS    12
#define LIBDMA_2D_CAPABLE_MASK ((1 << 11) | (1 << 10))

#define LIBDMA_HAS_DATA_CACHE 0
#endif

#define DMA_MAX_LEN ((1u << 16) - 1u)

#define LIBDMA_DIRECTIONS 2

enum {
	dma_per2mem = 0,
	dma_mem2per,
};


enum libdma_peripheral_type {
	dma_spi = 0,
	dma_uart,
	dma_tim_upd,
	dma_memTransfer,
	dma_tim_cap1,
	dma_tim_cap2,
	dma_tim_cap3,
	dma_tim_cap4,
};


/* Memory-to-memory transfer is implemented as a fictional "peripheral type" dma_memTransfer.
 * There are two "peripherals" of this type that differ in how the source and destination of the transfer is configured.
 */
enum libdma_memTransfer_num {
	libxpdma_memTransfer_perIsDst = 0, /* libxpdma_configurePeripheral configures destination, use dma_mem2per direction  */
	libxpdma_memTransfer_perIsSrc,     /* libxpdma_configurePeripheral configures source, use dma_per2mem direction */
};


/* clang-format off */
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


/* Functions for communicating with xPDMA manager running in multidriver */
typedef struct {
	/* Retrieve memory parameters */
	int (*getMemoryParams)(void **dmaMemPtr, size_t *dmaMemSz);
	/* Acquire hardware channels for performing DMA transactions */
	int (*acquireChannels)(unsigned int dma, const uint8_t requests[LIBDMA_DIRECTIONS], uint32_t flags, uint8_t chns_out[LIBDMA_DIRECTIONS], uint32_t *listbufs_out[LIBDMA_DIRECTIONS]);
	/* Release a hardware channel after use */
	void (*releaseChannel)(unsigned int dma, uint8_t chn);
	/* Allocate a buffer in DMA-capable memory. May be NULL, in which case malloc() will be used instead. */
	void *(*malloc)(size_t size, size_t alignment);
} libxpdma_mgrFunctions_t;


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


/* Function for backwards compatibility only. Real initialization is performed by `libxpdma_init`. */
int libdma_init(void);


/* Allocate DMA-capable (uncached) memory.
 * For now we assume DMA buffers will be allocated once at the start of the program as needed
 * and for this reason no corresponding `libdma_free` function exists.
 */
void *libdma_malloc(size_t size, size_t alignment);


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


/* Start transaction with no interrupt to notify of progress. Only polling can be used to detect the end of transaction.
 * * per - peripheral's handle
 * * dir - direction of transfer
 */
int libxpdma_startTransferPollingOnly(const struct libdma_per *per, int dir);


/* Wait for transaction to finish. Can wait on both transfer directions at once.
 * If either flag pointer is NULL, the function won't wait on transfer in that direction.
 * * per - peripheral's handle
 * * flagMem2Per - pointer to flag in memory for `dma_per2mem` direction. Can be NULL.
 * * flagPer2Mem - pointer to flag in memory for `dma_mem2per` direction. Can be NULL.
 * * timeout - us to wait for transfer to finish or 0 to wait indefinitely
 */
int libxpdma_waitForTransaction(const struct libdma_per *per, volatile int *flagMem2Per, volatile int *flagPer2Mem, time_t timeout);


/* Wait for transaction to finish using polling.
 * * per - peripheral's handle
 * * dir - direction of transfer
 * * timeout - us to wait for transfer to finish or 0 to wait indefinitely
 */
int libxpdma_waitPoll(const struct libdma_per *per, int dir, time_t timeout);


/* Cancel a transfer in progress and reset the channel.
 * Can be used to stop infinite transfer or to clean up in case of an error.
 */
int libxpdma_cancelTransfer(const struct libdma_per *per, int dir);


/* Returns size of data remaining in the current buffer.
 * NOTE: During circular transfers, once a buffer is finished the next one will be loaded.
 * For this reason, once transfer complete interrupt occurs this function will return the size of the buffer instead of 0.
 */
ssize_t libxpdma_bufferRemaining(const struct libdma_per *per, int dir);


/* Initialize library for use. Pointer to mgrFunctions must remain valid as long as DMA functions are used. */
int libxpdma_init(const libxpdma_mgrFunctions_t *mgrFunctions);


/* Connect to the multidriver at given path which serves as DMA manager.
 * Outputs a pointer to a struct with function pointers that will communicate with DMA manager using messages.
 * Use of this function is optional; user can instead supply custom manager functions to libxpdma_init.
 */
int libxpdma_mgrConnect(const char *path, const libxpdma_mgrFunctions_t **mgrFunctions_out);


#endif
