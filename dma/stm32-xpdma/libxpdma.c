/*
 * Phoenix-RTOS
 *
 * STM32 xPDMA driver
 *
 * Copyright 2020-2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Daniel Sawka, Aleksander Kaminski, Jacek Maksymowicz, Mateusz Karcz
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/interrupt.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <stm32l4-multi.h>
#include "libxpdma.h"
#include "xpdma_defs.h"

#define NELEMS(x) (sizeof(x) / sizeof(*(x)))


struct libdma_perSetup {
	uint8_t defDMA;                      /* One of DMA_CTRL_* */
	uint8_t requests[LIBDMA_DIRECTIONS]; /* One of dma_req_* */
	uint8_t portPer;                     /* DMA controller port to use for peripheral communication */
	uint8_t portMem;                     /* DMA controller port to use for memory communication */
	uint8_t valid;
};

#if LIBDMA_HAS_DATA_CACHE
#define MAX_BUFS_PER_TRANSACTION 8

typedef struct {
	const void *buf;
	ssize_t dirSize; /* 0 => don't perform operation (end of list), < 0 => perform per2mem, > 0 => perform mem2per */
} libdma_cacheOp_t;
#endif

typedef struct {
	uint32_t tr1;
	uint32_t tr2;
	uint32_t br1;
	uint32_t sar;
	uint32_t dar;
	uint32_t llr;
} libdma_chn_setup_t;

typedef struct {
	handle_t irqLock;
	handle_t cond;
	handle_t intr;
	libdma_chn_setup_t cx;
	libdma_callback_t *cb;
	void *cb_arg;
#if LIBDMA_HAS_DATA_CACHE
	libdma_cacheOp_t cacheOps[MAX_BUFS_PER_TRANSACTION];
#endif
	uint32_t *listbuf;
	uint8_t dma; /* Index of parent DMA controller */
	uint8_t idx; /* Index of hardware DMA channel */
	uint8_t priority;
	/* libxpdma_* API functions configure the memory data size and increment at a different point than
	 * older libdma_* functions, so we need to store them in the driver */
	struct {
		uint8_t msize;
		uint8_t minc;
	} oldAPI;
} libxpdma_channel_t;


struct libdma_per {
	libxpdma_channel_t *sChns[LIBDMA_DIRECTIONS]; /* May be NULL if no channel allocated for selected direction */
	uint8_t reqs[LIBDMA_DIRECTIONS];
	uint8_t portPer;
};


/* Note about DMA bus ports:
 * In GPDMA both ports are AHB, so they are equivalent.
 * In HPDMA port 0 is AXI and port 1 is AHB.
 * Only the AXI port of HPDMA can access TCM memories in the CPU.
 */

static const uint8_t libdma_reqsUartRx[] = {
	[usart1] = dma_req_usart1_rx,
	[usart2] = dma_req_usart2_rx,
	[usart3] = dma_req_usart3_rx,
	[uart4] = dma_req_uart4_rx,
	[uart5] = dma_req_uart5_rx,
#if defined(__CPU_STM32N6)
	[usart6] = dma_req_usart6_rx,
	[uart7] = dma_req_uart7_rx,
	[uart8] = dma_req_uart8_rx,
	[uart9] = dma_req_uart9_rx,
	[usart10] = dma_req_usart10_rx,
#endif
};


static const uint8_t libdma_reqsUartTx[] = {
	[usart1] = dma_req_usart1_tx,
	[usart2] = dma_req_usart2_tx,
	[usart3] = dma_req_usart3_tx,
	[uart4] = dma_req_uart4_tx,
	[uart5] = dma_req_uart5_tx,
#if defined(__CPU_STM32N6)
	[usart6] = dma_req_usart6_tx,
	[uart7] = dma_req_uart7_tx,
	[uart8] = dma_req_uart8_tx,
	[uart9] = dma_req_uart9_tx,
	[usart10] = dma_req_usart10_tx,
#endif
};


static const uint8_t libdma_reqsSpiRx[] = {
	[spi1] = dma_req_spi1_rx,
	[spi2] = dma_req_spi2_rx,
	[spi3] = dma_req_spi3_rx,
	[spi4] = dma_req_spi4_rx,
#if defined(__CPU_STM32N6)
	[spi5] = dma_req_spi5_rx,
	[spi6] = dma_req_spi6_rx,
#endif
};


static const uint8_t libdma_reqsSpiTx[] = {
	[spi1] = dma_req_spi1_tx,
	[spi2] = dma_req_spi2_tx,
	[spi3] = dma_req_spi3_tx,
	[spi4] = dma_req_spi4_tx,
#if defined(__CPU_STM32N6)
	[spi5] = dma_req_spi5_tx,
	[spi6] = dma_req_spi6_tx,
#endif
};


static const uint8_t libdma_reqsTimUpd[] = {
	[pwm_tim1] = dma_req_tim1_upd,
	[pwm_tim2] = dma_req_tim2_upd,
	[pwm_tim3] = dma_req_tim3_upd,
	[pwm_tim4] = dma_req_tim4_upd,
#if defined(__CPU_STM32N6)
	[pwm_tim5] = dma_req_tim5_upd,
#endif
	[pwm_tim6] = dma_req_tim6_upd,
	[pwm_tim7] = dma_req_tim7_upd,
	[pwm_tim8] = dma_req_tim8_upd,
	[pwm_tim9] = dma_req_invalid,
	[pwm_tim10] = dma_req_invalid,
	[pwm_tim11] = dma_req_invalid,
	[pwm_tim12] = dma_req_invalid,
	[pwm_tim13] = dma_req_invalid,
	[pwm_tim14] = dma_req_invalid,
	[pwm_tim15] = dma_req_tim15_upd,
	[pwm_tim16] = dma_req_tim16_upd,
	[pwm_tim17] = dma_req_tim17_upd,
#if defined(__CPU_STM32N6)
	[pwm_tim18] = dma_req_tim18_upd,
#endif
};


static const uint8_t libdma_reqsTimCC1[] = {
	[pwm_tim1] = dma_req_tim1_cc1,
	[pwm_tim2] = dma_req_tim2_cc1,
	[pwm_tim3] = dma_req_tim3_cc1,
	[pwm_tim4] = dma_req_tim4_cc1,
#if defined(__CPU_STM32N6)
	[pwm_tim5] = dma_req_tim5_cc1,
#endif
	[pwm_tim6] = dma_req_invalid,
	[pwm_tim7] = dma_req_invalid,
	[pwm_tim8] = dma_req_tim8_cc1,
	[pwm_tim9] = dma_req_invalid,
	[pwm_tim10] = dma_req_invalid,
	[pwm_tim11] = dma_req_invalid,
	[pwm_tim12] = dma_req_invalid,
	[pwm_tim13] = dma_req_invalid,
	[pwm_tim14] = dma_req_invalid,
	[pwm_tim15] = dma_req_tim15_cc1,
	[pwm_tim16] = dma_req_tim16_cc1,
	[pwm_tim17] = dma_req_tim17_cc1,
#if defined(__CPU_STM32N6)
	[pwm_tim18] = dma_req_tim18_cc1,
#endif
};


static const uint8_t libdma_reqsTimCC2[] = {
	[pwm_tim1] = dma_req_tim1_cc2,
	[pwm_tim2] = dma_req_tim2_cc2,
	[pwm_tim3] = dma_req_tim3_cc2,
	[pwm_tim4] = dma_req_tim4_cc2,
#if defined(__CPU_STM32N6)
	[pwm_tim5] = dma_req_tim5_cc2,
#endif
	[pwm_tim6] = dma_req_invalid,
	[pwm_tim7] = dma_req_invalid,
	[pwm_tim8] = dma_req_tim8_cc2,
	[pwm_tim9] = dma_req_invalid,
	[pwm_tim10] = dma_req_invalid,
	[pwm_tim11] = dma_req_invalid,
	[pwm_tim12] = dma_req_invalid,
	[pwm_tim13] = dma_req_invalid,
	[pwm_tim14] = dma_req_invalid,
#if defined(__CPU_STM32N6)
	[pwm_tim15] = dma_req_tim15_cc2,
#endif
};


static const uint8_t libdma_reqsTimCC3[] = {
	[pwm_tim1] = dma_req_tim1_cc3,
	[pwm_tim2] = dma_req_tim2_cc3,
	[pwm_tim3] = dma_req_tim3_cc3,
	[pwm_tim4] = dma_req_tim4_cc3,
#if defined(__CPU_STM32N6)
	[pwm_tim5] = dma_req_tim5_cc3,
#endif
	[pwm_tim6] = dma_req_invalid,
	[pwm_tim7] = dma_req_invalid,
	[pwm_tim8] = dma_req_tim8_cc3,
};


static const uint8_t libdma_reqsTimCC4[] = {
	[pwm_tim1] = dma_req_tim1_cc4,
	[pwm_tim2] = dma_req_tim2_cc4,
	[pwm_tim3] = dma_req_tim3_cc4,
	[pwm_tim4] = dma_req_tim4_cc4,
#if defined(__CPU_STM32N6)
	[pwm_tim5] = dma_req_tim5_cc4,
#endif
	[pwm_tim6] = dma_req_invalid,
	[pwm_tim7] = dma_req_invalid,
	[pwm_tim8] = dma_req_tim8_cc4,
};


static const uint8_t libdma_memTransferP2M[] = {
	[libxpdma_memTransfer_perIsDst] = dma_req_invalid,
	[libxpdma_memTransfer_perIsSrc] = dma_req_sw_trig,
};


static const uint8_t libdma_memTransferM2P[] = {
	[libxpdma_memTransfer_perIsDst] = dma_req_sw_trig,
	[libxpdma_memTransfer_perIsSrc] = dma_req_invalid,
};

typedef struct {
	const uint8_t *reqsPer2Mem;
	const uint8_t *reqsMem2Per;
	size_t num;
	uint8_t defaultDma;
	uint8_t portPer;
} libdma_perReqLookup_t;


_Static_assert(NELEMS(libdma_reqsSpiRx) == NELEMS(libdma_reqsSpiTx));
_Static_assert(NELEMS(libdma_reqsUartRx) == NELEMS(libdma_reqsUartTx));
_Static_assert(NELEMS(libdma_memTransferP2M) == NELEMS(libdma_memTransferM2P));


static const libdma_perReqLookup_t libdma_reqsLookup[] = {
	[dma_spi] = { libdma_reqsSpiRx, libdma_reqsSpiTx, NELEMS(libdma_reqsSpiRx), DMA_CTRL_GPDMA1, 1 },
	[dma_uart] = { libdma_reqsUartRx, libdma_reqsUartTx, NELEMS(libdma_reqsUartRx), DMA_CTRL_GPDMA1, 1 },
	[dma_tim_upd] = { NULL, libdma_reqsTimUpd, NELEMS(libdma_reqsTimUpd), DMA_CTRL_GPDMA1, 1 },
	[dma_memTransfer] = { libdma_memTransferP2M, libdma_memTransferM2P, NELEMS(libdma_memTransferM2P), DMA_DEFAULT, 0 },
	[dma_tim_cap1] = { libdma_reqsTimCC1, NULL, NELEMS(libdma_reqsTimCC1), DMA_CTRL_GPDMA1, 1 },
	[dma_tim_cap2] = { libdma_reqsTimCC2, NULL, NELEMS(libdma_reqsTimCC2), DMA_CTRL_GPDMA1, 1 },
	[dma_tim_cap3] = { libdma_reqsTimCC3, NULL, NELEMS(libdma_reqsTimCC3), DMA_CTRL_GPDMA1, 1 },
	[dma_tim_cap4] = { libdma_reqsTimCC4, NULL, NELEMS(libdma_reqsTimCC4), DMA_CTRL_GPDMA1, 1 },
};


static struct {
	const libxpdma_mgrFunctions_t *mgrFunctions;
	// libdma_listbufs_t *listbufs;
#if LIBDMA_HAS_DATA_CACHE
	void *dmaMemPtr;
	size_t dmaMemSz;
#endif
} libdma_common = {
	.mgrFunctions = NULL,
};


static inline void dataBarrier(void)
{
	__asm__ volatile("dmb");
}


static void libdma_cleanupTransfer(libxpdma_channel_t *chn);


static inline bool libdma_hasChannelFinished(volatile uint32_t *chnBase)
{
	return (chnBase == NULL) || ((*(chnBase + xpdma_cxsr) & XPDMA_IDLEF) != 0);
}


static libxpdma_channel_t *libxpdma_getVerifyChannel(const struct libdma_per *per, int dir)
{
	if (per == NULL) {
		return NULL;
	}

	if ((dir != dma_per2mem) && (dir != dma_mem2per)) {
		return NULL;
	}

	return per->sChns[dir];
}


#if defined(__CPU_STM32N6)
static inline bool libdma_irqToChannel(int irq, unsigned int *dma, unsigned int *chn)
{
	if ((irq >= gpdma1_ch0_irq) && (irq <= gpdma1_ch15_irq)) {
		*dma = DMA_CTRL_GPDMA1;
		*chn = irq - gpdma1_ch0_irq;
		return true;
	}
	else if ((irq >= hpdma1_ch0_irq) && (irq <= hpdma1_ch15_irq)) {
		*dma = DMA_CTRL_HPDMA1;
		*chn = irq - hpdma1_ch0_irq;
		return true;
	}

	return false;
}


static inline int libdma_channelToIRQ(unsigned int dma, unsigned int chn)
{
	if (dma == DMA_CTRL_GPDMA1) {
		return gpdma1_ch0_irq + chn;
	}
	else if (dma == DMA_CTRL_HPDMA1) {
		return hpdma1_ch0_irq + chn;
	}

	return 0;
}
#elif defined(__CPU_STM32U3)
static inline bool libdma_irqToChannel(int irq, unsigned int *dma, unsigned int *chn)
{
	if ((irq >= gpdma1_ch0_irq) && (irq <= gpdma1_ch7_irq)) {
		*dma = DMA_CTRL_GPDMA1;
		*chn = irq - gpdma1_ch0_irq;
		return true;
	}

	if ((irq >= gpdma1_ch8_irq) && (irq <= gpdma1_ch11_irq)) {
		*dma = DMA_CTRL_GPDMA1;
		*chn = 8 + irq - gpdma1_ch8_irq;
		return true;
	}

	return false;
}


static inline int libdma_channelToIRQ(unsigned int dma, unsigned int chn)
{
	if (dma != DMA_CTRL_GPDMA1) {
		return 0;
	}

	if ((chn >= 0) && (chn <= 7)) {
		return gpdma1_ch0_irq + chn;
	}

	if ((chn >= 8) && (chn <= 11)) {
		return gpdma1_ch8_irq + chn - 8;
	}

	return 0;
}
#endif


#if LIBDMA_HAS_DATA_CACHE
/* Check if a buffer is inside the DMA-capable memory range */
static bool libxpdma_isInsideDMAMemory(const void *addr, size_t sz)
{
	uintptr_t addr_start = (uintptr_t)addr;
	uintptr_t addr_end = (uintptr_t)addr + sz;
	bool lowerBound = addr_start >= (uintptr_t)libdma_common.dmaMemPtr;
	bool noOverflow = addr_end >= addr_start;
	bool upperBound = addr_end <= ((uintptr_t)libdma_common.dmaMemPtr + libdma_common.dmaMemSz);
	return lowerBound && noOverflow && upperBound;
}


static void libdma_cacheOpMem2Per(const void *addr, size_t sz)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_cleanDCache;
	pctl.opDCache.addr = (void *)addr;
	pctl.opDCache.sz = sz;
	platformctl(&pctl);
}


static void libdma_cacheOpPer2Mem(const void *addr, size_t sz)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_cleanInvalDCache;
	pctl.opDCache.addr = (void *)addr;
	pctl.opDCache.sz = sz;
	platformctl(&pctl);
}


static void libxpdma_performCacheOps(libxpdma_channel_t *chn)
{
	libdma_cacheOp_t *ops = chn->cacheOps;
	for (size_t i = 0; i < MAX_BUFS_PER_TRANSACTION; i++) {
		if (ops[i].dirSize < 0) {
			libdma_cacheOpPer2Mem(ops[i].buf, -ops[i].dirSize);
		}
		else if (ops[i].dirSize == 0) {
			break;
		}
		else {
			libdma_cacheOpMem2Per(ops[i].buf, ops[i].dirSize);
		}
	}
}


static int libxpdma_addCacheOp(libxpdma_channel_t *chn, const void *buf, size_t size, int dir)
{
	if (size > INT32_MAX) {
		return -EINVAL;
	}

	libdma_cacheOp_t *ops = chn->cacheOps;
	ssize_t dirSize = (dir == dma_per2mem) ? -((ssize_t)size) : size;
	for (size_t i = 0; i < MAX_BUFS_PER_TRANSACTION; i++) {
		if (ops[i].buf == buf && ops[i].dirSize == dirSize) {
			return EOK;
		}

		if (ops[i].dirSize == 0) {
			ops[i].buf = buf;
			ops[i].dirSize = dirSize;
			if ((i + 1) < MAX_BUFS_PER_TRANSACTION) {
				ops[i + 1].dirSize = 0;
			}

			return EOK;
		}
	}

	return -ENOMEM;
}


static void libxpdma_clearCacheOps(libxpdma_channel_t *chn)
{
	chn->cacheOps[0].dirSize = 0;
}
#else
static bool libxpdma_isInsideDMAMemory(const void *addr, size_t sz)
{
	return true;
}
#endif


static int libdma_irqHandler(unsigned int n, void *arg)
{
	unsigned int dma, idx;
	if (!libdma_irqToChannel(n, &dma, &idx)) {
		return -1;
	}

	libxpdma_channel_t *chn = (libxpdma_channel_t *)arg;
	volatile uint32_t *chnBase = libdma_channelBase(dma, idx);
	uint32_t flags = *(chnBase + xpdma_cxsr) & XPDMA_ALLFLAGS;
	*(chnBase + xpdma_cxfcr) = flags;
	int handlerFlags = 0;
	if ((flags & XPDMA_HTF) != 0) {
		handlerFlags |= dma_ht;
	}

	if ((flags & XPDMA_TCF) != 0) {
		handlerFlags |= dma_tc;
	}

	if (chn->cb != NULL) {
		chn->cb(chn->cb_arg, handlerFlags);
	}

	/* Check the channel enabled flag to determine if transfer is finished and needs to be cleaned up.
	 * For infinite transfers we set the controller to report TCF after each linked list element,
	 * so despite reporting TCF the channel enabled flag will stay set.
	 * For single transfers, once the transfer is finished TCF will be raised and channel enabled flag
	 * will be cleared by hardware. */
	if ((*(chnBase + xpdma_cxcr) & 1) == 0) {
		libdma_cleanupTransfer(chn);
	}

	return 1;
}


static void libdma_transferOnceCallback(void *cb_arg, int type)
{
	volatile int *doneFlag = cb_arg;
	if ((type & dma_tc) != 0) {
		*doneFlag = 1;
	}
}


/* Cancel transfer if a transfer is in progress and reset interrupt-related variables */
static void libdma_cleanupTransfer(libxpdma_channel_t *chn)
{
	volatile uint32_t *chnBase = libdma_channelBase(chn->dma, chn->idx);

	dataBarrier();
	/* Disable DMA interrupts */
	*(chnBase + xpdma_cxcr) &= ~XPDMA_ALLFLAGS;
	dataBarrier();
	if (*(chnBase + xpdma_cxcr) & 1) {
		/* Channel is still enabled; need to suspend and reset it before it can be used again */
		*(chnBase + xpdma_cxcr) |= (1 << 2); /* Set SUSP flag */
		dataBarrier();
		while ((*(chnBase + xpdma_cxcr) & (1 << 2)) == 0) {
			/* Wait for hardware to suspend the channel */
		}

		*(chnBase + xpdma_cxcr) |= (1 << 1); /* Set RESET flag */
		dataBarrier();
	}

	chn->cb = NULL;
}


static int libxpdma_lookupPeripheral(int per, unsigned int num, uint8_t requests[2], uint8_t *defaultDMA, uint8_t *portPer)
{
	if (per >= NELEMS(libdma_reqsLookup)) {
		return -EINVAL;
	}

	const libdma_perReqLookup_t *l = &libdma_reqsLookup[per];
	if (num >= l->num) {
		return -EINVAL;
	}

	requests[dma_per2mem] = (l->reqsPer2Mem == NULL) ? dma_req_invalid : l->reqsPer2Mem[num];
	requests[dma_mem2per] = (l->reqsMem2Per == NULL) ? dma_req_invalid : l->reqsMem2Per[num];
	if ((requests[dma_per2mem] == dma_req_invalid) && (requests[dma_mem2per] == dma_req_invalid)) {
		/* If both requests are invalid, this peripheral/number combination is invalid */
		return -EINVAL;
	}

	*defaultDMA = l->defaultDma;
	*portPer = l->portPer;
	return EOK;
}


static int libxpdma_initChannel(uint8_t dma, uint8_t idx, uint32_t *listbuf, libxpdma_channel_t **sChn_out)
{
	if (idx >= LIBDMA_NUM_CHANNELS) {
		*sChn_out = NULL;
		return 0;
	}

	int ret;
	libxpdma_channel_t *chn = calloc(1, sizeof(libxpdma_channel_t));
	if (chn == NULL) {
		return -ENOMEM;
	}

	chn->dma = dma;
	chn->idx = idx;
	chn->listbuf = listbuf;
	ret = condCreate(&chn->cond);
	if (ret < 0) {
		free(chn);
		return ret;
	}

	ret = mutexCreate(&chn->irqLock);
	if (ret < 0) {
		resourceDestroy(chn->cond);
		free(chn);
		return ret;
	}

	/* Remaning items initialized to 0 by calloc */
	*sChn_out = chn;
	return 0;
}


static void libxpdma_freeChannel(libxpdma_channel_t *chn)
{
	if (chn == NULL) {
		return;
	}

	if (chn->intr != 0) {
		resourceDestroy(chn->intr);
	}

	resourceDestroy(chn->cond);
	resourceDestroy(chn->irqLock);
	free(chn);
}


int libxpdma_acquirePeripheral(int per, unsigned int num, uint32_t flags, const struct libdma_per **perP)
{
	uint8_t requests[LIBDMA_DIRECTIONS];
	uint8_t chns[LIBDMA_DIRECTIONS];
	uint32_t *listbufs[LIBDMA_DIRECTIONS];
	uint8_t dma;
	uint8_t portPer;

	int ret = libxpdma_lookupPeripheral(per, num, requests, &dma, &portPer);
	if (ret < 0) {
		return ret;
	}

	if ((flags & (LIBXPDMA_ACQUIRE_FORCE_HPDMA | LIBXPDMA_ACQUIRE_FORCE_GPDMA)) != 0) {
		bool findHPDMA = (flags & LIBXPDMA_ACQUIRE_FORCE_HPDMA) != 0;
		for (dma = 0; dma < LIBDMA_NUM_CONTROLLERS; dma++) {
			if (dma_setup[dma].isHPDMA == findHPDMA) {
				break;
			}
		}

		if (dma >= LIBDMA_NUM_CONTROLLERS) {
			return -EINVAL;
		}
	}

	struct libdma_per *p = malloc(sizeof(struct libdma_per));
	if (p == NULL) {
		return -ENOMEM;
	}

	ret = libdma_common.mgrFunctions->acquireChannels(dma, requests, flags, chns, listbufs);
	if (ret < 0) {
		free(p);
		return ret;
	}

	p->portPer = portPer;
	for (size_t dir = 0; dir < LIBDMA_DIRECTIONS; dir++) {
		p->reqs[dir] = requests[dir];
		ret = (ret < 0) ? ret : libxpdma_initChannel(dma, chns[dir], listbufs[dir], &p->sChns[dir]);
		if (ret < 0) {
			/* If ret < 0, run through the loop and init all pointers to 0 - resources will be freed later */
			p->sChns[dir] = NULL;
		}
	}

	if (ret < 0) {
		for (size_t dir = 0; dir < LIBDMA_DIRECTIONS; dir++) {
			libxpdma_freeChannel(p->sChns[dir]);
			libdma_common.mgrFunctions->releaseChannel(dma, chns[dir]);
		}

		free(p);
		return ret;
	}

	*perP = p;
	return EOK;
}


int libxpdma_configureChannel(const struct libdma_per *per, int dir, int priority, handle_t *cond)
{
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	chn->priority = priority;
	if (chn->intr == 0) {
		handle_t interruptCond = (cond == NULL) ? chn->cond : *cond;
		interrupt(libdma_channelToIRQ(chn->dma, chn->idx), libdma_irqHandler, (void *)chn, interruptCond, &chn->intr);
	}

	chn->cx.br1 &= 0xffff; /* Zero out upper bits of CxBR1 */

	return EOK;
}


int libxpdma_configurePeripheral(const struct libdma_per *per, int dir, const libdma_peripheral_config_t *cfg)
{
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	const uint8_t max_elSize_log = dma_setup[chn->dma].isHPDMA ? 3 : 2;
	if (cfg->elSize_log > max_elSize_log) {
		return -EINVAL;
	}

	if ((cfg->burstSize == 0) || (cfg->burstSize > 64)) {
		return -EINVAL;
	}

	uint32_t shift = (dir == dma_mem2per) ? 16 : 0;
	/* The other half of the register will be configured by libxpdma_configureMemory */
	chn->cx.tr1 &= ~(XPDMA_CXTR1_INDIVIDUAL_BITS << shift);
	uint32_t cxtr1_temp =
			(1u << 15) | /* secure transfer */
			((per->portPer & 1) << 14) |
			(((cfg->burstSize - 1) & 0x3f) << 4) |
			((cfg->increment != 0) ? (1 << 3) : 0) |
			((cfg->elSize_log & 0x3) << 0);
	chn->cx.tr1 |= cxtr1_temp << shift;

	/* TCEM bits will be configured by libxpdma_configureMemory */
	chn->cx.tr2 &= XPDMA_CXTR2_TCEM_MASK;
	if (per->reqs[dir] == dma_req_sw_trig) {
		chn->cx.tr2 |= XPDMA_CXTR2_MEM2MEM;
	}
	else {
		uint32_t dir_bit = (dir == dma_mem2per) ? XPDMA_CXTR2_MEM2PER : XPDMA_CXTR2_PER2MEM;
		chn->cx.tr2 |=
				XPDMA_CXTR2_PF_NORMAL |
				XPDMA_CXTR2_PER_BURST |
				dir_bit |
				(per->reqs[dir] & 0xff);
	}

	if (dir == dma_mem2per) {
		chn->cx.dar = (uint32_t)cfg->addr;
	}
	else {
		chn->cx.sar = (uint32_t)cfg->addr;
	}

	return EOK;
}


static bool libxpdma_setTransform(uint16_t flags, uint32_t *val)
{
	uint8_t ssize = (*val) & 0x3;
	uint8_t dsize = (*val >> 16) & 0x3;
	uint8_t pamRequest = flags & 0x6;
	*val &= ~0x1C003800;
	if (ssize < dsize) {
		switch (pamRequest) {
			case LIBXPDMA_TRANSFORM_ALIGNR0: *val |= (0 << 11); break;
			case LIBXPDMA_TRANSFORM_ALIGNRS: *val |= (1 << 11); break;
			case LIBXPDMA_TRANSFORM_PACK: *val |= (2 << 11); break;
			case LIBXPDMA_TRANSFORM_ALIGNL: return false;
			default: return false; /* Should never happen */
		}
	}
	else if (ssize > dsize) {
		switch (pamRequest) {
			case LIBXPDMA_TRANSFORM_ALIGNR0: *val |= (0 << 11); break;
			case LIBXPDMA_TRANSFORM_ALIGNRS: *val |= (0 << 11); break;
			case LIBXPDMA_TRANSFORM_PACK: *val |= (2 << 11); break;
			case LIBXPDMA_TRANSFORM_ALIGNL: *val |= (1 << 11); break;
			default: return false; /* Should never happen */
		}
	}

	*val |= ((flags & LIBXPDMA_TRANSFORM_SSWAPB) != 0) ? (1 << 13) : 0;
	*val |= ((flags & LIBXPDMA_TRANSFORM_SWAPB) != 0) ? (1 << 26) : 0;
	*val |= ((flags & LIBXPDMA_TRANSFORM_SWAPH) != 0) ? (1 << 27) : 0;
	*val |= ((flags & LIBXPDMA_TRANSFORM_SWAPW) != 0) ? (1 << 28) : 0;
	return true;
}


static ssize_t libxpdma_configureBuffer(
		libxpdma_channel_t *chn,
		const struct libdma_per *per,
		const libdma_transfer_buffer_t *buf,
		int dir,
		libdma_chn_setup_t *setup,
		uint32_t *changeMask_out)
{
	const uint8_t max_elSize_log = dma_setup[chn->dma].isHPDMA ? 3 : 2;
	if ((buf->bufSize == 0) || ((buf->bufSize & ((1 << buf->elSize_log) - 1)) != 0)) {
		return -EINVAL;
	}

	if (buf->elSize_log > max_elSize_log) {
		return -EINVAL;
	}

	uint32_t bufSizeAtSource = buf->bufSize;
	if (dir == dma_per2mem) {
		/* This is a workaround for an unintuitive behavior of the DMA controller.
		 * Block size stored in BR1 register is always counted at *source* element size.
		 * E.g. if you have a 32-bit peripheral and only want low 16 bits stored in the buffer,
		 * BR1 value will have to be twice the size of destination buffer in bytes. */
		uint32_t perElSizeLog = XPDMA_CXTR1_SRC_EL_SIZE(setup->tr1);
		uint8_t pamRequest = buf->transform & 0x6;
		if ((perElSizeLog != buf->elSize_log) && (pamRequest != LIBXPDMA_TRANSFORM_PACK)) {
			bufSizeAtSource = (buf->bufSize >> buf->elSize_log) << perElSizeLog;
		}
	}

	if (bufSizeAtSource > DMA_MAX_LEN) {
		return -EINVAL;
	}

	if ((buf->burstSize == 0) || (buf->burstSize > 64)) {
		return -EINVAL;
	}

	uint32_t changeMask = 0;
	ssize_t n_changes = 0;
	uint32_t shift = (dir == dma_mem2per) ? 0 : 16;
	/* The other half of the register will be configured by libxpdma_configurePeripheral */
	uint32_t tr1_new = setup->tr1;
	tr1_new &= ~(XPDMA_CXTR1_INDIVIDUAL_BITS << shift);
	uint32_t tr1_bits =
			(1u << 15) | /* secure transfer */
			(0 << 14) |  /* Use port 0 for memory (AHB on GPDMA, AXI on HPDMA) */
			((buf->burstSize - 1) << 4) |
			((buf->increment != 0) ? (1 << 3) : 0) |
			(buf->elSize_log & 0x3);
	tr1_new |= tr1_bits << shift;
	if (!libxpdma_setTransform(buf->transform, &tr1_new)) {
		return -EINVAL;
	}

	if (tr1_new != setup->tr1) {
		setup->tr1 = tr1_new;
		changeMask |= XPDMA_CXLLR_UT1;
		n_changes++;
	}

	if ((setup->br1 & 0xffff) != bufSizeAtSource) {
		setup->br1 = (setup->br1 & 0xffff0000) | (bufSizeAtSource & 0xffff);
		changeMask |= XPDMA_CXLLR_UB1;
		n_changes++;
	}

	if (dir == dma_mem2per) {
		setup->sar = (uint32_t)buf->buf;
		changeMask |= XPDMA_CXLLR_USA;
		n_changes++;
	}
	else {
		setup->dar = (uint32_t)buf->buf;
		changeMask |= XPDMA_CXLLR_UDA;
		n_changes++;
	};

	*changeMask_out = changeMask;
#if LIBDMA_HAS_DATA_CACHE
	if (buf->isCached != 0) {
		int ret = libxpdma_addCacheOp(chn, buf->buf, buf->bufSize, dir);
		if (ret < 0) {
			return ret;
		}
	}
#endif

	return n_changes;
}


static int libxpdma_updateLL(libxpdma_channel_t *chn, libdma_chn_setup_t *setup, ssize_t n_changes, uint32_t changeMask, uint32_t **this_ll, size_t *listbuf_offset)
{
	if (n_changes < 0) {
		return n_changes;
	}

	changeMask |= XPDMA_CXLLR_ULL;
	n_changes++;
	size_t i = *listbuf_offset;
	if ((i + (size_t)n_changes) > XPDMA_LISTBUF_LEN) {
		return -ENOMEM;
	}

	uint32_t *listbuf = chn->listbuf;
	**this_ll = changeMask | (((uint32_t)&listbuf[i]) & 0xffff);
	if ((changeMask & XPDMA_CXLLR_UT1) != 0) {
		listbuf[i] = setup->tr1;
		i++;
	}

	if ((changeMask & XPDMA_CXLLR_UB1) != 0) {
		listbuf[i] = setup->br1;
		i++;
	}

	if ((changeMask & XPDMA_CXLLR_USA) != 0) {
		listbuf[i] = setup->sar;
		i++;
	}

	if ((changeMask & XPDMA_CXLLR_UDA) != 0) {
		listbuf[i] = setup->dar;
		i++;
	}

	if ((changeMask & XPDMA_CXLLR_ULL) != 0) {
		/* If this is the last element, CxLLR value of 0 will finish transfer.
		 * Otherwise, this value of 0 will be replaced later. */
		listbuf[i] = 0;
		*this_ll = &listbuf[i];
		i++;
	}

	*listbuf_offset = i;
	return EOK;
}


int libxpdma_configureMemory(const struct libdma_per *per, int dir, int isCircular, const libdma_transfer_buffer_t *buffers, size_t n_buffers)
{
	ssize_t ret;
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	if ((buffers == NULL) || (n_buffers == 0)) {
		return -EINVAL;
	}

	libdma_chn_setup_t lastSetup = chn->cx;
	lastSetup.tr2 &= ~XPDMA_CXTR2_TCEM_MASK;
	lastSetup.tr2 |= (isCircular != 0) ? XPDMA_CXTR2_TCEM_EACH_LL : XPDMA_CXTR2_TCEM_LAST_LL;

#if LIBDMA_HAS_DATA_CACHE
	libxpdma_clearCacheOps(chn);
#endif
	uint32_t unused;
	ret = libxpdma_configureBuffer(chn, per, &buffers[0], dir, &lastSetup, &unused);
	if (ret < 0) {
		return ret;
	}

	lastSetup.llr = 0;
	chn->cx = lastSetup;

	uint32_t *this_ll = &chn->cx.llr;
	size_t listbuf_offset = 0;
	uint32_t changeMask;
	for (size_t i = 1; i < n_buffers; i++) {
		/* Add linked list element */
		ret = libxpdma_configureBuffer(chn, per, &buffers[i], dir, &lastSetup, &changeMask);
		ret = libxpdma_updateLL(chn, &lastSetup, ret, changeMask, &this_ll, &listbuf_offset);
		if (ret < 0) {
			return ret;
		}
	}

	if (isCircular != 0) {
		ret = libxpdma_configureBuffer(chn, per, &buffers[0], dir, &lastSetup, &changeMask);
		ret = libxpdma_updateLL(chn, &lastSetup, ret, changeMask, &this_ll, &listbuf_offset);
		if (n_buffers > 1) {
			/* Loop back to first element in memory */
			*this_ll = chn->cx.llr;
		}
		else {
			/* Optimization - if there is only one buffer, we can skip updating LLR -
			 * updating it would only set it to the same value it had before. */
			chn->cx.llr &= ~XPDMA_CXLLR_ULL;
		}

		if (ret < 0) {
			return ret;
		}
	}

	return EOK;
}


static int libxpdma_startTransaction(const struct libdma_per *per, int dir, int intrFlags, libdma_callback_t *cb, void *cb_arg)
{
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	if ((cb != NULL) && (intrFlags == 0)) {
		/* Interrupt callback given but no interrupt condition requested */
		return -EINVAL;
	}

	volatile uint32_t *chn_base = libdma_channelBase(chn->dma, chn->idx);
	/* Channel is currently enabled, cannot reconfigure it now */
	if ((*(chn_base + xpdma_cxcr) & 1) != 0) {
		return -EBUSY;
	}

#if LIBDMA_HAS_DATA_CACHE
	libxpdma_performCacheOps(chn);
#endif
	chn->cb = cb;
	chn->cb_arg = cb_arg;

	*(chn_base + xpdma_cxtr1) = chn->cx.tr1;
	*(chn_base + xpdma_cxtr2) = chn->cx.tr2;
	*(chn_base + xpdma_cxbr1) = chn->cx.br1;
	*(chn_base + xpdma_cxsar) = chn->cx.sar;
	*(chn_base + xpdma_cxdar) = chn->cx.dar;
	*(chn_base + xpdma_cxllr) = chn->cx.llr;

	uint32_t v = *(chn_base + xpdma_cxcr);
	v &= ~(0x3 << 22);
	v |= (chn->priority & 0x3) << 22;
	v &= ~(XPDMA_TCF | XPDMA_HTF);
	v |= ((intrFlags & dma_tc) != 0) ? XPDMA_TCF : 0;
	v |= ((intrFlags & dma_ht) != 0) ? XPDMA_HTF : 0;
	/* Use port 1 to access linked lists (AHB on both HPDMA and GPDMA instances) */
	v |= 1 << 17;
	*(chn_base + xpdma_cxcr) = v;
	dataBarrier();
	*(chn_base + xpdma_cxcr) = v | 1;
	dataBarrier();
	return EOK;
}


int libxpdma_startTransferWithCallback(const struct libdma_per *per, int dir, int intrFlags, libdma_callback_t *cb, void *cb_arg)
{
	return libxpdma_startTransaction(per, dir, intrFlags, cb, cb_arg);
}


int libxpdma_startTransferWithFlag(const struct libdma_per *per, int dir, volatile int *doneFlag)
{
	*doneFlag = 0;
	return libxpdma_startTransaction(per, dir, dma_tc, libdma_transferOnceCallback, (void *)doneFlag);
}


int libxpdma_startTransferPollingOnly(const struct libdma_per *per, int dir)
{
	return libxpdma_startTransaction(per, dir, 0, NULL, NULL);
}


static int libxpdma_waitForChannelIntr(libxpdma_channel_t *chn, volatile int *doneFlag, time_t timeout, time_t end)
{
	int ret = EOK;
	time_t condTimeout = timeout;
	mutexLock(chn->irqLock);
	while (*doneFlag == 0) {
		condWait(chn->cond, chn->irqLock, condTimeout);
		if (timeout != 0) {
			time_t now;
			gettime(&now, NULL);
			if (end <= now) {
				ret = -ETIME;
				break;
			}

			condTimeout = end - now;
		}
	}

	mutexUnlock(chn->irqLock);

	/* Channel may have been cleaned up by interrupt, in which case this function call won't do anything */
	libdma_cleanupTransfer(chn);
	return ret;
}


int libxpdma_waitForTransaction(const struct libdma_per *per, volatile int *flagMem2Per, volatile int *flagPer2Mem, time_t timeout)
{
	/* Phoenix-RTOS doesn't allow us to wait on two conditionals at the same time, so we do a little trick:
	 * if we are asked to wait on two channels, we wait on the RX channel's conditional and active poll on the TX channel.
	 * For our use cases this will be OK (no wasted time) because TX channel will usually finish before RX channel,
	 * so after RX conditional is signalled we just check the TX channel once and exit. */

	bool doM2P = flagMem2Per != NULL;
	bool doP2M = flagPer2Mem != NULL;

	volatile int *flags[2];
	libxpdma_channel_t *sChns[2];
	if (doM2P != doP2M) {
		/* Waiting on 1 channel */
		int dir = doM2P ? dma_mem2per : dma_per2mem;
		flags[0] = doM2P ? flagMem2Per : flagPer2Mem;
		sChns[0] = libxpdma_getVerifyChannel(per, dir);
		if (sChns[0] == NULL) {
			return -EINVAL;
		}

		flags[1] = NULL;
	}
	else if (doP2M) {
		/* Waiting on 2 channels */
		flags[0] = flagPer2Mem;
		sChns[0] = libxpdma_getVerifyChannel(per, dma_per2mem);
		if (sChns[0] == NULL) {
			return -EINVAL;
		}

		flags[1] = flagMem2Per;
		sChns[1] = libxpdma_getVerifyChannel(per, dma_mem2per);
		if (sChns[1] == NULL) {
			return -EINVAL;
		}
	}
	else {
		return EOK;
	}

	time_t end = 0;
	if (timeout != 0) {
		time_t now;
		gettime(&now, NULL);
		end = now + timeout;
	}

	int ret = libxpdma_waitForChannelIntr(sChns[0], flags[0], timeout, end);
	if ((ret == EOK) && (flags[1] != NULL)) {
		/* Both RX and TX channels given - we already waited for RX, now wait for TX */
		volatile uint32_t *chnBase = libdma_channelBase(sChns[1]->dma, sChns[1]->idx);
		while (!libdma_hasChannelFinished(chnBase)) {
			/* Wait for the other channel to finish */
			if (timeout != 0) {
				time_t now;
				gettime(&now, NULL);
				if (end <= now) {
					ret = -ETIME;
					libdma_cleanupTransfer(sChns[1]);
					break;
				}
			}
		}
	}

	return ret;
}


int libxpdma_waitPoll(const struct libdma_per *per, int dir, time_t timeout)
{
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	time_t end = 0;
	if (timeout != 0) {
		time_t now;
		gettime(&now, NULL);
		end = now + timeout;
	}

	int ret = EOK;
	volatile uint32_t *chnBase = libdma_channelBase(chn->dma, chn->idx);
	while (!libdma_hasChannelFinished(chnBase)) {
		/* Wait for the channel to finish */
		if (timeout != 0) {
			time_t now;
			gettime(&now, NULL);
			if (end <= now) {
				ret = -ETIME;
				libdma_cleanupTransfer(chn);
				break;
			}
		}
	}

	return ret;
}


int libxpdma_cancelTransfer(const struct libdma_per *per, int dir)
{
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	libdma_cleanupTransfer(chn);
	return EOK;
}


ssize_t libxpdma_bufferRemaining(const struct libdma_per *per, int dir)
{
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	volatile uint32_t *chnBase = libdma_channelBase(chn->dma, chn->idx);
	/* Only bottom 16 bits contain data. */
	uint32_t nBytesSrc = *(chnBase + xpdma_cxbr1) & 0xffff;
	/* Block size stored in BR1 register is always counted at *source* element size.
	 * If source is memory, then it's OK already.
	 * If source is peripheral, it's at different element size and packing is not applied, we need to calculate
	 * the resulting number of bytes in memory. */
	if (dir == dma_mem2per) {
		return (ssize_t)nBytesSrc;
	}

	uint32_t tr1 = *(chnBase + xpdma_cxtr1);
	if (XPDMA_CXTR1_SRC_EL_SIZE(tr1) != XPDMA_CXTR1_DST_EL_SIZE(tr1) && ((tr1 & XPDMA_CXTR1_PAM_MASK) != XPDMA_CXTR1_PAM_PACK)) {
		return (nBytesSrc >> XPDMA_CXTR1_SRC_EL_SIZE(tr1)) << XPDMA_CXTR1_DST_EL_SIZE(tr1);
	}

	return nBytesSrc;
}


void *libdma_malloc(size_t size, size_t alignment)
{
	/* Alignment must be a power of 2 */
	if ((alignment & (alignment - 1)) != 0) {
		return NULL;
	}


	if (libdma_common.mgrFunctions->malloc != NULL) {
		return libdma_common.mgrFunctions->malloc(size, alignment);
	}

	void *ret = malloc(size);
	if (ret == NULL) {
		return NULL;
	}

	if ((alignment != 0) && (((uintptr_t)ret & (alignment - 1)) != 0)) {
		void *alignedRet = realloc(ret, size + alignment);
		if (alignedRet == NULL) {
			free(ret);
			return NULL;
		}

		ret = alignedRet;
		uintptr_t diff = alignment - ((uintptr_t)ret & (alignment - 1));
		if (diff != alignment) {
			ret = (char *)ret + diff;
		}
	}

	/*
	 * Note: because `ret` could be offset from the malloc'ed chunk of memory, it can't be passed directly to free().
	 * Remember this for the future when implementing libdma_free().
	 */
	return ret;
}


/* Below are functions for compatibility with old DMA API */


int libdma_acquirePeripheral(int per, unsigned int num, const struct libdma_per **perP)
{
	return libxpdma_acquirePeripheral(per, num, 0, perP);
}


int libdma_configurePeripheral(
		const struct libdma_per *per,
		int dir,
		int priority,
		void *paddr,
		int msize,
		int psize,
		int minc,
		int pinc,
		handle_t *cond)
{
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	int ret = libxpdma_configureChannel(per, dir, priority, cond);
	if (ret < 0) {
		return ret;
	}

	libdma_peripheral_config_t perCfg = {
		.addr = paddr,
		.elSize_log = psize,
		.burstSize = 1,
		.increment = pinc,
	};
	ret = libxpdma_configurePeripheral(per, dir, &perCfg);
	if (ret < 0) {
		return ret;
	}

	chn->oldAPI.minc = minc;
	chn->oldAPI.msize = msize;

	return EOK;
}


static int libdma_transferInternal(const struct libdma_per *per, int dir, void *addr, size_t len, int mode, time_t timeout, volatile int *doneFlagPtr)
{
	int ret;
	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dir);
	if (chn == NULL) {
		return -EINVAL;
	}

	libdma_transfer_buffer_t buf = {
		.buf = addr,
		.bufSize = len,
		.elSize_log = chn->oldAPI.msize,
		.burstSize = 1,
		.increment = chn->oldAPI.minc,
		/* Because the older APIs don't have a way to state if the buffer is cacheable,
		 * use a heuristic - if it's not in DMA memory, treat it as cacheable. */
		.isCached = libxpdma_isInsideDMAMemory(addr, len) ? 0 : 1,

	};
	ret = libxpdma_configureMemory(per, dir, 0, &buf, 1);
	if (ret < 0) {
		return ret;
	}

	volatile int doneFlag;
	bool doWait = doneFlagPtr == NULL;
	if (doWait) {
		doneFlagPtr = &doneFlag;
	}

	ret = libxpdma_startTransferWithFlag(per, dir, doneFlagPtr);
	if (ret < 0) {
		return ret;
	}

	if (!doWait) {
		return 0;
	}


	if (dir == dma_per2mem) {
		ret = libxpdma_waitForTransaction(per, NULL, &doneFlag, timeout);
	}
	else {
		ret = libxpdma_waitForTransaction(per, &doneFlag, NULL, timeout);
	}

	if (mode == dma_modeNoBlock) {
		if (doneFlag != 0) {
			return len;
		}

		return len - (*(libdma_channelBase(chn->dma, chn->idx) + xpdma_cxbr1) & 0xffff);
	}

	return (ret < 0) ? ret : len;
}


int libdma_transfer(const struct libdma_per *per, void *rxMAddr, const void *txMAddr, size_t len)
{
	int ret;
	libxpdma_channel_t *txChn = libxpdma_getVerifyChannel(per, dma_mem2per);
	if (txChn == NULL) {
		return -EINVAL;
	}

	libdma_transfer_buffer_t buf;
	if (txMAddr == NULL) {
		static uint32_t dummy;
		buf.buf = &dummy;
		buf.bufSize = len;
		buf.elSize_log = 0;
		buf.burstSize = 1;
		buf.increment = 0;
		buf.isCached = 0; /* We don't care what is transmitted, so we can skip cache operations */
	}
	else {
		buf.buf = (void *)txMAddr;
		buf.bufSize = len;
		buf.elSize_log = txChn->oldAPI.msize;
		buf.burstSize = 1;
		buf.increment = txChn->oldAPI.minc;
		buf.isCached = libxpdma_isInsideDMAMemory(txMAddr, len) ? 0 : 1;
	}

	ret = libxpdma_configureMemory(per, dma_mem2per, 0, &buf, 1);
	if (ret < 0) {
		return ret;
	}

	volatile int txDoneFlag, rxDoneFlag;
	if (rxMAddr != NULL) {
		ret = libdma_transferInternal(per, dma_per2mem, rxMAddr, len, dma_modeNormal, 0, &rxDoneFlag);
		if (ret < 0) {
			return ret;
		}
	}

	ret = libxpdma_startTransferWithFlag(per, dma_mem2per, &txDoneFlag);
	if (ret < 0) {
		return ret;
	}

	return libxpdma_waitForTransaction(per, &txDoneFlag, (rxMAddr != NULL) ? &rxDoneFlag : NULL, 0);
}


int libdma_tx(const struct libdma_per *per, const void *txMAddr, size_t len, int mode, time_t timeout)
{
	return libdma_transferInternal(per, dma_mem2per, (void *)txMAddr, len, mode, timeout, NULL);
}


int libdma_rx(const struct libdma_per *per, void *rxMAddr, size_t len, int mode, time_t timeout)
{
	return libdma_transferInternal(per, dma_per2mem, rxMAddr, len, mode, timeout, NULL);
}


int libdma_txAsync(const struct libdma_per *per, const void *txMAddr, size_t len, volatile int *doneFlag)
{
	return libdma_transferInternal(per, dma_mem2per, (void *)txMAddr, len, dma_modeNormal, 0, doneFlag);
}


int libdma_rxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, volatile int *doneFlag)
{
	return libdma_transferInternal(per, dma_per2mem, rxMAddr, len, dma_modeNormal, 0, doneFlag);
}


int libdma_infiniteRxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, void fn(void *arg, int type), void *arg)
{
	if ((rxMAddr == NULL) || (len == 0)) {
		return libxpdma_cancelTransfer(per, dma_per2mem);
	}

	libxpdma_channel_t *chn = libxpdma_getVerifyChannel(per, dma_per2mem);
	if (chn == NULL) {
		return -EINVAL;
	}

	libdma_transfer_buffer_t buf = {
		.buf = rxMAddr,
		.bufSize = len,
		.elSize_log = chn->oldAPI.msize,
		.burstSize = 1,
		.increment = chn->oldAPI.minc,
		.isCached = libxpdma_isInsideDMAMemory(rxMAddr, len) ? 0 : 1,
	};

	int ret = libxpdma_configureMemory(per, dma_per2mem, 1, &buf, 1);
	if (ret < 0) {
		return ret;
	}

	return libxpdma_startTransferWithCallback(per, dma_per2mem, dma_tc | dma_ht, fn, arg);
}


uint16_t libdma_leftToRx(const struct libdma_per *per)
{
	ssize_t ret = libxpdma_bufferRemaining(per, dma_per2mem);
	return (ret < 0) ? 0 : (ret & 0xffff);
}


int libdma_init(void)
{
	return (libdma_common.mgrFunctions == NULL) ? -ENXIO : 0;
}


int libxpdma_init(const libxpdma_mgrFunctions_t *mgrFunctions)
{
	if (mgrFunctions == NULL) {
		return -EINVAL;
	}

	if (mgrFunctions->getMemoryParams == NULL ||
			mgrFunctions->acquireChannels == NULL ||
			mgrFunctions->releaseChannel == NULL) {
		return -EINVAL;
	}

	void *dmaMemPtr;
	size_t dmaMemSz;
	int ret = mgrFunctions->getMemoryParams(&dmaMemPtr, &dmaMemSz);
	if (ret < 0) {
		return ret;
	}

#if LIBDMA_HAS_DATA_CACHE
	libdma_common.dmaMemPtr = dmaMemPtr;
	libdma_common.dmaMemSz = dmaMemSz;
#else
	(void)dmaMemPtr;
	(void)dmaMemSz;
#endif

	libdma_common.mgrFunctions = mgrFunctions;
	return 0;
}
