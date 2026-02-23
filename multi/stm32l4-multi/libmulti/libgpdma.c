/*
 * Phoenix-RTOS
 *
 * STM32N6 DMA driver
 *
 * Copyright 2020-2025 Phoenix Systems
 * Author: Daniel Sawka, Aleksander Kaminski, Jacek Maksymowicz
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

#include "../common.h"
#include "../stm32l4-multi.h"
#include "libmulti/libdma.h"

#define DMA_SYSPAGE_MAP_NAME "dmamem"

#define DMA_CTRL_GPDMA1     0
#define DMA_CTRL_HPDMA1     1
#define DMA_NUM_CONTROLLERS 2
/* Channels 12..15 are capable of 2D transfers, others can only do linear transfers */
#define DMA_NUM_CHANNELS    16
#define DMA_2D_CAPABLE_MASK ((1 << 16) - (1 << 12))


enum dma_reqs {
	dma_req_jpeg_rx = 0,
	dma_req_jpeg_tx = 1,
	dma_req_xspi1 = 2,
	dma_req_xspi2 = 3,
	dma_req_xspi3 = 4,
	dma_req_fmc_txrx = 5,
	dma_req_fmc_bch = 6,
	dma_req_adc1 = 7,
	dma_req_adc2 = 8,
	dma_req_cryp_in = 9,
	dma_req_cryp_out = 10,
	dma_req_saes_out = 11,
	dma_req_saes_in = 12,
	dma_req_hash_in = 13,
	dma_req_tim1_cc1 = 14,
	dma_req_tim1_cc2 = 15,
	dma_req_tim1_cc3 = 16,
	dma_req_tim1_cc4 = 17,
	dma_req_tim1_upd = 18,
	dma_req_tim1_trg = 19,
	dma_req_tim1_com = 20,
	dma_req_tim2_cc1 = 21,
	dma_req_tim2_cc2 = 22,
	dma_req_tim2_cc3 = 23,
	dma_req_tim2_cc4 = 24,
	dma_req_tim2_upd = 25,
	dma_req_tim2_trg = 26,
	dma_req_tim3_cc1 = 27,
	dma_req_tim3_cc2 = 28,
	dma_req_tim3_cc3 = 29,
	dma_req_tim3_cc4 = 30,
	dma_req_tim3_upd = 31,
	dma_req_tim3_trg = 32,
	dma_req_tim4_cc1 = 33,
	dma_req_tim4_cc2 = 34,
	dma_req_tim4_cc3 = 35,
	dma_req_tim4_cc4 = 36,
	dma_req_tim4_upd = 37,
	dma_req_tim4_trg = 38,
	dma_req_tim5_cc1 = 39,
	dma_req_tim5_cc2 = 40,
	dma_req_tim5_cc3 = 41,
	dma_req_tim5_cc4 = 42,
	dma_req_tim5_upd = 43,
	dma_req_tim5_trg = 44,
	dma_req_tim6_upd = 45,
	dma_req_tim7_upd = 46,
	dma_req_tim8_cc1 = 47,
	dma_req_tim8_cc2 = 48,
	dma_req_tim8_cc3 = 49,
	dma_req_tim8_cc4 = 50,
	dma_req_tim8_upd = 51,
	dma_req_tim8_trg = 52,
	dma_req_tim8_com = 53,
	dma_req_tim15_cc1 = 56,
	dma_req_tim15_cc2 = 57,
	dma_req_tim15_upd = 58,
	dma_req_tim15_trg = 59,
	dma_req_tim15_com = 60,
	dma_req_tim16_cc1 = 61,
	dma_req_tim16_upd = 62,
	dma_req_tim16_com = 63,
	dma_req_tim17_cc1 = 64,
	dma_req_tim17_upd = 65,
	dma_req_tim17_com = 66,
	dma_req_tim18_cc1 = 67,
	dma_req_tim18_upd = 68,
	dma_req_tim18_com = 69,
	dma_req_lptim1_ic1 = 70,
	dma_req_lptim1_ic2 = 71,
	dma_req_lptim1_ue = 72,
	dma_req_lptim2_ic1 = 73,
	dma_req_lptim2_ic2 = 74,
	dma_req_lptim2_ue = 75,
	dma_req_lptim3_ic1 = 76,
	dma_req_lptim3_ic2 = 77,
	dma_req_lptim3_ue = 78,
	dma_req_spi1_rx = 79,
	dma_req_spi1_tx = 80,
	dma_req_spi2_rx = 81,
	dma_req_spi2_tx = 82,
	dma_req_spi3_rx = 83,
	dma_req_spi3_tx = 84,
	dma_req_spi4_rx = 85,
	dma_req_spi4_tx = 86,
	dma_req_spi5_rx = 87,
	dma_req_spi5_tx = 88,
	dma_req_spi6_rx = 89,
	dma_req_spi6_tx = 90,
	dma_req_sai1_a = 91,
	dma_req_sai1_b = 92,
	dma_req_sai2_a = 93,
	dma_req_sai2_b = 94,
	dma_req_i2c1_rx = 95,
	dma_req_i2c1_tx = 96,
	dma_req_i2c2_rx = 97,
	dma_req_i2c2_tx = 98,
	dma_req_i2c3_rx = 99,
	dma_req_i2c3_tx = 100,
	dma_req_i2c4_rx = 101,
	dma_req_i2c4_tx = 102,
	dma_req_i3c1_rx = 103,
	dma_req_i3c1_tx = 104,
	dma_req_i3c2_rx = 105,
	dma_req_i3c2_tx = 106,
	dma_req_usart1_rx = 107,
	dma_req_usart1_tx = 108,
	dma_req_usart2_rx = 109,
	dma_req_usart2_tx = 110,
	dma_req_usart3_rx = 111,
	dma_req_usart3_tx = 112,
	dma_req_uart4_rx = 113,
	dma_req_uart4_tx = 114,
	dma_req_uart5_rx = 115,
	dma_req_uart5_tx = 116,
	dma_req_usart6_rx = 117,
	dma_req_usart6_tx = 118,
	dma_req_uart7_rx = 119,
	dma_req_uart7_tx = 120,
	dma_req_uart8_rx = 121,
	dma_req_uart8_tx = 122,
	dma_req_uart9_rx = 123,
	dma_req_uart9_tx = 124,
	dma_req_usart10_rx = 125,
	dma_req_usart10_tx = 126,
	dma_req_lpuart1_rx = 127,
	dma_req_lpuart1_tx = 128,
	dma_req_spdifrx_cs = 129,
	dma_req_spdifrx_dt = 130,
	dma_req_adf1_flt0 = 131,
	dma_req_mdf1_flt0 = 132,
	dma_req_mdf1_flt1 = 133,
	dma_req_mdf1_flt2 = 134,
	dma_req_mdf1_flt3 = 135,
	dma_req_mdf1_flt4 = 136,
	dma_req_mdf1_flt5 = 137,
	dma_req_ucpd1_tx = 138,
	dma_req_ucpd1_rx = 139,
	dma_req_dcmi = 140,
	dma_req_i3c1_tc = 141,
	dma_req_i3c1_rs = 142,
	dma_req_i3c2_tc = 143,
	dma_req_i3c2_rs = 144,
	dma_req_sw_trig = 254,
	dma_req_invalid = 255,
};


enum xpdma_regs {
	xpdma_seccfgr = 0x0,
	xpdma_privcfgr,
	xpdma_rcfglockr,
	xpdma_misr,
	xpdma_smisr,
};


#define CHAN_OFFS(chan) (0x14 + ((chan) * 0x20))
enum xpdma_cx_regs {
	xpdma_cxlbar = 0x0,
	hpdma_cxcidcfgr, /* HPDMA instances only */
	hpdma_cxsemcr,   /* HPDMA instances only */
	xpdma_cxfcr,
	xpdma_cxsr,
	xpdma_cxcr,
	xpdma_cxtr1 = 0x10,
	xpdma_cxtr2,
	xpdma_cxbr1,
	xpdma_cxsar,
	xpdma_cxdar,
	xpdma_cxtr3, /* 2D DMA only */
	xpdma_cxbr2, /* 2D DMA only */
	xpdma_cxllr = 0x1f,
};


/* CxTR1 register is slightly weird - the upper 16 bits mostly refer to the destination and lower 16 bits mostly refer to the source.
 * However, bits 29:26 and 13:10 refer to the transfer as a whole (byte exchange, padding and alignment). */
#define XPDMA_CXTR1_INDIVIDUAL_BITS 0xc3ffUL
#define XPDMA_CXTR1_COMMON_BITS     0x3c003c00UL

#define XPDMA_CXTR2_TCEM_BLOCK   (0 << 30) /* Generate TC event on block completion, HT on half block */
#define XPDMA_CXTR2_TCEM_RBLOCK  (1 << 30) /* Generate TC event on repeated block completion, HT on half repeated block */
#define XPDMA_CXTR2_TCEM_EACH_LL (2 << 30) /* Generate TC event on each linked-list item completion, HT on half of each list item */
#define XPDMA_CXTR2_TCEM_LAST_LL (3 << 30) /* Generate TC event on last linked-list item completion, HT on half of last list item */
#define XPDMA_CXTR2_TCEM_MASK    (3 << 30)

#define XPDMA_CXTR2_PF_NORMAL (0 << 12) /* XPDMA_CxBR1.BNDT[15:0] controls block size */
#define XPDMA_CXTR2_PF_CTRL   (1 << 12) /* Peripheral controls block size and can complete transfer early */
#define XPDMA_CXTR2_PER_BURST (0 << 11) /* Hardware requests one burst to be transferred */
#define XPDMA_CXTR2_PER_BLOCK (1 << 11) /* Hardware requests one block to be transferred */
#define XPDMA_CXTR2_PER2MEM   (0 << 10) /* Hardware request signals that data is available from source port */
#define XPDMA_CXTR2_MEM2PER   (1 << 10) /* Hardware request signals that data can be written to destination port */
#define XPDMA_CXTR2_MEM2MEM   (1 << 9)  /* Channel is software requested, ignore hardware requests */

#define XPDMA_TOF      (1 << 14) /* Trigger overflow */
#define XPDMA_SUSPF    (1 << 13) /* Channel suspended */
#define XPDMA_USEF     (1 << 12) /* User setting error */
#define XPDMA_ULEF     (1 << 11) /* Bus error when updating from linked-list */
#define XPDMA_DTEF     (1 << 10) /* Bus error when transferring data */
#define XPDMA_HTF      (1 << 9)  /* Half-transfer complete */
#define XPDMA_TCF      (1 << 8)  /* Transfer complete */
#define XPDMA_ALLFLAGS ((XPDMA_TOF << 1) - XPDMA_TCF)
#define XPDMA_ERRFLAGS (XPDMA_USEF | XPDMA_ULEF | XPDMA_DTEF)
#define XPDMA_IDLEF    (1 << 0) /* Channel idle */

#define XPDMA_CXLLR_UT1 (1 << 31)
#define XPDMA_CXLLR_UT2 (1 << 30)
#define XPDMA_CXLLR_UB1 (1 << 29)
#define XPDMA_CXLLR_USA (1 << 28)
#define XPDMA_CXLLR_UDA (1 << 27)
#define XPDMA_CXLLR_UT3 (1 << 26) /* 2D DMA only */
#define XPDMA_CXLLR_UB2 (1 << 25) /* 2D DMA only */
#define XPDMA_CXLLR_ULL (1 << 16)

#define XPDMA_LISTBUF_SIZE 16 /* Maximum size of linked list in memory in words */


struct libdma_perSetup {
	uint8_t defDMA;                    /* One of DMA_CTRL_* */
	uint8_t requests[dma_mem2per + 1]; /* One of dma_req_* */
	uint8_t portPer;                   /* DMA controller port to use for peripheral communication */
	uint8_t portMem;                   /* DMA controller port to use for memory communication */
	uint8_t valid;
};


struct libdma_per {
	const struct libdma_perSetup *setup;
	uint8_t dma;                   /* Controller selected for the given peripheral */
	uint8_t chns[dma_mem2per + 1]; /* Channel used for a given direction. Value of DMA_NUM_CHANNELS signifies channel is not in use. */
};


/* Note about DMA bus ports:
 * In GPDMA both ports are AHB, so they are equivalent.
 * In HPDMA port 0 is AXI and port 1 is AHB.
 * Only the AXI port of HPDMA can access TCM memories in the CPU.
 */

static const struct libdma_perSetup libdma_persUart[] = {
	[usart1] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_usart1_rx, dma_req_usart1_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[usart2] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_usart2_rx, dma_req_usart2_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[usart3] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_usart3_rx, dma_req_usart3_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[uart4] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_uart4_rx, dma_req_uart4_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[uart5] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_uart5_rx, dma_req_uart5_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[usart6] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_usart6_rx, dma_req_usart6_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[uart7] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_uart7_rx, dma_req_uart7_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[uart8] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_uart8_rx, dma_req_uart8_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[uart9] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_uart9_rx, dma_req_uart9_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[usart10] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_usart10_rx, dma_req_usart10_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
};


static const struct libdma_perSetup libdma_persSpi[] = {
	[spi1] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_spi1_rx, dma_req_spi1_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[spi2] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_spi2_rx, dma_req_spi2_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[spi3] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_spi3_rx, dma_req_spi3_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[spi4] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_spi4_rx, dma_req_spi4_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[spi5] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_spi5_rx, dma_req_spi5_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	[spi6] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_spi6_rx, dma_req_spi6_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
};


static const struct libdma_perSetup libdma_persTimUpd[] = {
	[pwm_tim1] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim1_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim2] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim2_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim3] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim3_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim4] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim4_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim5] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim5_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim8] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim8_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim15] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim15_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim16] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim16_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
	[pwm_tim17] = { .defDMA = DMA_CTRL_GPDMA1, .requests = { dma_req_invalid, dma_req_tim17_upd }, .portPer = 1, .portMem = 0, .valid = 1 },
};


static const struct libdma_perSetup libdma_persMemTransfer[] = {
	[memTransfer_perIsDst] = { .defDMA = DMA_CTRL_HPDMA1, .requests = { dma_req_invalid, dma_req_sw_trig }, .portPer = 0, .portMem = 0, .valid = 1 },
	[memTransfer_perIsSrc] = { .defDMA = DMA_CTRL_HPDMA1, .requests = { dma_req_sw_trig, dma_req_invalid }, .portPer = 0, .portMem = 0, .valid = 1 },
};


typedef struct {
	uint32_t tr1;
	uint32_t tr2;
	uint32_t br1;
	uint32_t sar;
	uint32_t dar;
	uint32_t llr;
} libdma_chn_setup_t;


/* Type of buffer used for making linked lists, will be allocated in non-cached memory */
typedef uint32_t libdma_listbufs_t[DMA_NUM_CONTROLLERS][DMA_NUM_CHANNELS][XPDMA_LISTBUF_SIZE];


#define MAX_BUFS_PER_TRANSACTION 8
typedef struct {
	const void *buf;
	ssize_t dirSize; /* 0 => don't perform operation (end of list), < 0 => perform per2mem, > 0 => perform mem2per */
} libdma_cacheOp_t;


static const struct dma_setup {
	volatile uint32_t *base;
	int pctl;
	bool isHPDMA;
} dma_setup[DMA_NUM_CONTROLLERS] = {
	[DMA_CTRL_GPDMA1] = {
		.base = GPDMA_BASE,
		.pctl = pctl_gpdma1,
		.isHPDMA = false,
	},
	[DMA_CTRL_HPDMA1] = {
		.base = HPDMA_BASE,
		.pctl = pctl_hpdma1,
		.isHPDMA = true,
	},
};


static struct dma_ctrl {
	struct dma_channel {
		handle_t irqLock;
		handle_t cond;
		handle_t intr;
		libdma_chn_setup_t cx;
		libdma_callback_t *cb;
		void *cb_arg;
		libdma_cacheOp_t cacheOps[MAX_BUFS_PER_TRANSACTION];
		uint8_t priority;
		/* libxpdma_* API functions configure the memory data size and increment at a different point than
		 * older libdma_* functions, so we need to store them in the driver */
		struct {
			uint8_t msize;
			uint8_t minc;
		} oldAPI;
	} chns[DMA_NUM_CHANNELS];
	struct libdma_per pers[DMA_NUM_CHANNELS]; /* Each peripheral takes at least 1 channel, so we can have at most DMA_NUM_CHANNELS peripherals */
	uint32_t chanFree;                        /* Bitmap of free channels */
	handle_t takenLock;
} dma_ctrl[DMA_NUM_CONTROLLERS];

static struct {
	bool initialized;
	libdma_listbufs_t *listbufs;
	void *dmaMemPtr;
	size_t dmaMemSz;
	handle_t dmaAllocMutex;
	void *dmaAllocPtr;
	size_t dmaAllocSz;
} dma_common;


static void libdma_cleanupTransfer(int dma, int chn);


static inline bool libdma_hasChannelFinished(volatile uint32_t *chnBase)
{
	return (chnBase == NULL) || ((*(chnBase + xpdma_cxsr) & XPDMA_IDLEF) != 0);
}


/* Utility function for verifying arguments and getting the DMA channel */
static bool libxpdma_getDmaAndChannel(const struct libdma_per *per, int dir, int *dma_out, int *chn_out)
{
	if (per == NULL) {
		return false;
	}

	if ((dir != dma_per2mem) && (dir != dma_mem2per)) {
		return false;
	}

	*dma_out = per->dma;
	*chn_out = per->chns[dir];

	if (*chn_out == DMA_NUM_CHANNELS) {
		/* Channel for this direction not allocated */
		return false;
	}

	return true;
}


/* Check if a buffer is inside the DMA-capable memory range */
static bool libxpdma_isInsideDMAMemory(const void *addr, size_t sz)
{
	uintptr_t addr_start = (uintptr_t)addr;
	uintptr_t addr_end = (uintptr_t)addr + sz;
	bool lowerBound = addr_start >= (uintptr_t)dma_common.dmaMemPtr;
	bool noOverflow = addr_end >= addr_start;
	bool upperBound = addr_end <= ((uintptr_t)dma_common.dmaMemPtr + dma_common.dmaMemSz);
	return lowerBound && noOverflow && upperBound;
}


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


static inline volatile uint32_t *libdma_channelBase(int dma, unsigned int chn)
{
	return dma_setup[dma].base + CHAN_OFFS(chn);
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


static void libxpdma_performCacheOps(int dma, int chn)
{
	libdma_cacheOp_t *ops = dma_ctrl[dma].chns[chn].cacheOps;
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


static int libxpdma_addCacheOp(int dma, int chn, const void *buf, size_t size, int dir)
{
	if (size > INT32_MAX) {
		return -EINVAL;
	}

	libdma_cacheOp_t *ops = dma_ctrl[dma].chns[chn].cacheOps;
	ssize_t dirSize = (dir == dma_per2mem) ? -((ssize_t)size) : size;
	for (size_t i = 0; i < MAX_BUFS_PER_TRANSACTION; i++) {
		if (ops[i].buf == buf && ops[i].dirSize == dirSize) {
			return EOK;
		}

		if (ops[i].dirSize == 0) {
			ops[i].buf = buf;
			ops[i].dirSize = dirSize;
			return EOK;
		}
	}

	return -ENOMEM;
}


static void libxpdma_clearCacheOps(int dma, int chn)
{
	memset(dma_ctrl[dma].chns[chn].cacheOps, 0, sizeof(dma_ctrl[dma].chns[chn].cacheOps));
}


static void libdma_setLBAR(int dma, int chn, void *addr)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_dmaLinkBaseAddr;
	pctl.dmaLinkBaseAddr.dev = dma_setup[dma].pctl;
	pctl.dmaLinkBaseAddr.channel = chn;
	pctl.dmaLinkBaseAddr.addr = ((uint32_t)addr) & 0xffff0000;
	platformctl(&pctl);
}


static int libdma_irqHandler(unsigned int n, void *arg)
{
	unsigned int dma, chn;
	if (!libdma_irqToChannel(n, &dma, &chn)) {
		return -1;
	}

	volatile uint32_t *chnBase = libdma_channelBase(dma, chn);
	struct dma_channel *sChn = &dma_ctrl[dma].chns[chn];
	uint32_t flags = *(chnBase + xpdma_cxsr) & XPDMA_ALLFLAGS;
	*(chnBase + xpdma_cxfcr) = flags;
	uint8_t handlerFlags = 0;
	if ((flags & XPDMA_HTF) != 0) {
		handlerFlags |= dma_ht;
	}

	if ((flags & XPDMA_TCF) != 0) {
		handlerFlags |= dma_tc;
	}

	if (sChn->cb != NULL) {
		sChn->cb(sChn->cb_arg, handlerFlags);
	}

	/* Check the channel enabled flag to determine if transfer is finished and needs to be cleaned up.
	 * For infinite transfers we set the controller to report TCF after each linked list element,
	 * so despite reporting TCF the channel enabled flag will stay set.
	 * For single transfers, once the transfer is finished TCF will be raised and channel enabled flag
	 * will be cleared by hardware. */
	if ((*(chnBase + xpdma_cxcr) & 1) == 0) {
		libdma_cleanupTransfer(dma, chn);
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
static void libdma_cleanupTransfer(int dma, int chn)
{
	volatile uint32_t *chnBase = libdma_channelBase(dma, chn);

	dataBarier();
	/* Disable DMA interrupts */
	*(chnBase + xpdma_cxcr) &= ~XPDMA_ALLFLAGS;
	dataBarier();
	if (*(chnBase + xpdma_cxcr) & 1) {
		/* Channel is still enabled; need to suspend and reset it before it can be used again */
		*(chnBase + xpdma_cxcr) |= (1 << 2); /* Set SUSP flag */
		dataBarier();
		while ((*(chnBase + xpdma_cxcr) & (1 << 2)) == 0) {
			/* Wait for hardware to suspend the channel */
		}

		*(chnBase + xpdma_cxcr) |= (1 << 1); /* Set RESET flag */
		dataBarier();
	}

	dma_ctrl[dma].chns[chn].cb = NULL;
}


static int libxpdma_findChannel(uint8_t request, int dir, uint32_t *chanFreePtr, uint32_t flags)
{
	if (request == dma_req_invalid) {
		return DMA_NUM_CHANNELS;
	}

	uint32_t chanFreeMasked = *chanFreePtr;
	bool mask2D = false;
	if ((dir == dma_per2mem) && ((flags & LIBXPDMA_ACQUIRE_2D_PER2MEM) != 0)) {
		mask2D = true;
	}
	else if ((dir == dma_mem2per) && ((flags & LIBXPDMA_ACQUIRE_2D_MEM2PER) != 0)) {
		mask2D = true;
	}

	if (mask2D) {
		chanFreeMasked &= DMA_2D_CAPABLE_MASK | (~((1u << DMA_NUM_CHANNELS) - 1));
	}

	int chn = __builtin_ctz(chanFreeMasked);
	if (chn >= DMA_NUM_CHANNELS) {
		return -EBUSY;
	}

	*chanFreePtr &= ~(1u << chn);
	return chn;
}


int libxpdma_acquirePeripheral(int per, unsigned int num, uint32_t flags, const struct libdma_per **perP)
{
	const struct libdma_perSetup *setups;
	size_t setupsSize;
	if (per == dma_spi) {
		setupsSize = NELEMS(libdma_persSpi);
		setups = libdma_persSpi;
	}
	else if (per == dma_uart) {
		setupsSize = NELEMS(libdma_persUart);
		setups = libdma_persUart;
	}
	else if (per == dma_tim_upd) {
		setupsSize = NELEMS(libdma_persTimUpd);
		setups = libdma_persTimUpd;
	}
	else if (per == dma_memTransfer) {
		setupsSize = NELEMS(libdma_persMemTransfer);
		setups = libdma_persMemTransfer;
	}
	else {
		return -EINVAL;
	}

	if ((num >= setupsSize) || (setups[num].valid == 0)) {
		return -EINVAL;
	}

	const struct libdma_perSetup *setup = &setups[num];
	uint32_t dma = setup->defDMA;
	if ((flags & (LIBXPDMA_ACQUIRE_FORCE_HPDMA | LIBXPDMA_ACQUIRE_FORCE_GPDMA)) != 0) {
		bool findHPDMA = (flags & LIBXPDMA_ACQUIRE_FORCE_HPDMA) != 0;
		for (dma = 0; dma < DMA_NUM_CONTROLLERS; dma++) {
			if (dma_setup[dma].isHPDMA == findHPDMA) {
				break;
			}
		}

		if (dma >= DMA_NUM_CONTROLLERS) {
			return -EINVAL;
		}
	}

	struct dma_ctrl *ctrl = &dma_ctrl[dma];
	mutexLock(ctrl->takenLock);

	struct libdma_per *p = NULL;
	/* Find a free slot for peripheral information */
	for (int i = 0; i < sizeof(ctrl->pers) / sizeof(ctrl->pers[0]); i++) {
		if (ctrl->pers[i].setup == NULL) {
			p = &ctrl->pers[i];
			break;
		}
	}

	if (p == NULL) {
		mutexUnlock(ctrl->takenLock);
		return -ENOMEM;
	}

	/* Find enough unused channels necessary for peripheral */
	uint32_t chanFree = ctrl->chanFree;
	for (int dir = dma_per2mem; dir <= dma_mem2per; dir++) {
		int acqResult = libxpdma_findChannel(setup->requests[dir], dir, &chanFree, flags);
		if (acqResult < 0) {
			mutexUnlock(ctrl->takenLock);
			return acqResult;
		}

		p->chns[dir] = acqResult;
	}

	ctrl->chanFree = chanFree;
	p->setup = setup;
	p->dma = dma;
	mutexUnlock(ctrl->takenLock);

	*perP = p;
	return EOK;
}


int libxpdma_configureChannel(const struct libdma_per *per, int dir, int priority, handle_t *cond)
{
	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
		return -EINVAL;
	}

	struct dma_channel *sChn = &dma_ctrl[dma].chns[chn];
	sChn->priority = priority;
	if (sChn->intr == 0) {
		handle_t interruptCond = (cond == NULL) ? sChn->cond : *cond;
		interrupt(libdma_channelToIRQ(dma, chn), libdma_irqHandler, NULL, interruptCond, &sChn->intr);
	}

	sChn->cx.br1 &= 0xffff; /* Zero out upper bits of CxBR1 */

	return EOK;
}


int libxpdma_configurePeripheral(const struct libdma_per *per, int dir, const libdma_peripheral_config_t *cfg)
{
	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
		return -EINVAL;
	}

	const uint8_t max_elSize_log = dma_setup[dma].isHPDMA ? 3 : 2;
	if (cfg->elSize_log > max_elSize_log) {
		return -EINVAL;
	}

	if ((cfg->burstSize == 0) || (cfg->burstSize > 64)) {
		return -EINVAL;
	}

	uint32_t shift = (dir == dma_mem2per) ? 16 : 0;
	struct dma_channel *sChn = &dma_ctrl[dma].chns[chn];
	/* The other half of the register will be configured by libxpdma_configureMemory */
	sChn->cx.tr1 &= ~(XPDMA_CXTR1_INDIVIDUAL_BITS << shift);
	uint32_t cxtr1_temp =
			(1u << 15) | /* secure transfer */
			((per->setup->portPer & 1) << 14) |
			(((cfg->burstSize - 1) & 0x3f) << 4) |
			((cfg->increment != 0) ? (1 << 3) : 0) |
			((cfg->elSize_log & 0x3) << 0);
	sChn->cx.tr1 |= cxtr1_temp << shift;

	/* TCEM bits will be configured by libxpdma_configureMemory */
	sChn->cx.tr2 &= XPDMA_CXTR2_TCEM_MASK;
	if (per->setup->requests[dir] == dma_req_sw_trig) {
		sChn->cx.tr2 |= XPDMA_CXTR2_MEM2MEM;
	}
	else {
		uint32_t dir_bit = (dir == dma_mem2per) ? XPDMA_CXTR2_MEM2PER : XPDMA_CXTR2_PER2MEM;
		sChn->cx.tr2 |=
				XPDMA_CXTR2_PF_NORMAL |
				XPDMA_CXTR2_PER_BURST |
				dir_bit |
				(per->setup->requests[dir] & 0xff);
	}

	if (dir == dma_mem2per) {
		sChn->cx.dar = (uint32_t)cfg->addr;
	}
	else {
		sChn->cx.sar = (uint32_t)cfg->addr;
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
		int dma,
		int chn,
		const struct libdma_per *per,
		const libdma_transfer_buffer_t *buf,
		int dir,
		libdma_chn_setup_t *setup,
		uint32_t *changeMask_out)
{
	if ((buf->bufSize == 0) || (buf->bufSize > DMA_MAX_LEN)) {
		return -EINVAL;
	}

	const uint8_t max_elSize_log = dma_setup[dma].isHPDMA ? 3 : 2;
	if (buf->elSize_log > max_elSize_log) {
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
			((per->setup->portMem & 1) << 14) |
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

	if ((setup->br1 & 0xffff) != buf->bufSize) {
		setup->br1 = (setup->br1 & 0xffff0000) | (buf->bufSize & 0xffff);
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
	if (buf->isCached != 0) {
		int ret = libxpdma_addCacheOp(dma, chn, buf->buf, buf->bufSize, dir);
		if (ret < 0) {
			return ret;
		}
	}

	return n_changes;
}


static inline uint32_t *libxpdma_getListbuf(int dma, int chn)
{
	return (*dma_common.listbufs)[dma][chn];
}


static int libxpdma_updateLL(int dma, int chn, libdma_chn_setup_t *setup, ssize_t n_changes, uint32_t changeMask, uint32_t **this_ll, size_t *listbuf_offset)
{
	if (n_changes < 0) {
		return n_changes;
	}

	changeMask |= XPDMA_CXLLR_ULL;
	n_changes++;
	size_t i = *listbuf_offset;
	if ((i + (size_t)n_changes) > XPDMA_LISTBUF_SIZE) {
		return -ENOMEM;
	}

	uint32_t *listbuf = libxpdma_getListbuf(dma, chn);
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
	int dma, chn;
	ssize_t ret;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
		return -EINVAL;
	}

	if ((buffers == NULL) || (n_buffers == 0)) {
		return -EINVAL;
	}

	libdma_chn_setup_t lastSetup = dma_ctrl[dma].chns[chn].cx;
	lastSetup.tr2 &= ~XPDMA_CXTR2_TCEM_MASK;
	lastSetup.tr2 |= (isCircular != 0) ? XPDMA_CXTR2_TCEM_EACH_LL : XPDMA_CXTR2_TCEM_LAST_LL;

	libxpdma_clearCacheOps(dma, chn);
	uint32_t unused;
	ret = libxpdma_configureBuffer(dma, chn, per, &buffers[0], dir, &lastSetup, &unused);
	if (ret < 0) {
		return ret;
	}

	lastSetup.llr = 0;
	dma_ctrl[dma].chns[chn].cx = lastSetup;

	uint32_t *this_ll = &dma_ctrl[dma].chns[chn].cx.llr;
	size_t listbuf_offset = 0;
	uint32_t changeMask;
	for (size_t i = 1; i < n_buffers; i++) {
		/* Add linked list element */
		ret = libxpdma_configureBuffer(dma, chn, per, &buffers[i], dir, &lastSetup, &changeMask);
		ret = libxpdma_updateLL(dma, chn, &lastSetup, ret, changeMask, &this_ll, &listbuf_offset);
		if (ret < 0) {
			return ret;
		}
	}

	if (isCircular != 0) {
		ret = libxpdma_configureBuffer(dma, chn, per, &buffers[0], dir, &lastSetup, &changeMask);
		ret = libxpdma_updateLL(dma, chn, &lastSetup, ret, changeMask, &this_ll, &listbuf_offset);
		if (n_buffers > 1) {
			/* Loop back to first element in memory */
			*this_ll = dma_ctrl[dma].chns[chn].cx.llr;
		}
		else {
			/* Optimization - if there is only one buffer, we can skip updating LLR -
			 * updating it would only set it to the same value it had before. */
			dma_ctrl[dma].chns[chn].cx.llr &= ~XPDMA_CXLLR_ULL;
		}

		if (ret < 0) {
			return ret;
		}
	}

	return EOK;
}


static int libxpdma_startTransaction(const struct libdma_per *per, int dir, int intrFlags, libdma_callback_t *cb, void *cb_arg)
{
	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
		return -EINVAL;
	}

	if ((cb != NULL) && (intrFlags == 0)) {
		/* Interrupt callback given but no interrupt condition requested */
		return -EINVAL;
	}

	volatile uint32_t *chn_base = libdma_channelBase(dma, chn);
	/* Channel is currently enabled, cannot reconfigure it now */
	if ((*(chn_base + xpdma_cxcr) & 1) != 0) {
		return -EBUSY;
	}

	libxpdma_performCacheOps(dma, chn);
	struct dma_channel *sChn = &dma_ctrl[dma].chns[chn];
	sChn->cb = cb;
	sChn->cb_arg = cb_arg;

	*(chn_base + xpdma_cxtr1) = dma_ctrl[dma].chns[chn].cx.tr1;
	*(chn_base + xpdma_cxtr2) = dma_ctrl[dma].chns[chn].cx.tr2;
	*(chn_base + xpdma_cxbr1) = dma_ctrl[dma].chns[chn].cx.br1;
	*(chn_base + xpdma_cxsar) = dma_ctrl[dma].chns[chn].cx.sar;
	*(chn_base + xpdma_cxdar) = dma_ctrl[dma].chns[chn].cx.dar;
	*(chn_base + xpdma_cxllr) = dma_ctrl[dma].chns[chn].cx.llr;

	uint32_t v = *(chn_base + xpdma_cxcr);
	v &= ~(0x3 << 22);
	v |= (sChn->priority & 0x3) << 22;
	v &= ~(XPDMA_TCF | XPDMA_HTF);
	v |= ((intrFlags & dma_tc) != 0) ? XPDMA_TCF : 0;
	v |= ((intrFlags & dma_ht) != 0) ? XPDMA_HTF : 0;
	/* Use port 1 to access linked lists (AHB on both HPDMA and GPDMA instances) */
	v |= 1 << 17;
	*(chn_base + xpdma_cxcr) = v;
	dataBarier();
	*(chn_base + xpdma_cxcr) = v | 1;
	dataBarier();
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


static int libxpdma_waitForChannelIntr(int dma, int chn, volatile int *doneFlag, time_t timeout, time_t end)
{
	int ret = EOK;
	struct dma_channel *sChn = &dma_ctrl[dma].chns[chn];
	time_t condTimeout = timeout;
	mutexLock(sChn->irqLock);
	while (*doneFlag == 0) {
		condWait(sChn->cond, sChn->irqLock, condTimeout);
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

	mutexUnlock(sChn->irqLock);

	/* Channel may have been cleaned up by interrupt, in which case this function call won't do anything */
	libdma_cleanupTransfer(dma, chn);
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
	int dmas[2], chns[2];
	if (doM2P != doP2M) {
		/* Waiting on 1 channel */
		int dir = doM2P ? dma_mem2per : dma_per2mem;
		flags[0] = doM2P ? flagMem2Per : flagPer2Mem;
		if (!libxpdma_getDmaAndChannel(per, dir, &dmas[0], &chns[0])) {
			return -EINVAL;
		}

		flags[1] = NULL;
	}
	else if (doP2M) {
		/* Waiting on 2 channels */
		flags[0] = flagPer2Mem;
		if (!libxpdma_getDmaAndChannel(per, dma_per2mem, &dmas[0], &chns[0])) {
			return -EINVAL;
		}

		flags[1] = flagMem2Per;
		if (!libxpdma_getDmaAndChannel(per, dma_mem2per, &dmas[1], &chns[1])) {
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

	int ret = libxpdma_waitForChannelIntr(dmas[0], chns[0], flags[0], timeout, end);
	if ((ret == EOK) && (flags[1] != NULL)) {
		/* Both RX and TX channels given - we already waited for RX, now wait for TX */
		volatile uint32_t *chnBase = libdma_channelBase(dmas[1], chns[1]);
		while (!libdma_hasChannelFinished(chnBase)) {
			/* Wait for the other channel to finish */
			if (timeout != 0) {
				time_t now;
				gettime(&now, NULL);
				if (end <= now) {
					ret = -ETIME;
					break;
				}
			}
		}
	}

	return ret;
}


int libxpdma_cancelTransfer(const struct libdma_per *per, int dir)
{
	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
		return -EINVAL;
	}

	libdma_cleanupTransfer(dma, chn);
	return EOK;
}


ssize_t libxpdma_bufferRemaining(const struct libdma_per *per, int dir)
{
	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
		return -EINVAL;
	}

	volatile uint32_t *chnBase = libdma_channelBase(dma, chn);
	/* Only bottom 16 bits contain data. */
	return *(chnBase + xpdma_cxbr1) & 0xffff;
}


/* Get memory that is DMA capable (non-cached).
 * We can't do this using mmap(), because this is a NOMMU target so the MAP_UNCACHED flag does nothing.
 * Instead, we check the system's memory maps if there's a map with a pre-defined name (`DMA_SYSPAGE_MAP_NAME`).
 */
static int libdma_getDmaMemory(void)
{
	void *dmaMemPtr = NULL;
	size_t dmaMemSz = 0;
	meminfo_t mi;
	mi.page.mapsz = -1;
	mi.entry.kmapsz = -1;
	mi.entry.mapsz = -1;
	mi.maps.mapsz = 0;
	mi.maps.map = NULL;
	meminfo(&mi);

	mi.maps.map = malloc(mi.maps.mapsz * sizeof(mapinfo_t));
	if (mi.maps.map == NULL) {
		return -ENOMEM;
	}

	meminfo(&mi);
	for (int i = 0; i < mi.maps.mapsz; i++) {
		if (strcmp(mi.maps.map[i].name, DMA_SYSPAGE_MAP_NAME) == 0) {
			dmaMemPtr = (void *)mi.maps.map[i].vstart;
			dmaMemSz = mi.maps.map[i].vend - mi.maps.map[i].vstart;
			break;
		}
	}

	free(mi.maps.map);
	if (dmaMemPtr == NULL) {
		return -ENODEV;
	}

	dma_common.dmaMemPtr = dmaMemPtr;
	dma_common.dmaMemSz = dmaMemSz;
	return EOK;
}


void *libdma_malloc(size_t size)
{
	mutexLock(dma_common.dmaAllocMutex);
	if (size > dma_common.dmaAllocSz) {
		mutexUnlock(dma_common.dmaAllocMutex);
		return NULL;
	}

	void *ret = dma_common.dmaAllocPtr;
	dma_common.dmaAllocSz -= size;
	dma_common.dmaAllocPtr += size;
	mutexUnlock(dma_common.dmaAllocMutex);
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
	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
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

	dma_ctrl[dma].chns[chn].oldAPI.minc = minc;
	dma_ctrl[dma].chns[chn].oldAPI.msize = msize;

	return EOK;
}


static int libdma_transferInternal(const struct libdma_per *per, int dir, void *addr, size_t len, int mode, time_t timeout, volatile int *doneFlagPtr)
{
	int ret;
	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dir, &dma, &chn)) {
		return -EINVAL;
	}

	libdma_transfer_buffer_t buf = {
		.buf = addr,
		.bufSize = len,
		.elSize_log = dma_ctrl[dma].chns[chn].oldAPI.msize,
		.burstSize = 1,
		.increment = dma_ctrl[dma].chns[chn].oldAPI.minc,
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

		return len - (*(libdma_channelBase(dma, chn) + xpdma_cxbr1) & 0xffff);
	}

	return (ret < 0) ? ret : len;
}


int libdma_transfer(const struct libdma_per *per, void *rxMAddr, const void *txMAddr, size_t len)
{
	int ret;
	int txDma, txChn;
	if (!libxpdma_getDmaAndChannel(per, dma_mem2per, &txDma, &txChn)) {
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
		buf.elSize_log = dma_ctrl[txDma].chns[txChn].oldAPI.msize;
		buf.burstSize = 1;
		buf.increment = dma_ctrl[txDma].chns[txChn].oldAPI.minc;
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

	int dma, chn;
	if (!libxpdma_getDmaAndChannel(per, dma_per2mem, &dma, &chn)) {
		return -EINVAL;
	}

	libdma_transfer_buffer_t buf = {
		.buf = rxMAddr,
		.bufSize = len,
		.elSize_log = dma_ctrl[dma].chns[chn].oldAPI.msize,
		.burstSize = 1,
		.increment = dma_ctrl[dma].chns[chn].oldAPI.minc,
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
	if (dma_common.initialized) {
		return EOK;
	}

	int ret;
	ret = libdma_getDmaMemory();
	if (ret < 0) {
		return ret;
	}

	if (dma_common.dmaMemSz < sizeof(*dma_common.listbufs)) {
		return -ENOMEM;
	}

	dma_common.listbufs = dma_common.dmaMemPtr;

	mutexCreate(&dma_common.dmaAllocMutex);
	dma_common.dmaAllocPtr = dma_common.dmaMemPtr + sizeof(*dma_common.listbufs);
	dma_common.dmaAllocSz = dma_common.dmaMemSz - sizeof(*dma_common.listbufs);

	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_dmaPermissions;
	pctl.dmaPermissions.secure = 1;
	pctl.dmaPermissions.privileged = -1;
	pctl.dmaPermissions.lock = 0;

	for (size_t dma = 0; dma < DMA_NUM_CONTROLLERS; dma++) {
		devClk(dma_setup[dma].pctl, 1);
		mutexCreate(&dma_ctrl[dma].takenLock);
		pctl.dmaPermissions.dev = dma_setup[dma].pctl;
		for (size_t chn = 0; chn < DMA_NUM_CHANNELS; chn++) {
			pctl.dmaPermissions.channel = chn;
			platformctl(&pctl);
			condCreate(&dma_ctrl[dma].chns[chn].cond);
			mutexCreate(&dma_ctrl[dma].chns[chn].irqLock);
			/* Set base address for channel's linked list buffer */
			libdma_setLBAR(dma, chn, libxpdma_getListbuf(dma, chn));
			*(libdma_channelBase(dma, chn) + xpdma_cxcr) = (1 << 1); /* Reset channel */
		}

		for (size_t per = 0; per < (sizeof(dma_ctrl[dma].pers) / sizeof(dma_ctrl[dma].pers[0])); per++) {
			dma_ctrl[dma].pers[per].setup = NULL;
			dma_ctrl[dma].pers[per].chns[0] = DMA_NUM_CHANNELS;
			dma_ctrl[dma].pers[per].chns[1] = DMA_NUM_CHANNELS;
		}

		dma_ctrl[dma].chanFree = ~0u; /* NOTE: bits after (1 << DMA_NUM_CHANNELS) must always be 1 */
	}

	dma_common.initialized = true;
	return EOK;
}
