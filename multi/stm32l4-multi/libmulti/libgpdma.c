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
#include <stdbool.h>

#include "../common.h"
#include "libmulti/libdma.h"

#define DMA_CTRL_GPDMA1     0
#define DMA_CTRL_HPDMA1     1
#define DMA_NUM_CONTROLLERS 2
/* Channels 12..15 are capable of 2D transfers, others can only do linear transfers */
#define DMA_NUM_CHANNELS 16
/* Length of transfer in bytes from which a transfer is considered "long" and
 * will be carried out with interrupts (otherwise polling will be used).
 * TODO: Value needs to be tested and optimized empirically. */
#define DMA_LONG_TRANSFER_THRESHOLD 24


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


/* Destination Wider than Source */
enum xpdma_padalign_dws {
	xpdma_padalign_dws_right_zero = 0x0, /* Source right-aligned, zero-extended (0xAB => 0x00AB) */
	xpdma_padalign_dws_right_sign = 0x1, /* Source right-aligned, sign-extended (0xAB => 0xFFAB) */
	xpdma_padalign_dws_pack = 0x2,       /* Source read into FIFO, output little-endian at destination width (0xAB, 0xCD => 0xCDAB) */
};

/* Source Wider than Destination */
enum xpdma_padalign_swd {
	xpdma_padalign_swd_right = 0x0,  /* Right-aligned (truncated on the left) (0xABCD => 0xCD) */
	xpdma_padalign_swd_left = 0x1,   /* Left-aligned (truncated on the right) (0xABCD => 0xAB) */
	xpdma_padalign_swd_unpack = 0x2, /* Source read into FIFO, output little-endian at destination width (0xABCD => 0xCD, 0xAB) */
};

#define XPDMA_CXTR1_SINC (1 << 3)
#define XPDMA_CXTR1_DINC (1 << 19)

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

#define DMA_USE_TC_IRQ        (1 << 0)
#define DMA_USE_HT_IRQ        (1 << 1)
#define DMA_CIRCULAR          (1 << 2)
#define DMA_OVERRIDE_SINC_OFF (1 << 3)
#define DMA_OVERRIDE_DINC_OFF (1 << 4)


struct libdma_perSetup {
	uint8_t dma;                       /* One of DMA_CTRL_* */
	uint8_t requests[dma_mem2per + 1]; /* One of dma_req_* */
	uint8_t portPer;                   /* DMA controller port to use for peripheral communication */
	uint8_t portMem;                   /* DMA controller port to use for memory communication */
	uint8_t valid;
};


struct libdma_per {
	const struct libdma_perSetup *setup;
	uint8_t chns[dma_mem2per + 1]; /* Channel used for a given direction. Value of DMA_NUM_CHANNELS signifies channel is not in use. */
};


/* Note about DMA bus ports:
 * In GPDMA both ports are AHB, so they are equivalent.
 * In HPDMA port 0 is AXI and port 1 is AHB.
 * Only the AXI port of HPDMA can access TCM memories in the CPU.
 */

static const struct libdma_perSetup libdma_persUart[] = {
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_usart1_rx, dma_req_usart1_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_usart2_rx, dma_req_usart2_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_usart3_rx, dma_req_usart3_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_uart4_rx, dma_req_uart4_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_uart5_rx, dma_req_uart5_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_usart6_rx, dma_req_usart6_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_uart7_rx, dma_req_uart7_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_uart8_rx, dma_req_uart8_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_uart9_rx, dma_req_uart9_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_usart10_rx, dma_req_usart10_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
};


static const struct libdma_perSetup libdma_persSpi[] = {
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_spi1_rx, dma_req_spi1_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_spi2_rx, dma_req_spi2_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_spi3_rx, dma_req_spi3_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_spi4_rx, dma_req_spi4_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_spi5_rx, dma_req_spi5_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
	{ .dma = DMA_CTRL_GPDMA1, .requests = { dma_req_spi6_rx, dma_req_spi6_tx }, .portPer = 1, .portMem = 0, .valid = 1 },
};


typedef struct {
	uint32_t tr1;
	uint32_t tr2;
	uint32_t br1;
	uint32_t sar;
	uint32_t dar;
	uint32_t llr;
} libdma_chn_setup_t;


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
		libdma_chn_setup_t cx;
		void (*cb)(void *arg, int type);
		void *cb_arg;
		uint8_t priority;
	} chns[DMA_NUM_CHANNELS];
	struct libdma_per pers[DMA_NUM_CHANNELS];
	uint32_t chanFree; /* Bitmap of free channels */
	handle_t takenLock;
} dma_ctrl[DMA_NUM_CONTROLLERS];

static struct {
	bool initialized;
} dma_common;


static void libdma_cleanupTransfer(int dma, int chn);


static inline bool libdma_hasChannelFinished(volatile uint32_t *chnBase)
{
	return (chnBase == NULL) || ((*(chnBase + xpdma_cxsr) & XPDMA_IDLEF) != 0);
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


static void libdma_cacheOpMem2Per(void *addr, size_t sz)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_cleanDCache;
	pctl.opDCache.addr = addr;
	pctl.opDCache.sz = sz;
	platformctl(&pctl);
}


static void libdma_cacheOpPer2Mem(void *addr, size_t sz)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_cleanInvalDCache;
	pctl.opDCache.addr = addr;
	pctl.opDCache.sz = sz;
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


static void libdma_setTransferOnceCallback(int dma, int chn, volatile int *doneFlag)
{
	*doneFlag = 0;
	dma_ctrl[dma].chns[chn].cb = libdma_transferOnceCallback;
	dma_ctrl[dma].chns[chn].cb_arg = (void *)doneFlag;
}


int libdma_acquirePeripheral(int per, unsigned int num, const struct libdma_per **perP)
{
	const struct libdma_perSetup *setups;
	size_t setupsSize;
	if (per == dma_spi) {
		setupsSize = sizeof(libdma_persSpi) / sizeof(libdma_persSpi[0]);
		setups = libdma_persSpi;
	}
	else if (per == dma_uart) {
		setupsSize = sizeof(libdma_persUart) / sizeof(libdma_persUart[0]);
		setups = libdma_persUart;
	}
	else {
		return -EINVAL;
	}

	if ((num >= setupsSize) || (setups[num].valid == 0)) {
		return -EINVAL;
	}

	const struct libdma_perSetup *setup = &setups[num];
	struct dma_ctrl *ctrl = &dma_ctrl[setup->dma];
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
	for (int i = dma_per2mem; i <= dma_mem2per; i++) {
		if (setup->requests[i] == dma_req_invalid) {
			p->chns[i] = DMA_NUM_CHANNELS;
			continue;
		}

		uint8_t chn = __builtin_ctz(chanFree);
		if (chn >= DMA_NUM_CHANNELS) {
			mutexUnlock(ctrl->takenLock);
			return -EBUSY;
		}

		chanFree &= ~(1u << chn);
		p->chns[i] = chn;
	}

	ctrl->chanFree = chanFree;
	p->setup = setup;
	mutexUnlock(ctrl->takenLock);

	*perP = p;
	return EOK;
}


static void libdma_configureChannel(
		int dma,
		int chn,
		int request,
		int dir,
		int priority,
		void *paddr,
		int sSize,
		int dSize,
		int sInc,
		int dInc,
		int sPort,
		int dPort)
{
	struct dma_channel *sChn = &dma_ctrl[dma].chns[chn];
	sChn->priority = priority & 0x3;
	sChn->cx.tr1 =
			/* Destination: */
			(1u << 31) | /* secure transfer */
			((dPort & 0x1) << 30) |
			(0u << 27) | /* no half-word exchange */
			(0u << 26) | /* no byte exchange */
			(0u << 20) | /* burst size = 1 data */
			((dInc & 0x1) << 19) |
			((dSize & 0x3) << 16) |
			/* Source: */
			(1u << 15) | /* secure transfer */
			((sPort & 0x1) << 14) |
			(0u << 13) | /* no byte exchange */
			(0u << 11) | /* padding/alignment - always right-aligned */
			(0u << 4) |  /* burst size = 1 data */
			((sInc & 0x1) << 3) |
			((sSize & 0x3) << 0);
	uint32_t dir_bit = (dir == dma_per2mem) ? XPDMA_CXTR2_PER2MEM : XPDMA_CXTR2_MEM2PER;
	sChn->cx.tr2 =
			XPDMA_CXTR2_TCEM_BLOCK |
			XPDMA_CXTR2_PF_NORMAL |
			XPDMA_CXTR2_PER_BURST |
			dir_bit |
			(request & 0xff);
	if (dir == dma_per2mem) {
		sChn->cx.sar = (uint32_t)paddr;
	}
	else {
		sChn->cx.dar = (uint32_t)paddr;
	}
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
	const struct libdma_perSetup *s = per->setup;
	int chn = per->chns[dir];
	if (chn == DMA_NUM_CHANNELS) {
		return -EINVAL;
	}

	handle_t interruptCond = (cond == NULL) ? dma_ctrl[s->dma].chns[chn].cond : *cond;
	interrupt(libdma_channelToIRQ(s->dma, chn), libdma_irqHandler, NULL, interruptCond, NULL);
	if (dir == dma_per2mem) {
		libdma_configureChannel(s->dma, chn, s->requests[dma_per2mem], dir, priority, paddr, psize, msize, pinc, minc, s->portPer, s->portMem);
	}
	else if (dir == dma_mem2per) {
		libdma_configureChannel(s->dma, chn, s->requests[dma_mem2per], dir, priority, paddr, msize, psize, minc, pinc, s->portMem, s->portPer);
	}
	else {
		return -EINVAL;
	}

	return EOK;
}


static void libdma_prepareTransfer(int dma, int chn, void *maddr, size_t len, int flags)
{
	volatile uint32_t *chnBase = libdma_channelBase(dma, chn);

	dataBarier();

	struct dma_channel *sChn = &dma_ctrl[dma].chns[chn];
	uint32_t cxtr1 = sChn->cx.tr1;
	if ((flags & DMA_OVERRIDE_SINC_OFF) != 0) {
		cxtr1 &= ~XPDMA_CXTR1_SINC;
	}

	if ((flags & DMA_OVERRIDE_DINC_OFF) != 0) {
		cxtr1 &= ~XPDMA_CXTR1_DINC;
	}

	if ((sChn->cx.tr2 & XPDMA_CXTR2_MEM2PER) != 0) {
		sChn->cx.sar = (uint32_t)maddr;
	}
	else {
		sChn->cx.dar = (uint32_t)maddr;
	}

	sChn->cx.tr2 &= ~XPDMA_CXTR2_TCEM_MASK;
	sChn->cx.tr2 |= ((flags & DMA_CIRCULAR) != 0) ? XPDMA_CXTR2_TCEM_EACH_LL : XPDMA_CXTR2_TCEM_LAST_LL;
	sChn->cx.br1 = len;
	sChn->cx.llr = 0; /* Link register == 0 (no list in memory) */

	*(chnBase + xpdma_cxlbar) = 0; /* Linked-list base address (unused) */
	*(chnBase + xpdma_cxfcr) = XPDMA_ALLFLAGS;
	*(chnBase + xpdma_cxtr1) = cxtr1;
	*(chnBase + xpdma_cxtr2) = sChn->cx.tr2;
	*(chnBase + xpdma_cxbr1) = sChn->cx.br1;
	*(chnBase + xpdma_cxsar) = sChn->cx.sar;
	*(chnBase + xpdma_cxdar) = sChn->cx.dar;
	*(chnBase + xpdma_cxllr) = sChn->cx.llr;

	uint32_t v = *(chnBase + xpdma_cxcr);
	v &= ~(0x3 << 22);
	v |= (uint32_t)(sChn->priority) << 22;
	v &= ~(XPDMA_TCF | XPDMA_HTF);
	v |= ((flags & DMA_USE_TC_IRQ) != 0) ? XPDMA_TCF : 0;
	v |= ((flags & DMA_USE_HT_IRQ) != 0) ? XPDMA_HTF : 0;
	*(chnBase + xpdma_cxcr) = v;
	dataBarier();
	*(chnBase + xpdma_cxcr) = v | 1;
	dataBarier();
}


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


static int libdma_transferTimeout(int dma, int chn, void *maddr, size_t len, int dir, int mode, int flags, time_t timeout)
{
	time_t now, end, condTimeout;
	volatile uint32_t *chnBase = libdma_channelBase(dma, chn);
	volatile int done = 0;

	libdma_setTransferOnceCallback(dma, chn, &done);
	if (dir == dma_per2mem) {
		libdma_cacheOpPer2Mem(maddr, len);
	}
	else {
		libdma_cacheOpMem2Per(maddr, len);
	}

	libdma_prepareTransfer(dma, chn, maddr, len, flags | DMA_USE_TC_IRQ);

	condTimeout = timeout;
	if (timeout != 0) {
		gettime(&now, NULL);
		end = now + timeout;
	}

	mutexLock(dma_ctrl[dma].chns[chn].irqLock);
	while (done == 0) {
		condWait(dma_ctrl[dma].chns[chn].cond, dma_ctrl[dma].chns[chn].irqLock, condTimeout);
		if (mode == dma_modeNoBlock) {
			break;
		}
		if (timeout != 0) {
			gettime(&now, NULL);
			if (end <= now) {
				break;
			}
			condTimeout = end - now;
		}
	}
	mutexUnlock(dma_ctrl[dma].chns[chn].irqLock);

	if (done == 1) {
		return len;
	}

	libdma_cleanupTransfer(dma, chn);

	return len - (*(chnBase + xpdma_cxbr1) & 0xffff);
}


static int libdma_transferHelperInterrupts(int dma, int rxChn, int txChn, void *rxMAddr, const void *txMAddr, size_t len, int flags)
{
	struct dma_channel *rxSChn = &dma_ctrl[dma].chns[rxChn];
	volatile uint32_t *txChnBase = libdma_channelBase(dma, txChn);
	volatile int rxDone = 0;

	if (rxMAddr == NULL) {
		return libdma_transferTimeout(dma, txChn, (void *)txMAddr, len, dma_mem2per, dma_modeNormal, flags, 0);
	}

	libdma_setTransferOnceCallback(dma, rxChn, &rxDone);
	libdma_cacheOpMem2Per((void *)txMAddr, len);
	libdma_cacheOpPer2Mem(rxMAddr, len);

	/* Wait on RX transfer using an interrupt + condWait and wait on TX channel by polling.
	 * Because RX transfer will usually finish last, there will be no long wait on the TX channel.
	 * This is for two reasons: we avoid unnecessary interrupt handling and condSignal()
	 * and also Phoenix-RTOS doesn't allow waiting on two conditionals at once. */
	libdma_prepareTransfer(dma, rxChn, rxMAddr, len, flags | DMA_USE_TC_IRQ);
	libdma_prepareTransfer(dma, txChn, (void *)txMAddr, len, flags);

	mutexLock(rxSChn->irqLock);
	while (rxDone == 0) {
		condWait(rxSChn->cond, rxSChn->irqLock, 0);
	}
	mutexUnlock(rxSChn->irqLock);

	while (libdma_hasChannelFinished(txChnBase) == 0) {
		/* Active-wait for TX channel */
	}

	/* RX channel was cleaned up by IRQ */
	libdma_cleanupTransfer(dma, txChn);

	return len;
}


static int libdma_transferHelperNoInterrupts(int dma, int rxChn, int txChn, void *rxMAddr, const void *txMAddr, size_t len, int flags)
{
	volatile uint32_t *rxChnBase;
	volatile uint32_t *txChnBase = libdma_channelBase(dma, txChn);

	if (rxMAddr == NULL) {
		rxChnBase = NULL;
	}
	else {
		rxChnBase = libdma_channelBase(dma, rxChn);
		libdma_cacheOpPer2Mem(rxMAddr, len);
		libdma_prepareTransfer(dma, rxChn, rxMAddr, len, flags);
	}

	libdma_cacheOpMem2Per((void *)txMAddr, len);
	libdma_prepareTransfer(dma, txChn, (void *)txMAddr, len, flags);

	while (!(libdma_hasChannelFinished(rxChnBase) && libdma_hasChannelFinished(txChnBase))) {
		/* Wait for transfer to finish */
	}

	if (rxChnBase != NULL) {
		libdma_cleanupTransfer(dma, rxChn);
	}

	libdma_cleanupTransfer(dma, txChn);

	return len;
}


int libdma_transfer(const struct libdma_per *per, void *rxMAddr, const void *txMAddr, size_t len)
{
	uint32_t dummyBuf = 0;
	int dma = per->setup->dma;
	int rxChn = per->chns[dma_per2mem];
	int txChn = per->chns[dma_mem2per];

	if ((len > DMA_MAX_LEN) || (rxChn == DMA_NUM_CHANNELS) || (txChn == DMA_NUM_CHANNELS)) {
		return -EINVAL;
	}

	int flags = 0;
	if (txMAddr == NULL) {
		/* In case no tx buffer is provided, use a dummy value
		 * and configure DMA not to increment the memory address. */
		txMAddr = &dummyBuf;
		flags |= DMA_OVERRIDE_SINC_OFF;
	}

	if (len > DMA_LONG_TRANSFER_THRESHOLD) {
		return libdma_transferHelperInterrupts(dma, rxChn, txChn, rxMAddr, txMAddr, len, flags);
	}
	else {
		return libdma_transferHelperNoInterrupts(dma, rxChn, txChn, rxMAddr, txMAddr, len, flags);
	}
}


int libdma_tx(const struct libdma_per *per, const void *txMAddr, size_t len, int mode, time_t timeout)
{
	int dma = per->setup->dma;
	int txChn = per->chns[dma_mem2per];

	if ((len > DMA_MAX_LEN) || (txChn == DMA_NUM_CHANNELS)) {
		return -EINVAL;
	}

	return libdma_transferTimeout(dma, txChn, (void *)txMAddr, len, dma_mem2per, mode, 0, timeout);
}


int libdma_rx(const struct libdma_per *per, void *rxMAddr, size_t len, int mode, time_t timeout)
{
	int dma = per->setup->dma;
	int rxChn = per->chns[dma_per2mem];

	if ((len > DMA_MAX_LEN) || (rxChn == DMA_NUM_CHANNELS)) {
		return -EINVAL;
	}

	return libdma_transferTimeout(dma, rxChn, rxMAddr, len, mode, dma_per2mem, 0, timeout);
}


static int libdma_transferAsync(const struct libdma_per *per, void *maddr, int dir, size_t len, volatile int *doneFlag)
{
	int dma = per->setup->dma;
	int chn = per->chns[dir];

	if ((len > DMA_MAX_LEN) || (chn == DMA_NUM_CHANNELS)) {
		return -EINVAL;
	}

	libdma_setTransferOnceCallback(dma, chn, doneFlag);
	if (dir == dma_per2mem) {
		libdma_cacheOpPer2Mem(maddr, len);
	}
	else {
		libdma_cacheOpMem2Per(maddr, len);
	}

	libdma_prepareTransfer(dma, chn, maddr, len, DMA_USE_TC_IRQ);

	return 0;
}


int libdma_txAsync(const struct libdma_per *per, const void *txMAddr, size_t len, volatile int *doneFlag)
{
	return libdma_transferAsync(per, (void *)txMAddr, dma_mem2per, len, doneFlag);
}


int libdma_rxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, volatile int *doneFlag)
{
	return libdma_transferAsync(per, rxMAddr, dma_per2mem, len, doneFlag);
}


int libdma_infiniteRxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, void fn(void *arg, int type), void *arg)
{
	return -ENOSYS;
}


uint16_t libdma_leftToRx(const struct libdma_per *per)
{
	volatile uint32_t *chnBase = libdma_channelBase(per->setup->dma, per->chns[dma_per2mem]);
	/* Only bottom 16 bits contain data. */
	return *(chnBase + xpdma_cxbr1) & 0xffff;
}


int libdma_init(void)
{
	if (dma_common.initialized) {
		return EOK;
	}

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
