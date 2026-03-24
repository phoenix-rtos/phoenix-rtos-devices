/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver.
 *
 * Low-level userspace driver using DMA Linked List Mode.
 * Supports only asynchronouse interface.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <board_config.h>
#include <phoenix/gaisler/ambapp.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#error "Unsupported TARGET"
#endif

#include "nandfctrl2-flashdrv.h"
#include "onfi-4.h"


/* clang-format off */
#define LOG(fmt, ...)       do { printf("grlib-nandfctrl2: " fmt "\n", ##__VA_ARGS__); } while (0)
#define TRACE(fmt, ...)     do { if (0) { LOG("%s:%d:" fmt, __FILE__, __LINE__, ##__VA_ARGS__); } } while (0)
#define LOG_ERROR(fmt, ...) do { LOG("error: " fmt, ##__VA_ARGS__); } while (0)
/* clang-format on */


#define NAND_BBM_SIZE         2 /* Bad Block Marker size in bytes */
#define ECC_BITFLIP_THRESHOLD 5 /* Min number of bitflips for page rewrite */


/* ============================== Register layout ============================== */


typedef struct {
	uint32_t ctrl0;    /* 0x000: Core control 0 */
	uint32_t ctrl1;    /* 0x004: Core control 1 (interrupt masks) */
	uint32_t ctrl2;    /* 0x008: Core control 2 (abort/reset/trigger) */
	uint32_t ctrl3;    /* 0x00c: Core control 3 */
	uint32_t ctrl4;    /* 0x010: Core control 4 */
	uint32_t res0[3];  /* 0x014-0x01f */
	uint32_t sts0;     /* 0x020: Core status 0 */
	uint32_t sts1;     /* 0x024: Core status 1 */
	uint32_t sts2;     /* 0x028: Core status 2 */
	uint32_t sts3;     /* 0x02c: Core status 3 */
	uint32_t res1[2];  /* 0x030-0x037 */
	uint32_t cap0;     /* 0x038: Capability 0 */
	uint32_t cap1;     /* 0x03c: Capability 1 */
	uint32_t cap2;     /* 0x040: Capability 2 */
	uint32_t cap3;     /* 0x044: Capability 3 */
	uint32_t cap4;     /* 0x048: Capability 4 */
	uint32_t cap5;     /* 0x04c: Capability 5 */
	uint32_t tme0;     /* 0x050: Timing 0 */
	uint32_t tme1;     /* 0x054: Timing 1 */
	uint32_t tme2;     /* 0x058: Timing 2 */
	uint32_t tme3;     /* 0x05c: Timing 3 */
	uint32_t tme4;     /* 0x060: Timing 4 */
	uint32_t tme5;     /* 0x064: Timing 5 */
	uint32_t tme6;     /* 0x068: Timing 6 */
	uint32_t tme7;     /* 0x06c: Timing 7 */
	uint32_t tme8;     /* 0x070: Timing 8 */
	uint32_t tme9;     /* 0x074: Timing 9 */
	uint32_t tme10;    /* 0x078: Timing 10 */
	uint32_t tme11;    /* 0x07c: Timing 11 */
	uint32_t res2[16]; /* 0x080-0x0bf */
	uint32_t txskew0;  /* 0x0c0: TX skew control */
	uint32_t rxskew0;  /* 0x0c4: RX skew control */
	uint32_t res3[2];  /* 0x0c8-0x0cf */
	uint32_t tout0;    /* 0x0d0: Timeout 0 */
	uint32_t res4[31]; /* 0x0d4-0x14f */
	uint32_t llpl;     /* 0x150: Linked list pointer low */
	uint32_t res5;     /* 0x154 */
	uint32_t res6[2];  /* 0x158-0x15f */
	uint32_t dcmd;     /* 0x160: Descriptor command (register mode) */
	uint32_t dtarsel0; /* 0x164: Descriptor target select 0 */
	uint32_t dtarsel1; /* 0x168: Descriptor target select 1 */
	uint32_t dchsel;   /* 0x16c: Descriptor channel select */
	uint32_t drbsel;   /* 0x170: Descriptor ready/busy select */
	uint32_t drow;     /* 0x174: Descriptor row address + control */
	uint32_t dcolsize; /* 0x178: Descriptor column address + size */
	uint32_t dsts;     /* 0x17c: Descriptor status */
	uint32_t deccsts0; /* 0x180: Descriptor ECC status */
	uint32_t res7[3];  /* 0x184-0x18f */
	uint32_t ddpl;     /* 0x190: Descriptor data pointer low */
	uint32_t res8[19]; /* 0x194-0x1df */
	uint32_t din0;     /* 0x1e0: Data-in 0 */
	uint32_t din1;     /* 0x1e4: Data-in 1 */
	uint32_t din2;     /* 0x1e8: Data-in 2 */
	uint32_t din3;     /* 0x1ec: Data-in 3 */
	uint32_t dout0;    /* 0x1f0: Data-out 0 */
	uint32_t dout1;    /* 0x1f4: Data-out 1 */
	uint32_t dout2;    /* 0x1f8: Data-out 2 */
	uint32_t dout3;    /* 0x1fc: Data-out 3 */
} nandfctrl2_regs_t;


/* ============================== Register bit definitions ============================== */

/* Core control 0 register */
#define CTRL0_EE        (1U << 13) /* EDAC enable */
#define CTRL0_LLM       (1U << 10) /* Linked list mode */
#define CTRL0_BBM       (1U << 9)  /* Preserve bad block marking */
#define CTRL0_MSEL_SHFT 3          /* Memory select shift */
#define CTRL0_MSEL_MSK  (3U << 3)  /* Memory select mask */

/* Core control 1 register */
#define CTRL1_IRQ_CERRDL  (1U << 13) /* Correctable error, downlink syncrams */
#define CTRL1_IRQ_CERRUL  (1U << 12) /* Correctable error, uplink syncrams */
#define CTRL1_IRQ_UCERRDL (1U << 11) /* Uncorrectable error, downlink syncrams */
#define CTRL1_IRQ_UCERRUL (1U << 10) /* Uncorrectable error, uplink syncrams */
#define CTRL1_IRQ_STOPLL  (1U << 9)  /* Stop linked list */
#define CTRL1_IRQ_ABORT   (1U << 8)  /* Abort done */
#define CTRL1_IRQ_TMOUT   (1U << 6)  /* Timeout */
#define CTRL1_IRQ_CMD     (1U << 5)  /* Invalid command */
#define CTRL1_IRQ_DL      (1U << 4)  /* DMA read (uplink write) error */
#define CTRL1_IRQ_UL      (1U << 3)  /* DMA write (downlink read) error */
#define CTRL1_IRQ_ECC     (1U << 2)  /* ECC uncorrectable */
#define CTRL1_IRQ_DS      (1U << 1)  /* Descriptor done */
#define CTRL1_IRQ         (1U << 0)  /* Main interrupt enable */

/* Interrupt enable mask - enables main IRQ + DS + all error sources */
#define CTRL1_OPS_IRQ_MASK \
	(CTRL1_IRQ | CTRL1_IRQ_DS | CTRL1_IRQ_ECC | CTRL1_IRQ_UL | CTRL1_IRQ_DL | CTRL1_IRQ_CMD | CTRL1_IRQ_TMOUT | CTRL1_IRQ_ABORT | CTRL1_IRQ_STOPLL)

/* Core control 2 register */
#define CTRL2_STOP_LL (1U << 3) /* Stop linked list */
#define CTRL2_ABORT   (1U << 2) /* Abort execution */
#define CTRL2_DT      (1U << 1) /* Descriptor trigger */
#define CTRL2_RST     (1U << 0) /* Software reset */

/* Core status 0 register */
#define STS0_DA  (1U << 3) /* Descriptor active */
#define STS0_RDY (1U << 0) /* Controller ready */

/* Core status 1 register */
#define STS1_UCERRDL (1U << 11) /* Uncorrectable error, downlink syncrams */
#define STS1_UCERRUL (1U << 10) /* Uncorrectable error, uplink syncrams */
#define STS1_ABORT   (1U << 8)  /* Abort complete */
#define STS1_TMOUT   (1U << 6)  /* Timeout */
#define STS1_DL      (1U << 4)  /* DMA read (uplink write) error */
#define STS1_UL      (1U << 3)  /* DMA write (downlink read) error */

#define STS1_FATAL_MSK (STS1_UCERRDL | STS1_UCERRUL | STS1_TMOUT | STS1_DL | STS1_UL)

/* Descriptor command register */
#define DCMD_CMD2_SHFT   24
#define DCMD_CMD1_SHFT   16
#define DCMD_DD          (1U << 11)               /* Data DMA disable (use APB, not DMA) */
#define DCMD_ED          (1U << 10)               /* EDAC disable */
#define DCMD_RD          (1U << 9)                /* Randomization disable */
#define DCMD_SUBCMD_SHFT 7                        /* Sub command shift */
#define DCMD_SUBCMD_MSK  (3U << DCMD_SUBCMD_SHFT) /* Sub command mask */
#define DCMD_SRB         (1U << 6)                /* Skip Ready/Busy wait before command */
#define DCMD_SA          (1U << 5)                /* Skip address phase */
#define DCMD_SD          (1U << 4)                /* Skip data phase */
#define DCMD_SC2         (1U << 3)                /* Skip second command (CMD2) phase */
#define DCMD_WARDY       (1U << 2)                /* Wait for ARDY after last command */
#define DCMD_IRQ_DS      (1U << 1)                /* Generate interrupt when descriptor finishes */
#define DCMD_EN          (1U << 0)                /* Descriptor enable */

/* Descriptor row & control register */
#define DROW_TAGEN (1U << 24) /* Tag enable */

/* Descriptor column & size register */
#define DCOLSIZE_SIZE_SHFT 16 /* Transfer size shift */

/* Descriptor status register */
#define DSTS_ECFAIL_SHFT 16                            /* ECC chunk fail bitmap shift */
#define DSTS_ECFAIL_MSK  (0xffffU << DSTS_ECFAIL_SHFT) /* ECC chunk fail bitmap mask */
#define DSTS_CERR_DL     (1U << 8)                     /* Correctable error, downlink syncrams */
#define DSTS_CERR_UL     (1U << 7)                     /* Correctable error, uplink syncrams */
#define DSTS_UCERR_DL    (1U << 6)                     /* Uncorrectable error, downlink syncrams */
#define DSTS_UCERR_UL    (1U << 5)                     /* Uncorrectable error, uplink syncrams */
#define DSTS_TMOUT       (1U << 4)                     /* Timeout */
#define DSTS_STOP_LL     (1U << 3)                     /* Stop linked list */
#define DSTS_ABORT       (1U << 2)                     /* Abort */
#define DSTS_UE          (1U << 1)                     /* Uncorrectable ECC error */
#define DSTS_INV         (1U << 0)                     /* Invalid command */

/* Descriptor ECC status register */
#define DECCSTS_CEC_SHFT 16                          /* Corrected errors in worst chunk shift */
#define DECCSTS_CEC_MSK  (0xffU << DECCSTS_CEC_SHFT) /* Corrected errors in worst chunk mask */
#define DECCSTS_CE_MSK   0xfffU                      /* Total corrected errors mask */

/* Capability 1 register */
#define CAP1_E1CAP_SHFT   26
#define CAP1_E1CHUNK_SHFT 21
#define CAP1_E1GF_SHFT    16
#define CAP1_E0CAP_SHFT   10
#define CAP1_E0CHUNK_SHFT 5
#define CAP1_E0GF_SHFT    0

#define CAP1_E1CAP   (0x3fU << CAP1_E1CAP_SHFT)
#define CAP1_E1CHUNK (0x1fU << CAP1_E1CHUNK_SHFT)
#define CAP1_E1GF    (0x1fU << CAP1_E1GF_SHFT)
#define CAP1_E0CAP   (0x3fU << CAP1_E0CAP_SHFT)
#define CAP1_E0CHUNK (0x1fU << CAP1_E0CHUNK_SHFT)
#define CAP1_E0GF    (0x1fU << CAP1_E0GF_SHFT)

/* Capability 2/3/4 register */
#define CAP_MSEL_SHFT   31
#define CAP_MSPARE_SHFT 16

#define CAP_MSEL   (1U << CAP_MSEL_SHFT)
#define CAP_MSPARE (0x1fffU << CAP_MSPARE_SHFT)
#define CAP_MDATA  0xffffU


/* ============================== Descriptor layout ============================== */

/*
 * LLM descriptor - must be 0x40 (64) byte aligned.
 * Fields mirror the APB shadow descriptor registers (0x160-0x190).
 * dsts and deccsts are written back by hardware after execution.
 */
typedef struct {
	uint32_t dcmd;     /* 0x00: Command, flags */
	uint32_t dtarsel0; /* 0x04: Target select low 32 bits */
	uint32_t dtarsel1; /* 0x08: Target select high 32 bits */
	uint32_t dchsel;   /* 0x0c: Channel select mask */
	uint32_t drbsel;   /* 0x10: Ready/Busy select mask */
	uint32_t drow;     /* 0x14: Row address + control */
	uint32_t dcolsize; /* 0x18: Size (31:16) | Column address (15:0) */
	uint32_t dsts;     /* 0x1c: Descriptor status (written by HW) */
	uint32_t deccsts;  /* 0x20: ECC status (written by HW) */
	uint32_t res1[3];  /* 0x24-0x2c */
	uint32_t ddpl;     /* 0x30: DMA data pointer (physical address) */
	uint32_t res2;     /* 0x34 */
	uint32_t dllpl;    /* 0x38: Next descriptor physical address (0 = end) */
	uint32_t res3;     /* 0x3c */
} nandfctrl2_desc_t;


/* ============================== DMA chain management ============================== */


#define DMA_SCRATCH_SIZE _PAGE_SIZE
#define DMA_DESC_SIZE    _PAGE_SIZE
#define DMA_MAX_DESCS    (DMA_DESC_SIZE / sizeof(nandfctrl2_desc_t))

#define DMA_NO_ECC            0U
#define DMA_PROGRAM_NO_COMMIT 0U
#define DMA_WITH_ECC          (1U << 0)
#define DMA_PROGRAM_COMMIT    (1U << 1)

typedef struct {
	uint32_t ndesc; /* Number of descriptors currently in the chain */
} flashdrv_dma_hdr_t;


struct _flashdrv_dma_t {
	flashdrv_dma_hdr_t hdr;
	nandfctrl2_desc_t *descs;
	void *scratchBuf;
};


typedef struct {
	uint32_t rowAddr;    /* 24-bit Page/Block address */
	uint16_t colAddr;    /* 16-bit Byte offset within the page */
	size_t transferSize; /* Number of bytes to read/write */
	addr_t data;         /* DMA physical address (0 if no data phase) */
} dma_xfer_t;


typedef struct {
	uint8_t manufacturerId;
	uint8_t deviceId;
	uint8_t bytes[3];
} __attribute__((packed)) flash_id_t;


/* ============================== Layout map (from board_config.h) ============================== */


typedef struct {
	uint64_t ceMask; /* CE# pin select mask */
	uint32_t rbMask; /* R/B# select mask */
	uint32_t channelMask;
} nand_layoutMap_t;


static const nand_layoutMap_t fctrl2_flashMap[NAND_DIE_CNT] = NAND_DIE_MAP;


static struct {
	volatile nandfctrl2_regs_t *regs;

	handle_t hwLock;
	handle_t irqLock;
	handle_t irqCond;
	handle_t irqHandle;

	_Atomic(uint32_t) sts1; /* Last STS1 value set by ISR */
	_Atomic(int) irqDone;   /* Set to 1 by ISR to mark a real interrupt */

	flashdrv_info_t info; /* Probed from die 0; shared by all dies */
} common;


static void abortOp(void);


/* ============================== Interrupt handler ============================== */


static int __attribute__((section(".interrupt"))) irqHandler(unsigned int n, void *data)
{
	(void)n;
	(void)data;

	/* Read and clear all pending interrupt flags */
	uint32_t sts = common.regs->sts1;
	common.regs->sts1 = sts;

	/* Store for main thread inspection */
	common.sts1 = sts;
	common.irqDone = 1;

	return 1;
}


/* ============================== DMA chain helpers ============================== */


static flashdrv_dma_t *dmaAlloc(void)
{
	flashdrv_dma_t *dma = malloc(sizeof(*dma));
	if (dma == NULL) {
		return NULL;
	}

	dma->descs = mmap(NULL, DMA_DESC_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (dma->descs == MAP_FAILED) {
		free(dma);
		return NULL;
	}

	dma->scratchBuf = mmap(NULL, DMA_SCRATCH_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (dma->scratchBuf == MAP_FAILED) {
		munmap(dma->descs, _PAGE_SIZE);
		free(dma);
		return NULL;
	}

	dma->hdr.ndesc = 0U;

	return dma;
}


static void dmaFree(flashdrv_dma_t *dma)
{
	(void)munmap(dma->descs, DMA_DESC_SIZE);
	(void)munmap(dma->scratchBuf, DMA_SCRATCH_SIZE);
	free(dma);
}


static nandfctrl2_desc_t *dmaDescAt(const flashdrv_dma_t *dma, size_t i)
{
	return &dma->descs[i];
}


static void *dmaScratch(const flashdrv_dma_t *dma)
{
	return dma->scratchBuf;
}


/* Reset the descriptor chain (reuse the dma buffer for a new operation). */
static void dmaChainReset(flashdrv_dma_t *dma)
{
	dma->hdr.ndesc = 0;
}


/*
 * Append one pre-built descriptor to the chain for `target`.
 * `xfer` may be NULL for commands with no address/data phase.
 */
static void dma_appendDesc(flashdrv_dma_t *dma, unsigned int target, uint32_t dcmd, const dma_xfer_t *xfer)
{
	unsigned int idx = dma->hdr.ndesc;

	/* Reserve one slot for the terminating descriptor appended by dma_run() */
	assert(idx < (DMA_MAX_DESCS - 1U));

	nandfctrl2_desc_t *desc = dmaDescAt(dma, idx);

	memset(desc, 0, sizeof(*desc));

	desc->dcmd = dcmd;

	/* Target routing */
	desc->dtarsel0 = (uint32_t)(fctrl2_flashMap[target].ceMask & 0xffffffffU);
	desc->dtarsel1 = (uint32_t)(fctrl2_flashMap[target].ceMask >> 32);
	desc->dchsel = fctrl2_flashMap[target].channelMask;
	desc->drbsel = fctrl2_flashMap[target].rbMask;

	if (xfer != NULL) {
		desc->drow = xfer->rowAddr;
		desc->dcolsize = (xfer->transferSize << DCOLSIZE_SIZE_SHFT) | xfer->colAddr;
		if (xfer->data != 0U) {
			desc->ddpl = xfer->data;
		}
	}

	/* Link previous descriptor to this one */
	if (idx > 0U) {
		nandfctrl2_desc_t *prev = dmaDescAt(dma, idx - 1U);
		prev->dllpl = (uint32_t)va2pa(desc);
	}

	++dma->hdr.ndesc;
}


/* ============================== Descriptor builder functions ============================== */


static void dmaAddReset(flashdrv_dma_t *dma, unsigned int target)
{
	/* RESET (0xFF), no CMD2, no address, no data */
	uint32_t dcmd = (0xffU << DCMD_CMD1_SHFT) | DCMD_SC2 | DCMD_SA | DCMD_SD | DCMD_ED | DCMD_EN;
	dma_appendDesc(dma, target, dcmd, NULL);
}


static void dmaAddReadId(flashdrv_dma_t *dma, unsigned int target, const dma_xfer_t *xfer)
{
	/* READ ID (0x90), no CMD2, no EDAC */
	uint32_t dcmd = (0x90U << DCMD_CMD1_SHFT) | DCMD_SC2 | DCMD_ED | DCMD_EN;
	dma_appendDesc(dma, target, dcmd, xfer);
}


static void dmaAddReadParamPage(flashdrv_dma_t *dma, unsigned int target, const dma_xfer_t *xfer)
{
	/* READ PARAMETER PAGE (0xEC), no CMD2, no EDAC */
	uint32_t dcmd = (0xecU << DCMD_CMD1_SHFT) | DCMD_SC2 | DCMD_ED | DCMD_EN;
	dma_appendDesc(dma, target, dcmd, xfer);
}


static void dmaAddRead(flashdrv_dma_t *dma, unsigned int target, const dma_xfer_t *xfer, uint32_t flags)
{
	/* READ (0x00 / 0x30) */
	uint32_t dcmd = (0x30U << DCMD_CMD2_SHFT) | (0x00U << DCMD_CMD1_SHFT) | DCMD_EN;
	if ((flags & DMA_WITH_ECC) == 0U) {
		dcmd |= DCMD_ED;
	}
	dma_appendDesc(dma, target, dcmd, xfer);
}


static void dmaAddChangeReadColumn(flashdrv_dma_t *dma, unsigned int target, const dma_xfer_t *xfer)
{
	/* CHANGE READ COLUMN (0x05 / 0xE0), no EDAC, skip R/B */
	uint32_t dcmd = (0xe0U << DCMD_CMD2_SHFT) | (0x05U << DCMD_CMD1_SHFT) | DCMD_ED | DCMD_SRB | DCMD_EN;
	dma_appendDesc(dma, target, dcmd, xfer);
}


static void dmaAddPageProgram(flashdrv_dma_t *dma, unsigned int target, const dma_xfer_t *xfer, uint32_t flags)
{
	/* PAGE PROGRAM (0x80), CMD2=0x10 only if commit */
	uint32_t dcmd = (0x80U << DCMD_CMD1_SHFT) | DCMD_EN;
	if ((flags & DMA_PROGRAM_COMMIT) == 0U) {
		dcmd |= DCMD_SC2;
	}
	else {
		dcmd |= (0x10U << DCMD_CMD2_SHFT);
	}
	if ((flags & DMA_WITH_ECC) == 0U) {
		dcmd |= DCMD_ED;
	}
	dma_appendDesc(dma, target, dcmd, xfer);
}


static void dmaAddChangeWriteColumn(flashdrv_dma_t *dma, unsigned int target, const dma_xfer_t *xfer, uint32_t flags)
{
	/* CHANGE WRITE COLUMN (0x85), SUBCMD=2, skip CMD2 */
	uint32_t dcmd = (0x85U << DCMD_CMD1_SHFT) | (2U << DCMD_SUBCMD_SHFT) | DCMD_EN;
	if ((flags & DMA_PROGRAM_COMMIT) == 0U) {
		dcmd |= DCMD_SC2;
	}
	else {
		dcmd |= (0x10U << DCMD_CMD2_SHFT);
	}
	if ((flags & DMA_WITH_ECC) == 0U) {
		dcmd |= DCMD_ED;
	}
	dma_appendDesc(dma, target, dcmd, xfer);
}


static void dmaAddBlockErase(flashdrv_dma_t *dma, unsigned int target, uint32_t row)
{
	/* BLOCK ERASE (0x60 / 0xD0), no EDAC */
	uint32_t dcmd = (0xd0U << DCMD_CMD2_SHFT) | (0x60U << DCMD_CMD1_SHFT) | DCMD_ED | DCMD_SD | DCMD_EN;
	dma_xfer_t xfer = { .rowAddr = row };
	dma_appendDesc(dma, target, dcmd, &xfer);
}


static void dmaAddStatusRead(flashdrv_dma_t *dma, unsigned int target)
{
	/* READ STATUS (0x70), no CMD2, skip address, no EDAC */
	uint32_t dcmd = (0x70U << DCMD_CMD1_SHFT) | DCMD_SC2 | DCMD_SA | DCMD_ED | DCMD_EN;
	dma_xfer_t xfer = { .transferSize = 1U, .data = (uint32_t)va2pa(dmaScratch(dma)) };
	dma_appendDesc(dma, target, dcmd, &xfer);
}


/*
 * Trigger LLM execution of the descriptor chain.
 * Returns 0 on success, negative errno on fatal controller error.
 * ECC errors are NOT fatal and must be checked separately via descriptor dsts.
 */
static int dmaRun(flashdrv_dma_t *dma)
{
	assert(dma->hdr.ndesc > 0);

	unsigned int lastIdx = dma->hdr.ndesc - 1U;

	/* Mark last active descriptor to generate an interrupt */
	dmaDescAt(dma, lastIdx)->dcmd |= DCMD_IRQ_DS;

	/* Append a terminating descriptor (EN bit = 0) after the last active one */
	nandfctrl2_desc_t *term = dmaDescAt(dma, lastIdx + 1U);
	memset(term, 0, sizeof(*term));
	dmaDescAt(dma, lastIdx)->dllpl = (uint32_t)va2pa(term);

	/* Prepare for interrupt */
	common.irqDone = 0;
	common.sts1 = 0U;

	/* Load first descriptor address and trigger */
	common.regs->llpl = (uint32_t)va2pa(dmaDescAt(dma, 0U));
	common.regs->ctrl2 = CTRL2_DT;

	/* Wait for interrupt - controller has HW timeout */
	mutexLock(common.irqLock);
	while (common.irqDone == 0) {
		condWait(common.irqCond, common.irqLock, 0);
	}
	mutexUnlock(common.irqLock);

	uint32_t sts1 = common.sts1;

	if ((sts1 & STS1_FATAL_MSK) != 0U) {
		abortOp();

		if ((sts1 & STS1_TMOUT) != 0U) {
			return -ETIME;
		}

		if ((sts1 & (STS1_UCERRDL | STS1_UCERRUL | STS1_DL | STS1_UL)) != 0U) {
			LOG_ERROR("sts1 error: 0x%x", sts1);
			return -EIO;
		}
	}

	return 0;
}


/*
 * Execute a descriptor chain that ends with a read_status descriptor.
 * Returns 0 on success, -EIO if the flash reports FAIL, or other negative errno.
 */
static int dmaRunAndCheckStatus(flashdrv_dma_t *dma)
{
	int err = dmaRun(dma);
	if (err < 0) {
		return err;
	}

	const uint8_t *scratch = dmaScratch(dma);
	/* NAND status: bit 0 set = FAIL */
	if ((scratch[0] & 0x01U) != 0U) {
		LOG_ERROR("NAND status FAIL");
		return -EIO;
	}
	return 0;
}


/* ============================== ONFI timing setup ============================== */


static uint32_t ns2cycles(uint32_t ns, uint32_t coreClk)
{
	uint32_t cycles = (uint32_t)(((uint64_t)ns * coreClk + 999999999U) / 1000000000U);
	return (cycles > 0U) ? cycles : 1U;
}


static void setOnfiTimingMode(unsigned int mode, const onfi_paramPage_t *paramPage, uint32_t coreClk, bool edoEn)
{
	const onfi_timingMode_t *t = onfi_getTimingModeSDR(mode);
	uint32_t tCS, tWW, tRR, tWB, tRHW, tWHR, tCCS, tADL, tREH, tRP, tRC, tWH, tWP, tWC, tVDLY;

	assert(t != NULL);

	tCS = (ns2cycles(t->tCS3, coreClk) - 1U) & 0xffffU;
	tWW = (ns2cycles(t->tWW, coreClk) - 1U) & 0xffffU;
	common.regs->tme0 = (tCS << 16) | tWW;

	tRR = (ns2cycles(t->tRR, coreClk) - 1U) & 0xffffU;
	tWB = (ns2cycles(t->tWB, coreClk) - 1U) & 0xffffU;
	common.regs->tme1 = (tRR << 16) | tWB;

	tRHW = (ns2cycles(t->tRHW, coreClk) - 1U) & 0xffffU;
	tWHR = (ns2cycles(t->tWHR, coreClk) - 1U) & 0xffffU;
	common.regs->tme2 = (tRHW << 16) | tWHR;

	if (paramPage != NULL) {
		tCCS = (ns2cycles(paramPage->tCCS, coreClk) - 1U) & 0xffffU;
		tADL = (ns2cycles(paramPage->tADL, coreClk) - 1U) & 0xffffU;
	}
	else {
		tCCS = (ns2cycles(ONFI_TCCS_BASE, coreClk) - 1U) & 0xffffU;
		tADL = (ns2cycles(t->tADL, coreClk) - 1U) & 0xffffU;
	}
	common.regs->tme3 = (tADL << 16) | tCCS;

	tREH = ns2cycles(t->tREH, coreClk) & 0xffffU;
	tRP = edoEn ? (ns2cycles(t->tRP, coreClk) & 0xffffU) : (ns2cycles(t->tREA, coreClk) & 0xffffU);
	tRC = ns2cycles(t->tRC, coreClk);
	if ((tREH + tRP) < tRC) {
		tREH += tRC - (tREH + tRP);
	}
	common.regs->tme4 = ((tREH - 1U) << 16) | (tRP - 1U);

	tWH = ns2cycles(t->tWH, coreClk) & 0xffffU;
	tWP = ns2cycles(t->tWP, coreClk) & 0xffffU;
	tWC = ns2cycles(t->tWC, coreClk);
	if ((tWH + tWP) < tWC) {
		tWH += tWC - (tWH + tWP);
	}
	common.regs->tme5 = ((tWH - 1U) << 16) | (tWP - 1U);

	common.regs->tme6 = 0U;
	common.regs->tme7 = 0U;
	common.regs->tme8 = 0U;
	common.regs->tme9 = 0U;
	common.regs->tme10 = 0U;

	tVDLY = (ns2cycles(ONFI_TVDLY, coreClk) - 1U) & 0xffffU;
	common.regs->tme11 = (tCS << 16) | tVDLY;
}


/* ============================== Controller initialisation ============================== */


static void abortOp(void)
{
	common.regs->ctrl2 = CTRL2_ABORT;

	/* Wait for abort + ready */
	while (((common.regs->sts1 & STS1_ABORT) == 0U) || ((common.regs->sts0 & STS0_RDY) == 0U)) { }
}


static void resetNandfctrl2(void)
{
	abortOp();

	/* Clear status register */
	common.regs->sts1 = common.regs->sts1;

	common.regs->ctrl2 = CTRL2_RST;

	while ((common.regs->ctrl2 & CTRL2_RST) != 0U) { }

	/* Preserve BBM */
	common.regs->ctrl0 = CTRL0_BBM | CTRL0_LLM | CTRL0_EE;
	common.regs->ctrl1 = 0U;
	common.regs->ctrl3 = 0U; /* Disable write protection */
	common.regs->ctrl4 = 0U;

	/* Timing mode 0 until ONFI page is read */
	setOnfiTimingMode(0U, NULL, SYSCLK_FREQ, 0);

	common.regs->tout0 = 0U; /* Disable timeout */
}


static void fillSpareLayout(flashdrv_info_t *info)
{
	const uint32_t msel = (common.regs->ctrl0 & CTRL0_MSEL_MSK) >> CTRL0_MSEL_SHFT;
	const volatile uint32_t *cap = &common.regs->cap2 + msel;
	const uint32_t eccSel = (*cap & CAP_MSEL) >> CAP_MSEL_SHFT;
	const uint32_t mspare = (*cap & CAP_MSPARE) >> CAP_MSPARE_SHFT;
	const uint32_t cap1 = common.regs->cap1;

	uint32_t chunksz, gfsz, eccCap;

	if (eccSel == 0) {
		chunksz = (cap1 & CAP1_E0CHUNK) >> CAP1_E0CHUNK_SHFT;
		gfsz = (cap1 & CAP1_E0GF) >> CAP1_E0GF_SHFT;
		eccCap = (cap1 & CAP1_E0CAP) >> CAP1_E0CAP_SHFT;
	}
	else {
		chunksz = (cap1 & CAP1_E1CHUNK) >> CAP1_E1CHUNK_SHFT;
		gfsz = (cap1 & CAP1_E1GF) >> CAP1_E1GF_SHFT;
		eccCap = (cap1 & CAP1_E1CAP) >> CAP1_E1CAP_SHFT;
	}

	info->eccChunksz = 1U << chunksz;
	info->eccsz = 2 * ((eccCap * gfsz + 15U) / 16U) * (info->writesz / info->eccChunksz);
	info->eccCap = eccCap;

	info->sparesz = mspare;
	info->spareavail = info->sparesz - info->eccsz - NAND_BBM_SIZE;

	TRACE("ECC: chunksz=%u gfsz: %u eccsz=%u ecccap=%u", info->eccChunksz, gfsz, info->eccsz, info->eccCap);
}


static int setMemorySelect(uint32_t pageSz, uint16_t spareSz)
{
	const uint32_t cap2 = common.regs->cap2;
	const uint32_t cap3 = common.regs->cap3;
	const uint32_t cap4 = common.regs->cap4;

	uint32_t msel;

	if (((cap2 & CAP_MDATA) == pageSz) && (((cap2 & CAP_MSPARE) >> CAP_MSPARE_SHFT) <= spareSz)) {
		msel = 0U;
	}
	else if (((cap3 & CAP_MDATA) == pageSz) && (((cap3 & CAP_MSPARE) >> CAP_MSPARE_SHFT) <= spareSz)) {
		msel = 1U;
	}
	else if (((cap4 & CAP_MDATA) == pageSz) && (((cap4 & CAP_MSPARE) >> CAP_MSPARE_SHFT) <= spareSz)) {
		msel = 2U;
	}
	else {
		LOG_ERROR("unsupported page/spare geometry (%u/%u)", pageSz, spareSz);
		return -EINVAL;
	}

	common.regs->ctrl0 = (common.regs->ctrl0 & ~CTRL0_MSEL_MSK) | (msel << CTRL0_MSEL_SHFT);

	return 0;
}


/* Probe die via ONFI, configure shared timing registers, fill the info struct. */
static int setupFlash(flashdrv_info_t *info, flashdrv_dma_t *dma, unsigned int target)
{
	/* Reset die */
	dmaChainReset(dma);

	dmaAddReset(dma, target);

	dma_xfer_t xfer = {
		.transferSize = sizeof(flash_id_t),
		.data = va2pa(dmaScratch(dma)),
	};
	dmaAddReadId(dma, target, &xfer);

	int err = dmaRun(dma);
	if (err < 0) {
		return err;
	}

	flash_id_t *flashId = (flash_id_t *)dmaScratch(dma);

	if ((flashId->manufacturerId == 0x2cU) && (flashId->deviceId == 0xacU)) {
		info->name = "Micron MT29F4G08ABBFA";
	}
	else {
		info->name = "Unknown NAND flash";
	}

	/* Read ONFI parameter page */
	dmaChainReset(dma);

	xfer = (dma_xfer_t) {
		.transferSize = sizeof(onfi_paramPage_t),
		.data = va2pa(dmaScratch(dma)),
	};
	dmaAddReadParamPage(dma, target, &xfer);

	err = dmaRun(dma);
	if (err < 0) {
		return err;
	}

	onfi_paramPage_t page;
	onfi_deserializeParamPage(&page, dmaScratch(dma));

	if ((page.signature[0] != 'O') || (page.signature[1] != 'N') || (page.signature[2] != 'F') || (page.signature[3] != 'I')) {
		LOG_ERROR("no ONFI signature on die %u", target);
		return -EIO;
	}

	unsigned int timingMode = onfi_calcTimingMode(&page);
	bool edoEn = timingMode > 3;

	setOnfiTimingMode(timingMode, &page, SYSCLK_FREQ, edoEn);

	err = setMemorySelect(page.bytesPerPage, page.spareBytesPerPage);
	if (err < 0) {
		return err;
	}

	info->writesz = page.bytesPerPage;
	info->pagesPerBlock = page.pagesPerBlock;
	info->erasesz = page.bytesPerPage * page.pagesPerBlock;
	info->size = (uint64_t)info->erasesz * page.blocksPerLun * page.numLuns;
	fillSpareLayout(info);

	TRACE("layout: page=%u spare=%u block=%u total=%" PRIu64, info->writesz, info->sparesz, info->erasesz, info->size);

	return 0;
}


static int readRawPageUnlocked(const nand_die_t *die, uint32_t page, void *data)
{
	const uint32_t writesz = die->info.writesz;
	const uint32_t sparesz = die->info.sparesz;

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	addr_t dataPhy = va2pa(data);
	dmaChainReset(dma);

	/* Whole Page Access for the Data Area */
	dma_xfer_t opData = {
		.rowAddr = page,
		.transferSize = writesz,
		.data = dataPhy,
	};
	dmaAddRead(dma, target, &opData, DMA_NO_ECC);

	/* Free Size Access for the Spare Area via CHANGE_READ_COLUMN */
	dma_xfer_t opSpare = {
		.rowAddr = page,
		.colAddr = (uint16_t)writesz,
		.transferSize = sparesz,
		.data = dataPhy + writesz,
	};
	dmaAddChangeReadColumn(dma, target, &opSpare);

	return dmaRun(dma);
}


static unsigned int checkErased(const void *buff, size_t offs, size_t len)
{
	const uint8_t *buff8 = buff;
	const uint32_t *buff32 = buff;
	unsigned int ret = 0;
	uint32_t data32;
	uint8_t data8;

	buff8 += offs / CHAR_BIT;
	offs %= CHAR_BIT;

	/* Check first byte */
	if (offs > 0) {
		data8 = *buff8++;
		data8 |= (uint8_t)(0xffU << (CHAR_BIT - offs));

		/* Is it also last byte? */
		if (offs + len < CHAR_BIT) {
			data8 |= (uint8_t)(0xffU >> (offs + len));
			len = 0;
		}
		else {
			len -= CHAR_BIT - offs;
		}

		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check bytes until 32-bit aligned address */
	while ((len > CHAR_BIT) && (((uintptr_t)buff8) % sizeof(data32))) {
		data8 = *buff8++;
		len -= CHAR_BIT;
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check 32-bit words */
	buff32 = (const uint32_t *)buff8;
	while (len > CHAR_BIT * sizeof(data32)) {
		data32 = *buff32++;
		len -= CHAR_BIT * sizeof(data32);

		if (data32 == 0xffffffffU) {
			continue;
		}
		ret += CHAR_BIT * sizeof(data32) - __builtin_popcount(data32);
	}

	/* Check rest of the bytes */
	buff8 = (const uint8_t *)buff32;
	while (len > CHAR_BIT) {
		data8 = *buff8++;
		len -= CHAR_BIT;
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check last byte */
	if (len > 0) {
		data8 = *buff8;
		data8 |= (uint8_t)(0xff >> len);
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	return ret;
}


static int verifyUncorrectableError(const nand_die_t *die, uint32_t page, void *buf, uint32_t *maxFlips)
{
	const uint32_t chunksz = die->info.eccChunksz;
	const uint32_t nchunks = die->info.writesz / chunksz;
	const uint32_t chunkEccsz = die->info.eccsz / nchunks;
	const uint32_t eccCap = die->info.eccCap;

	uint8_t *rawBuf = dmaScratch(die->dma);

	int err = readRawPageUnlocked(die, page, rawBuf);
	if (err != 0) {
		return err;
	}

	*maxFlips = 0;

	for (uint32_t i = 0; i < nchunks; i++) {
		uint32_t dataOffs = i * chunksz;
		uint32_t eccOffs = die->info.writesz + NAND_BBM_SIZE + (i * chunkEccsz);

		uint32_t flips = checkErased(rawBuf + dataOffs, 0, chunksz);

		flips += checkErased(rawBuf + eccOffs, 0, chunkEccsz);

		if (flips > eccCap) {
			/* A single chunk exceeded the capability. Truly uncorrectable. */
			return -EBADMSG;
		}

		if (flips > *maxFlips) {
			*maxFlips = flips;
		}
	}

	/* Page is either clean or has correctable bitflips */
	memset(buf, 0xffU, die->info.writesz);

	if (*maxFlips >= ECC_BITFLIP_THRESHOLD) {
		return -EUCLEAN;
	}

	return 0;
}


/* ============================== Public API ============================== */


nand_die_t *flashdrv_dieAlloc(unsigned int target)
{
	if (target >= NAND_DIE_CNT) {
		return NULL;
	}

	nand_die_t *die = malloc(sizeof(*die));
	if (die == NULL) {
		return NULL;
	}

	die->target = target;
	die->info = common.info;

	die->dma = dmaAlloc();
	if (die->dma == NULL) {
		free(die);
		return NULL;
	}

	return die;
}


void flashdrv_dieFree(nand_die_t *die)
{
	if (die == NULL) {
		return;
	}
	dmaFree(die->dma);
	free(die);
}


int flashdrv_reset(const nand_die_t *die)
{
	TRACE("%s(target=%u)", __func__, die->target);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dmaAddReset(dma, target);
	int err = dmaRun(dma);

	mutexUnlock(common.hwLock);
	return err;
}


int flashdrv_writePage(const nand_die_t *die, uint32_t page, const void *data)
{
	TRACE("%s(target=%u, page=%u, data=%p)", __func__, die->target, page, data);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;
	const uint32_t chunksz = die->info.eccChunksz;
	const uint32_t nChunks = die->info.writesz / chunksz;
	const uint32_t tagOfs = die->info.writesz + die->info.sparesz - die->info.spareavail;

	mutexLock(common.hwLock);
	dmaChainReset(dma);

	const uint8_t *data_ptr = (const uint8_t *)data;

	/* Write data area chunk by chunk */
	for (uint32_t i = 0; i < nChunks; i++) {
		dma_xfer_t xfer = {
			.rowAddr = page,
			.colAddr = (uint16_t)(i * chunksz),
			.transferSize = chunksz,
			.data = va2pa((void *)(data_ptr + (i * chunksz))),
		};

		if (i == 0) {
			/* First chunk: PAGE_PROGRAM, no commit, ECC on */
			dmaAddPageProgram(dma, target, &xfer, DMA_WITH_ECC | DMA_PROGRAM_NO_COMMIT);
		}
		else {
			/* Subsequent chunks: CHANGE_WRITE_COLUMN */
			dmaAddChangeWriteColumn(dma, target, &xfer, DMA_WITH_ECC | DMA_PROGRAM_NO_COMMIT);
		}
	}

	void *scratch = (uint8_t *)dmaScratch(dma);
	memset(scratch, 0xff, die->info.spareavail);

	/* Write dummy buffer to tag area and commit - prevents the controller zero-padding */
	dma_xfer_t xferSpare = {
		.rowAddr = page,
		.colAddr = (uint16_t)tagOfs,
		.transferSize = die->info.spareavail,
		.data = va2pa(scratch),
	};
	dmaAddChangeWriteColumn(dma, target, &xferSpare, DMA_NO_ECC | DMA_PROGRAM_COMMIT);
	dmaAddStatusRead(dma, target);

	int err = dmaRunAndCheckStatus(dma);

	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_readPage(const nand_die_t *die, uint32_t page, void *data, flashdrv_eccStatus_t *eccStatus)
{
	TRACE("%s(target=%u, page=%u, data=%p)", __func__, die->target, page, data);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	dma_xfer_t xfer = {
		.rowAddr = page,
		.colAddr = 0U,
		.transferSize = die->info.writesz,
		.data = va2pa(data),
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);

	dmaAddRead(dma, target, &xfer, DMA_WITH_ECC);

	int err = dmaRun(dma);
	if (err < 0) {
		mutexUnlock(common.hwLock);
		return err;
	}

	const volatile nandfctrl2_desc_t *desc = dmaDescAt(dma, 0U);

	if ((desc->dsts & (DSTS_UE | DSTS_ECFAIL_MSK)) != 0U) {
		uint32_t maxFlips = 0;
		/* Check if it's truly uncorrectable or just fully erased (ECC not initialized) */
		err = verifyUncorrectableError(die, page, data, &maxFlips);
		if (err == -EBADMSG) {
			eccStatus->eccState = flash_ecc_uncorrectable;
			eccStatus->worstBitflips = 0;
		}
		else {
			if (maxFlips > 0) {
				eccStatus->eccState = flash_ecc_corrected;
				eccStatus->worstBitflips = maxFlips;
			}
			else {
				eccStatus->eccState = flash_ecc_ok;
				eccStatus->worstBitflips = 0;
			}
		}

		mutexUnlock(common.hwLock);

		return err;
	}

	uint32_t cec = (desc->deccsts & DECCSTS_CEC_MSK) >> DECCSTS_CEC_SHFT;
	eccStatus->worstBitflips = cec;

	if (cec > 0U) {
		eccStatus->eccState = flash_ecc_corrected;
		if (cec >= ECC_BITFLIP_THRESHOLD) {
			/* Mark as degrading if it's close to the ECC capability. */
			err = -EUCLEAN;
		}
	}
	else {
		eccStatus->eccState = flash_ecc_ok;
		eccStatus->worstBitflips = 0;
	}

	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_metaWrite(const nand_die_t *die, uint32_t page, const void *data, size_t size)
{
	TRACE("%s(target=%u, page=%u, data=%p, size=%zu)", __func__, die->target, page, data, size);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;
	uint32_t tagOfs = die->info.writesz + die->info.sparesz - die->info.spareavail;

	/* Free-size tag program */
	dma_xfer_t xfer = {
		.rowAddr = page,
		.colAddr = (uint16_t)tagOfs,
		.transferSize = size,
		.data = va2pa((void *)data),
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dmaAddPageProgram(dma, target, &xfer, DMA_PROGRAM_COMMIT | DMA_NO_ECC);
	dmaAddStatusRead(dma, target);

	int err = dmaRunAndCheckStatus(dma);

	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_metaRead(const nand_die_t *die, uint32_t page, void *data, size_t size)
{
	TRACE("%s(target=%u, page=%u, data=%p, size=%zu)", __func__, die->target, page, data, size);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;
	uint32_t tagOfs = die->info.writesz + die->info.sparesz - die->info.spareavail;

	/* Free-size read from tag */
	dma_xfer_t xfer = {
		.rowAddr = page,
		.colAddr = (uint16_t)tagOfs,
		.transferSize = size,
		.data = va2pa(data),
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dmaAddRead(dma, target, &xfer, DMA_NO_ECC);
	int err = dmaRun(dma);

	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_writeRaw(const nand_die_t *die, uint32_t page, const void *data)
{
	TRACE("%s(target=%u, page=%u, data=%p)", __func__, die->target, page, data);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	/* PAGE_PROGRAM for data area, no commit, no ECC */
	dma_xfer_t opData = {
		.rowAddr = page,
		.colAddr = 0U,
		.transferSize = die->info.writesz,
		.data = va2pa((void *)data),
	};

	/* CHANGE_WRITE_COLUMN for spare area with commit */
	dma_xfer_t opSpare = {
		.rowAddr = 0U,
		.colAddr = (uint16_t)die->info.writesz,
		.transferSize = die->info.sparesz,
		.data = va2pa((void *)((const uint8_t *)data + die->info.writesz)),
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dmaAddPageProgram(dma, target, &opData, DMA_NO_ECC | DMA_PROGRAM_NO_COMMIT);
	dmaAddChangeWriteColumn(dma, target, &opSpare, DMA_PROGRAM_COMMIT);
	dmaAddStatusRead(dma, target);

	int err = dmaRunAndCheckStatus(dma);

	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_readRaw(const nand_die_t *die, uint32_t page, void *data)
{
	TRACE("%s(target=%u, page=%u, data=%p)", __func__, die->target, page, data);

	mutexLock(common.hwLock);
	int err = readRawPageUnlocked(die, page, data);
	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_erase(const nand_die_t *die, uint32_t block)
{
	TRACE("%s(target=%u, block=%u)", __func__, die->target, block);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dmaAddBlockErase(dma, target, block * die->info.pagesPerBlock);
	dmaAddStatusRead(dma, target);
	int err = dmaRunAndCheckStatus(dma);

	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_isbad(const nand_die_t *die, uint32_t block)
{
	TRACE("%s(target=%u, block=%u)", __func__, die->target, block);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	void *scratch = dmaScratch(dma);

	dma_xfer_t xfer = {
		.rowAddr = block * die->info.pagesPerBlock,
		.colAddr = (uint16_t)die->info.writesz,
		.transferSize = 1U,
		.data = va2pa(scratch),
	};

	dmaChainReset(dma);
	dmaAddRead(dma, target, &xfer, DMA_NO_ECC);
	int err = dmaRun(dma);

	uint8_t marker = ((const uint8_t *)scratch)[0];

	mutexUnlock(common.hwLock);

	if (err < 0) {
		return 1; /* Read error: assume bad */
	}

	/* 0x00 in first spare byte = bad block marker */
	return (marker == 0x00U) ? 1 : 0;
}


int flashdrv_markbad(const nand_die_t *die, uint32_t block)
{
	TRACE("%s(target=%u, block=%u)", __func__, die->target, block);

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	uint8_t *scratch = dmaScratch(dma);

	scratch[0] = 0x00U; /* Bad Block Marker */

	dma_xfer_t xfer = {
		.rowAddr = block * die->info.pagesPerBlock,
		.colAddr = (uint16_t)die->info.writesz,
		.transferSize = 1U,
		.data = va2pa(scratch),
	};

	dmaChainReset(dma);
	dmaAddPageProgram(dma, target, &xfer, DMA_PROGRAM_COMMIT | DMA_NO_ECC);
	dmaAddStatusRead(dma, target);
	int err = dmaRunAndCheckStatus(dma);

	mutexUnlock(common.hwLock);

	return err;
}


int flashdrv_init(void)
{
	unsigned int instance = 0U;
	ambapp_dev_t dev = { .devId = CORE_ID_NANDFCTRL2 };
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_ambapp,
		.task.ambapp = {
			.dev = &dev,
			.instance = &instance,
		},
	};

	if (platformctl(&pctl) < 0) {
		LOG_ERROR("NANDFCTRL2 not found on AMBA bus");
		return -ENODEV;
	}

	if (dev.bus != BUS_AMBA_APB) {
		LOG_ERROR("NANDFCTRL2 is not on APB bus");
		return -ENODEV;
	}

	uintptr_t base = (uintptr_t)dev.info.apb.base;
	uintptr_t pageOff = base & (_PAGE_SIZE - 1U);
	base &= ~(_PAGE_SIZE - 1U);

	void *mapped = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);

	if (mapped == MAP_FAILED) {
		LOG_ERROR("failed to map NANDFCTRL2 registers");
		return -ENOMEM;
	}

	common.regs = (void *)((uint8_t *)mapped + pageOff);

	int err = mutexCreate(&common.hwLock);
	if (err < 0) {
		(void)munmap(mapped, _PAGE_SIZE);
		return err;
	}

	err = mutexCreate(&common.irqLock);
	if (err < 0) {
		resourceDestroy(common.hwLock);
		(void)munmap(mapped, _PAGE_SIZE);
		return err;
	}

	err = condCreate(&common.irqCond);
	if (err < 0) {
		resourceDestroy(common.irqLock);
		resourceDestroy(common.hwLock);
		(void)munmap(mapped, _PAGE_SIZE);
		return err;
	}

	err = interrupt(dev.irqn, irqHandler, NULL, common.irqCond, &common.irqHandle);
	if (err < 0) {
		LOG_ERROR("failed to register interrupt %u: %d", dev.irqn, err);
		resourceDestroy(common.irqCond);
		resourceDestroy(common.irqLock);
		resourceDestroy(common.hwLock);
		(void)munmap(mapped, _PAGE_SIZE);
		return err;
	}

	/* Reset controller to known state */
	resetNandfctrl2();

	/* Enable interrupts: main + DS + error sources */
	common.regs->ctrl1 = CTRL1_OPS_IRQ_MASK;

	/* Allocate a temporary DMA buffer for die probing */
	flashdrv_dma_t *dma = dmaAlloc();
	if (dma == NULL) {
		LOG_ERROR("failed to allocate init DMA buffer");
		return -ENOMEM;
	}

	/* Probe die 0 for ONFI geometry and configure timing registers */
	err = setupFlash(&common.info, dma, 0U);
	if (err < 0) {
		dmaFree(dma);
		resourceDestroy(common.irqCond);
		resourceDestroy(common.irqLock);
		resourceDestroy(common.hwLock);
		(void)munmap(mapped, _PAGE_SIZE);
		LOG_ERROR("failed to setup die 0: %d", err);
		return err;
	}

	/* Reset remaining dies (same geometry) */
	for (unsigned int target = 1U; target < NAND_DIE_CNT; target++) {
		dmaChainReset(dma);
		dmaAddReset(dma, target);
		(void)dmaRun(dma);
	}

	dmaFree(dma);

	LOG("initialized %u die(s)", NAND_DIE_CNT);

	return 0;
}
