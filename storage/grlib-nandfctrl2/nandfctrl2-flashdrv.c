/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver.
 * Low-level userspace driver using DMA Linked List Mode (LLM).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/minmax.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <board_config.h>
#include <phoenix/gaisler/ambapp.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif

#include "nandfctrl2-flashdrv.h"
#include "nandfctrl2-onfi.h"


/* clang-format off */
#define LOG(str_, ...)       do { printf("nandfctrl2-flash: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG("error: %s:%d: " str_, __FILE__, __LINE__, ##__VA_ARGS__)
/* clang-format on */


/* ECC bitflip threshold above which -EUCLEAN is returned (page should be rewritten).
 * Can be overridden in board_config.h. */
#ifndef NAND_ECC_BITFLIP_THRESHOLD
#define NAND_ECC_BITFLIP_THRESHOLD 5U
#endif

/* Maximum allowed bitflip count for an erased page detected by raw read.
 * If a page reports uncorrectable ECC but the raw bitflip count per 512-byte chunk
 * is within this limit, the page is treated as erased. */
#ifndef NAND_ERASED_PAGE_BITFLIPS
#define NAND_ERASED_PAGE_BITFLIPS 4U
#endif


/* ============================== Register layout ============================== */


/*
 * APB register map for NANDFCTRL2 (volatile struct mapped via mmap).
 * Offsets match the hardware specification.
 */
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

/* CTRL0 bits */
#define CTRL0_EE        (1U << 13) /* EDAC enable */
#define CTRL0_LLM       (1U << 10) /* Linked list mode */
#define CTRL0_BBM       (1U << 9)  /* Preserve bad block marking */
#define CTRL0_MSEL_SHFT 3          /* Memory select shift */
#define CTRL0_MSEL_MSK  (3U << 3)  /* Memory select mask */

/* CTRL1 bits (interrupt masks) */
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

/* CTRL2 bits */
#define CTRL2_STOP_LL (1U << 3) /* Stop linked list */
#define CTRL2_ABORT   (1U << 2) /* Abort execution */
#define CTRL2_DT      (1U << 1) /* Descriptor trigger */
#define CTRL2_RST     (1U << 0) /* Software reset */

/* STS0 bits */
#define STS0_DA  (1U << 3) /* Descriptor active */
#define STS0_RDY (1U << 0) /* Controller ready */

/* STS1 bits */
#define STS1_ABORT (1U << 8) /* Abort complete */

/* DROW bits */
#define DROW_TAGEN        (1U << 24) /* Tag enable */
#define DROW_ROWADDR_SHFT 0          /* Row address (24-bit) field shift */

/* DCOLSIZE bits */
#define DCOLSIZE_SIZE_SHFT 16 /* Transfer size shift */
#define DCOLSIZE_COL_SHFT  0  /* Column address shift */

/* DCMD bits */
#define DCMD_CMD2_SHFT (24)
#define DCMD_CMD1_SHFT (16)
#define DCMD_DD        (1U << 11) /* Data DMA disable (use APB, not DMA) */
#define DCMD_ED        (1U << 10) /* EDAC disable */
#define DCMD_RD        (1U << 9)  /* Randomization disable */
#define DCMD_SRB       (1U << 6)  /* Skip Ready/Busy wait before command */
#define DCMD_SA        (1U << 5)  /* Skip address phase */
#define DCMD_SD        (1U << 4)  /* Skip data phase */
#define DCMD_SC2       (1U << 3)  /* Skip second command (CMD2) phase */
#define DCMD_WARDY     (1U << 2)  /* Wait for ARDY after last command */
#define DCMD_IRQ_DS    (1U << 1)  /* Generate interrupt when descriptor finishes */
#define DCMD_EN        (1U << 0)  /* Descriptor enable */

/* DSTS bits (descriptor status, written by hardware after execution) */
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

/* DECCSTS bits (ECC status, written by hardware after execution) */
#define DECCSTS_CEC_SHFT 16                          /* Corrected errors in worst chunk shift */
#define DECCSTS_CEC_MSK  (0xffU << DECCSTS_CEC_SHFT) /* Corrected errors in worst chunk mask */
#define DECCSTS_CE_MSK   0xfffU                      /* Total corrected errors mask */

/* CAP2/3/4 bits */
#define CAP_MSPARE_SHFT 16
#define CAP_MSPARE      (0x1fffU << CAP_MSPARE_SHFT)
#define CAP_MDATA       0xffffU

/* Interrupt enable mask used for operations - enables main IRQ + DS + all error sources */
#define CTRL1_OPS_IRQ_MASK \
	(CTRL1_IRQ | CTRL1_IRQ_DS | CTRL1_IRQ_UL | CTRL1_IRQ_DL | CTRL1_IRQ_CMD | CTRL1_IRQ_TMOUT | CTRL1_IRQ_ABORT)


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

_Static_assert(sizeof(nandfctrl2_desc_t) == 0x40U, "nandfctrl2_desc_t must be 64 bytes");


/* ============================== DMA chain management ============================== */

/*
 * flashdrv_dma_t occupies 2 PAGE_SIZE pages of physically contiguous uncached memory.
 *
 * Layout:
 *   [0x000 - 0x03F]  Management header (64 bytes)
 *   [0x040 - 0xFFF]  nandfctrl2_desc_t array (up to 63 descriptors)
 *   [0x1000 - 0x1FFF] Scratch buffer (for status reads, spare reads, etc.)
 */
#define FLASHDRV_DMA_ALLOC_SIZE   (2U * _PAGE_SIZE)
#define FLASHDRV_DMA_SCRATCH_SIZE _PAGE_SIZE
#define FLASHDRV_DMA_MAX_DESCS    (((_PAGE_SIZE) - 64U) / sizeof(nandfctrl2_desc_t))

/*
 * Management header at the beginning of the DMA allocation.
 * Must be exactly 64 bytes so descs[0] starts at offset 64 = 0x40.
 */
typedef struct {
	uint32_t ndesc; /* Number of descriptors currently in the chain */
} flashdrv_dma_hdr_t;


struct _flashdrv_dma_t {
	flashdrv_dma_hdr_t hdr;
	nandfctrl2_desc_t *descs;
	void *dataBuf;
};


typedef struct {
	uint32_t cmd;
	uint32_t rowAddr; /* 24-bit Page/Block address */
	uint16_t colAddr; /* 16-bit Byte offset within the page */

	uint8_t skipAddr; /* SA bit */
	uint8_t disEcc;   /* ED bit */

	size_t transferSize; /* Number of bytes to read/write */
	addr_t data;         /* DMA physical address (NULL if no data phase) */
} nand_op_t;


/* ============================== Command table ============================== */


/* clang-format off */
enum {
	cmd_read = 0, cmd_read_multiplane, cmd_copyback_read, cmd_change_read_column, cmd_change_read_column_enhanced,
	cmd_read_cache_random, cmd_read_cache_sequential, cmd_read_cache_end, cmd_block_erase, cmd_block_erase_multiplane,
	cmd_read_status, cmd_read_status_enhanced, cmd_page_program, cmd_page_program_multiplane, cmd_page_cache_program,
	cmd_copyback_program, cmd_copyback_program_multiplane, cmd_small_data_move, cmd_change_write_column,
	cmd_change_row_address, cmd_read_id, cmd_volume_select, cmd_odt_configure, cmd_read_parameter_page,
	cmd_read_unique_id, cmd_get_features, cmd_set_features, cmd_lun_get_features, cmd_lun_set_features,
	cmd_zq_calibration_short, cmd_zq_calibration_long, cmd_reset_lun, cmd_synchronous_reset, cmd_reset,
	cmd_count
};
/* clang-format on */

typedef struct {
	uint8_t cmd1;
	uint8_t cmd2;
	uint8_t addrsz;   /* Bytes in address phase (unused; controller uses DROW/DCOLSIZE) */
	uint8_t skipCmd2; /* 1 = set DCMD_SC2 (no CMD2 phase) */
} nand_cmd_t;


/* clang-format off */
static const nand_cmd_t nand_commands[cmd_count] = {
	{ 0x00U, 0x30U, 5U, 0U }, /* cmd_read */
	{ 0x00U, 0x32U, 5U, 0U }, /* cmd_read_multiplane */
	{ 0x00U, 0x35U, 5U, 0U }, /* cmd_copyback_read */
	{ 0x05U, 0xe0U, 5U, 0U }, /* cmd_change_read_column */
	{ 0x06u, 0xe0U, 5U, 0U }, /* cmd_change_read_column_enhanced */
	{ 0x00U, 0x31U, 5U, 0U }, /* cmd_read_cache_random */
	{ 0x31U, 0x00U, 5U, 1U }, /* cmd_read_cache_sequential */
	{ 0x3fU, 0x00U, 5U, 1U }, /* cmd_read_cache_end */
	{ 0x60U, 0xd0U, 3U, 0U }, /* cmd_block_erase */
	{ 0x60U, 0xd1U, 3U, 0U }, /* cmd_block_erase_multiplane */
	{ 0x70U, 0x00U, 1U, 1U }, /* cmd_read_status */
	{ 0x78u, 0x00U, 3U, 1U }, /* cmd_read_status_enhanced */
	{ 0x80U, 0x10U, 5U, 0U }, /* cmd_page_program */
	{ 0x80U, 0x11U, 5U, 0U }, /* cmd_page_program_multiplane */
	{ 0x80U, 0x15U, 5U, 0U }, /* cmd_page_cache_program */
	{ 0x85U, 0x10U, 5U, 0U }, /* cmd_copyback_program */
	{ 0x85U, 0x11U, 5U, 0U }, /* cmd_copyback_program_multiplane */
	{ 0x85U, 0x11U, 2U, 0U }, /* cmd_small_data_move */
	{ 0x85U, 0x00U, 5U, 1U }, /* cmd_change_write_column */
	{ 0x85U, 0x00U, 5U, 1U }, /* cmd_change_row_address */
	{ 0x90U, 0x00U, 1U, 1U }, /* cmd_read_id */
	{ 0xe1U, 0x00U, 0U, 1U }, /* cmd_volume_select */
	{ 0xe2U, 0x00U, 0U, 1U }, /* cmd_odt_configure */
	{ 0xecu, 0x00U, 1U, 1U }, /* cmd_read_parameter_page */
	{ 0xedU, 0x00U, 1U, 1U }, /* cmd_read_unique_id */
	{ 0xeeU, 0x00U, 1U, 1U }, /* cmd_get_features */
	{ 0xefU, 0x00U, 5U, 1U }, /* cmd_set_features */
	{ 0xd4U, 0x00U, 0U, 1U }, /* cmd_lun_get_features */
	{ 0xd5U, 0x00U, 0U, 1U }, /* cmd_lun_set_features */
	{ 0xd9U, 0x00U, 0U, 1U }, /* cmd_zq_calibration_short */
	{ 0xf9U, 0x00U, 0U, 1U }, /* cmd_zq_calibration_long */
	{ 0xfaU, 0x00U, 0U, 1U }, /* cmd_reset_lun */
	{ 0xfcU, 0x00U, 0U, 1U }, /* cmd_synchronous_reset */
	{ 0xffU, 0x00U, 0U, 1U }, /* cmd_reset */
};
/* clang-format on */


/* ============================== Layout map (from board_config.h) ============================== */


typedef struct {
	uint64_t ceMask; /* CE# pin select mask */
	uint32_t rbMask; /* R/B# select mask */
	uint32_t channelMask;
} nand_layoutMap_t;


static const nand_layoutMap_t fctrl2_flashMap[NAND_DIE_CNT] = NAND_DIE_MAP;


/* ============================== Common driver state ============================== */


static struct {
	volatile nandfctrl2_regs_t *regs;

	handle_t mutex;      /* Serializes hardware access: held for entire operation */
	handle_t wait_mutex; /* Used only for condWait */
	handle_t cond;       /* Signaled by kernel when ISR returns >= 0 */
	handle_t irqHandle;

	_Atomic(uint32_t) sts1; /* Last STS1 value set by ISR (write-1-to-clear) */
	_Atomic(int) irqDone;   /* Set to 1 by ISR to mark a real interrupt */

	flashdrv_info_t info[NAND_DIE_CNT];
} common;


/* ============================== Interrupt handler ============================== */


static int nandfctrl2_irqHandler(unsigned int n, void *data)
{
	(void)n;
	(void)data;

	/* Read and clear (W1C) all pending interrupt flags */
	uint32_t sts = common.regs->sts1;
	common.regs->sts1 = sts;

	/* Store for main thread inspection */
	atomic_store(&common.sts1, sts);
	atomic_store(&common.irqDone, 1);

	/* Returning >= 0 causes the kernel to signal fctrl2_common.cond */
	return 0;
}


/* ============================== DMA chain helpers ============================== */


static nandfctrl2_desc_t *dma_descAt(flashdrv_dma_t *dma, unsigned int i)
{
	return &dma->descs[i];
}


static void *dma_scratch(flashdrv_dma_t *dma)
{
	return dma->dataBuf;
}


/* Reset the descriptor chain (reuse the dma buffer for a new operation). */
static void dma_chainReset(flashdrv_dma_t *dma)
{
	dma->hdr.ndesc = 0;
}


/*
 * Add one descriptor to the chain for 'target', filling all routing and
 * command fields.  Returns a pointer to the freshly initialised descriptor.
 */
static void dma_addDesc(flashdrv_dma_t *dma, const nand_op_t *op)
{
	unsigned int target = dma->hdr.target;
	unsigned int idx = dma->hdr.ndesc;
	const nand_cmd_t *cmd = &nand_commands[op->cmd];
	nandfctrl2_desc_t *desc;
	nandfctrl2_desc_t *prev;

	assert(idx < FLASHDRV_DMA_MAX_DESCS);

	desc = dma_descAt(dma, idx);
	memset(desc, 0, sizeof(*desc));

	/* Build DCMD */
	desc->dcmd = ((uint32_t)cmd->cmd2 << DCMD_CMD2_SHFT) | ((uint32_t)cmd->cmd1 << DCMD_CMD1_SHFT) | DCMD_EN;

	if (cmd->skipCmd2 != 0U) {
		desc->dcmd |= DCMD_SC2;
	}
	if (op->skipAddr) {
		desc->dcmd |= DCMD_SA;
	}
	if (op->disEcc) {
		desc->dcmd |= DCMD_ED;
	}
	if (op->data == 0U) {
		desc->dcmd |= DCMD_SD;
	}

	/* Target routing */
	desc->dtarsel0 = (uint32_t)(fctrl2_flashMap[target].ceMask & 0xffffffffU);
	desc->dtarsel1 = (uint32_t)(fctrl2_flashMap[target].ceMask >> 32);
	desc->drbsel = fctrl2_flashMap[target].rbMask;
	desc->dchsel = fctrl2_flashMap[target].channelMask;

	/* Address */
	desc->drow = op->rowAddr << DROW_ROWADDR_SHFT;
	desc->dcolsize = (op->transferSize << DCOLSIZE_SIZE_SHFT) | op->colAddr;

	/* DMA data pointer */
	if (op->data != 0U) {
		desc->ddpl = op->data;
	}

	/* Link previous descriptor to this one (terminated by default) */
	desc->dllpl = 0U;
	if (idx > 0U) {
		prev = dma_descAt(dma, idx - 1U);
		prev->dllpl = (uint32_t)va2pa(desc);
	}

	dma->hdr.ndesc = idx + 1U;
}


/*
 * Trigger LLM execution of the descriptor chain in dma.
 * Sets DCMD_IRQ_DS on the last descriptor so we get an interrupt when done.
 * Blocks until the interrupt fires, then returns the STS1 value.
 */
static uint32_t dma_run(flashdrv_dma_t *dma)
{
	uint32_t sts;

	if (dma->hdr.ndesc == 0U) {
		return 0U;
	}

	/* Mark last descriptor to generate an interrupt */
	dma_descAt(dma, dma->hdr.ndesc - 1U)->dcmd |= DCMD_IRQ_DS;

	/* Prepare for interrupt */
	atomic_store(&common.irqDone, 0);
	atomic_store(&common.sts1, 0U);

	/* Load first descriptor address and trigger */
	common.regs->llpl = (uint32_t)va2pa(dma_descAt(dma, 0U));
	common.regs->ctrl2 = CTRL2_DT;

	/* Wait for interrupt */
	mutexLock(common.wait_mutex);
	while (atomic_load(&common.irqDone) == 0) {
		condWait(common.cond, common.wait_mutex, 0);
	}
	mutexUnlock(common.wait_mutex);

	sts = atomic_load(&common.sts1);
	return sts;
}


/* ============================== Descriptor status helpers ============================== */


/*
 * Check a descriptor's dsts field after execution.
 * Returns 0 on success, negative errno on hardware error.
 * Does NOT check ECC errors (use flashdrv_checkDescEcc for that).
 */
static int flashdrv_checkDescHwErr(const nandfctrl2_desc_t *desc)
{
	uint32_t dsts = desc->dsts;

	if ((dsts & DSTS_TMOUT) != 0U) {
		return -ETIME;
	}

	if ((dsts & (DSTS_ABORT | DSTS_STOP_LL | DSTS_INV)) != 0U) {
		return -EIO;
	}

	if ((dsts & (DSTS_UCERR_DL | DSTS_UCERR_UL)) != 0U) {
		return -EIO;
	}

	return 0;
}


/*
 * Count the number of '0' bits in a buffer (i.e. bits flipped from 0xFF).
 */
static unsigned int flashdrv_countBitflips(const uint8_t *buf, size_t len)
{
	unsigned int flips = 0U;

	for (size_t i = 0U; i < len; i++) {
		flips += (unsigned int)__builtin_popcount((unsigned int)(~buf[i]) & 0xffU);
	}

	return flips;
}


/*
 * Check ECC status from a completed read descriptor.
 * Also handles erased-page detection via raw re-read.
 *
 * Returns:
 *   0           - no errors
 *   -EUCLEAN    - corrected errors above bitflip threshold (degraded page)
 *   -EBADMSG    - uncorrectable errors
 */
static int flashdrv_checkDescEcc(const nandfctrl2_desc_t *desc)
{
	/* Hardware errors checked separately in checkDescHwErr */
	if ((desc->dsts & (DSTS_UE | DSTS_ECFAIL_MSK)) == 0U) {
		uint32_t cec = (desc->deccsts & DECCSTS_CEC_MSK) >> DECCSTS_CEC_SHFT; /* corrected errors in worst chunk */
		if (cec >= NAND_ECC_BITFLIP_THRESHOLD) {
			return -EUCLEAN;
		}
		return 0;
	}

	/* Uncorrectable ECC error */
	return -EBADMSG;
}


/* ============================== Flash status read helper ============================== */


/*
 * Add a read_status descriptor to the chain.
 * The status byte is DMA'd to the scratch buffer.
 * SRB=0 so controller waits for R/B# before issuing the status command.
 */
static void dma_addStatusRead(flashdrv_dma_t *dma)
{
	void *scratch = dma_scratch(dma);
	nand_op_t op = {
		.cmd = cmd_read_status,
		.rowAddr = 0U,
		.colAddr = 0U,
		.transferSize = 1U,
		.data = (uint32_t)va2pa(scratch),
		.skipAddr = 1U,
		.disEcc = 1U,
	};

	dma_addDesc(dma, &op);
}


/*
 * Execute a descriptor chain that ends with a read_status descriptor.
 * Returns 0 on success, -EIO if the flash reports FAIL, or other negative errno.
 */
static int dma_runAndCheckStatus(flashdrv_dma_t *dma)
{
	const nandfctrl2_desc_t *last;
	const uint8_t *scratch;
	unsigned int ndesc;
	int err;

	(void)dma_run(dma);

	ndesc = dma->hdr.ndesc;
	if (ndesc == 0U) {
		return -EIO;
	}

	last = dma_descAt(dma, ndesc - 1U);

	err = flashdrv_checkDescHwErr(last);
	if (err < 0) {
		return err;
	}

	scratch = (const uint8_t *)dma_scratch(dma);
	/* NAND status: bit 0 set = FAIL */
	return ((scratch[0] & 0x01U) != 0U) ? -EIO : 0;
}


/* ============================== ONFI timing setup ============================== */


static uint32_t ns2cycles(uint32_t ns, uint32_t coreClk)
{
	return (uint32_t)(((uint64_t)ns * coreClk + 999999999U) / 1000000000U);
}


static void setOnfiTimingMode(unsigned int mode, const onfi_paramPage_t *paramPage, uint32_t coreClk, bool edoEn)
{
	const onfi_timingMode_t *t = onfi_getTimingModeSDR(mode);
	uint32_t tCS, tWW, tRR, tWB, tRHW, tWHR, tCCS, tADL;
	uint32_t tREH, tRP, tRC, tWH, tWP, tWC, tVDLY;

	if (t == NULL) {
		return;
	}

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


static void fctrl2Reset(void)
{
	common.regs->ctrl2 = CTRL2_ABORT;

	/* Wait for abort + ready */
	while (((common.regs->sts1 & STS1_ABORT) == 0U) || ((common.regs->sts0 & STS0_RDY) == 0U)) { }

	/* Clear abort flag (W1C) */
	common.regs->sts1 = STS1_ABORT;

	common.regs->ctrl2 = CTRL2_RST;

	/* Preserve BBM; LLM and EE will be set after timing configuration */
	common.regs->ctrl0 = CTRL0_BBM;
	common.regs->ctrl1 = 0U;
	common.regs->ctrl3 = 0U; /* Disable write protection */
	common.regs->ctrl4 = 0U;

	/* Conservative timing (mode 0) until ONFI page is read */
	setOnfiTimingMode(0U, NULL, SYSCLK_FREQ, 0);

	common.regs->tout0 = 0U; /* Disable timeout */
}


static int setMemorySelect(uint32_t pageSz, uint16_t spareSz)
{
	const uint32_t cap2 = common.regs->cap2;
	const uint32_t cap3 = common.regs->cap3;
	const uint32_t cap4 = common.regs->cap4;

	uint32_t msel;

	if (((cap2 & CAP_MDATA) == pageSz) && (((cap2 & CAP_MSPARE) >> CAP_MSPARE_SHFT) == (uint32_t)spareSz)) {
		msel = 0U;
	}
	else if (((cap3 & CAP_MDATA) == pageSz) && (((cap3 & CAP_MSPARE) >> CAP_MSPARE_SHFT) == (uint32_t)spareSz)) {
		msel = 1U;
	}
	else if (((cap4 & CAP_MDATA) == pageSz) && (((cap4 & CAP_MSPARE) >> CAP_MSPARE_SHFT) == (uint32_t)spareSz)) {
		msel = 2U;
	}
	else {
		LOG_ERROR("unsupported page/spare geometry (%u/%u)", pageSz, spareSz);
		return -EINVAL;
	}

	common.regs->ctrl0 = (common.regs->ctrl0 & ~CTRL0_MSEL_MSK) | (msel << CTRL0_MSEL_SHFT);

	return 0;
}


/* Per-die initialisation: read ONFI, configure timing, fill info struct. */
static int flashdrv_setupDie(flashdrv_dma_t *dma, unsigned int target)
{
	onfi_paramPage_t *page;
	flashdrv_info_t *info = &common.info[target];
	unsigned int timingMode;
	bool edoEn;
	int err;

	/* Re-use scratch as temporary buffer (must fit onfi_paramPage_t = 256 bytes) */
	_Static_assert(sizeof(onfi_paramPage_t) <= FLASHDRV_DMA_SCRATCH_OFF, "ONFI param page must fit in DMA scratch buffer");

	page = (onfi_paramPage_t *)dma_scratch(dma);

	/* Reset die */
	dma_chainReset(dma);

	nand_op_t op = {
		.cmd = cmd_reset,
		.rowAddr = 0U,
		.colAddr = 0U,
		.transferSize = 0U,
		.data = 0U,
		.skipAddr = 1U,
		.disEcc = 1U,
	};
	dma_addDesc(dma, &op);
	(void)dma_run(dma);

	/* Read ONFI parameter page */
	dma_chainReset(dma);
	dma_addDesc(dma, cmd_read_parameter_page, 0U, 0U, (uint32_t)sizeof(onfi_paramPage_t), (uint32_t)va2pa(page), DCMD_SA | DCMD_ED);
	(void)dma_run(dma);

	if ((page->signature[0] != 'O') || (page->signature[1] != 'N') || (page->signature[2] != 'F') || (page->signature[3] != 'I')) {
		LOG_ERROR("no ONFI signature on die %u", target);
		return -EIO;
	}

	timingMode = 31U - (unsigned int)__builtin_clz((unsigned int)page->timingMode);
	edoEn = timingMode > 3;

	setOnfiTimingMode(timingMode, page, SYSCLK_FREQ, edoEn);

	err = setMemorySelect(page->bytesPerPage, page->spareBytesPerPage);
	if (err < 0) {
		return err;
	}

	/* Determine flash name */
	if ((page->jedecId == 0x2cU) && (page->devModel[0] == 'M') && (page->devModel[1] == 'T')) {
		info->name = "Micron NAND";
	}
	else {
		info->name = "Unknown NAND";
	}

	info->writesz = page->bytesPerPage;
	info->sparesz = page->spareBytesPerPage;
	info->spareavail = NAND_SPARE_AVAIL;
	info->pagesPerBlock = page->pagesPerBlock;
	info->erasesz = page->bytesPerPage * page->pagesPerBlock;
	info->size = (uint64_t)info->erasesz * page->blocksPerLun * page->numLuns;

	LOG("die %u: %s  size=%llu  page=%u+%u  block=%u pages  spare_avail=%u",
			target, info->name,
			(unsigned long long)info->size,
			info->writesz, info->sparesz,
			info->pagesPerBlock,
			info->spareavail);

	return 0;
}


/* ============================== Public API ============================== */


flashdrv_dma_t *flashdrv_dmanew(unsigned int target)
{
	flashdrv_dma_t *dma;

	if (target >= NAND_DIE_CNT) {
		return NULL;
	}

	dma = mmap(NULL, FLASHDRV_DMA_ALLOC_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (dma == MAP_FAILED) {
		return NULL;
	}

	dma->hdr.ndesc = 0U;

	return dma;
}


void flashdrv_dmadestroy(flashdrv_dma_t *dma)
{
	if ((dma != NULL) && (dma != MAP_FAILED)) {
		(void)munmap(dma, FLASHDRV_DMA_ALLOC_SIZE);
	}
}


int flashdrv_reset(flashdrv_dma_t *dma)
{
	mutexLock(common.mutex);

	dma_chainReset(dma);

	nand_op_t op = {
		.cmd = cmd_reset,
		.rowAddr = 0U,
		.colAddr = 0U,
		.skipAddr = 1U,
		.disEcc = 1U,
		.transferSize = 0U,
		.data = 0U,
	};
	dma_addDesc(dma, &op);
	(void)dma_run(dma);

	int err = flashdrv_checkDescHwErr(dma_descAt(dma, 0U));

	mutexUnlock(common.mutex);
	return err;
}


int flashdrv_write(flashdrv_dma_t *dma, uint32_t paddr, const void *data, const void *metadata)
{
	unsigned int target = dma->hdr.target;
	const flashdrv_info_t *info = &common.info[target];
	uint32_t tagOfs;
	int err = 0;

	if (data == NULL && metadata == NULL) {
		return 0;
	}

	mutexLock(common.mutex);

	/* --- Write data area with EDAC enabled --- */
	if (data != NULL) {
		dma_chainReset(dma);
		dma_addDesc(dma, cmd_page_program, paddr, 0U, info->writesz, (uint32_t)va2pa(data), 0U);
		dma_addStatusRead(dma);
		err = dma_runAndCheckStatus(dma);
		if (err < 0) {
			mutexUnlock(common.mutex);
			return err;
		}
	}

	/* --- Write metadata (user OOB) at end of spare, EDAC disabled --- */
	if (metadata != NULL) {
		tagOfs = info->writesz + info->sparesz - info->spareavail;

		dma_chainReset(dma);
		dma_addDesc(dma, cmd_page_program, paddr, (uint16_t)tagOfs, info->spareavail, (uint32_t)va2pa((void *)metadata), DCMD_ED);
		dma_addStatusRead(dma);
		err = dma_runAndCheckStatus(dma);
	}

	mutexUnlock(common.mutex);
	return err;
}


int flashdrv_read(flashdrv_dma_t *dma, uint32_t page, void *data, flashdrv_eccStatus_t *meta)
{
	unsigned int target = dma->hdr.target;
	const flashdrv_info_t *info = &common.info[target];
	const nandfctrl2_desc_t *desc;
	uint32_t tagOfs;
	int ret = 0;
	int err;

	if (data == NULL && meta == NULL) {
		return 0;
	}

	mutexLock(common.mutex);

	/* --- Read data area with EDAC enabled --- */
	if (data != NULL) {
		dma_chainReset(dma);

		nand_op_t op = {
			.cmd = cmd_read,
			.rowAddr = page,
			.colAddr = 0U,
			.transferSize = info->writesz,
			.data = va2pa(data),
			.disEcc = 0U,
			.skipAddr = 0U,
		};

		dma_addDesc(dma, &op);
		(void)dma_run(dma);

		desc = dma_descAt(dma, 0U);

		err = flashdrv_checkDescHwErr(desc);
		if (err < 0) {
			mutexUnlock(common.mutex);
			return err;
		}

		err = flashdrv_checkDescEcc(dma, desc, page, data);
		if (err == -EBADMSG) {
			ret = -EBADMSG;
		}
		else if ((err == -EUCLEAN) && (ret != -EBADMSG)) {
			ret = -EUCLEAN;
		}

		if (meta != NULL) {
			uint32_t deccsts = desc->deccsts;
			unsigned int cec = (deccsts & DECCSTS_CEC_MSK) >> DECCSTS_CEC_SHFT;
			meta->worstBitflips = cec;

			if (err == -EBADMSG) {
				meta->eccState = flash_ecc_uncorrectable;
			}
			else if (cec > 0U) {
				meta->eccState = flash_ecc_corrected;
			}
			else {
				meta->eccState = flash_ecc_ok;
			}
		}
	}

	/* --- Read spare (OOB) area: raw, EDAC disabled --- */
	if (meta != NULL) {
		void *scratch = dma_scratch(dma);

		tagOfs = info->writesz + info->sparesz - info->spareavail;

		dma_chainReset(dma);
		dma_addDesc(dma, cmd_read, page, (uint16_t)tagOfs, info->spareavail, (uint32_t)va2pa(scratch), DCMD_ED);
		(void)dma_run(dma);

		desc = dma_descAt(dma, 0U);
		err = flashdrv_checkDescHwErr(desc);
		if (err < 0) {
			mutexUnlock(common.mutex);
			return err;
		}

		memcpy(meta->metadata, scratch, info->spareavail);
	}

	mutexUnlock(common.mutex);
	return ret;
}


int flashdrv_erase(flashdrv_dma_t *dma, uint32_t paddr)
{
	int err;

	mutexLock(common.mutex);

	dma_chainReset(dma);
	dma_addDesc(dma, cmd_block_erase, paddr, 0U, 0U, 0U, 0U);
	dma_addStatusRead(dma);
	err = dma_runAndCheckStatus(dma);

	mutexUnlock(common.mutex);
	return err;
}


int flashdrv_writeraw(flashdrv_dma_t *dma, uint32_t paddr, uint32_t pageOffs,
		const void *data, size_t size)
{
	unsigned int target = dma->hdr.target;
	const flashdrv_info_t *info = &common.info[target];
	int err;

	if ((pageOffs + size) > (info->writesz + info->sparesz)) {
		return -EINVAL;
	}

	mutexLock(common.mutex);

	dma_chainReset(dma);
	dma_addDesc(dma, cmd_page_program, paddr, (uint16_t)pageOffs, (uint32_t)size, (uint32_t)va2pa(data), DCMD_ED);
	dma_addStatusRead(dma);
	err = dma_runAndCheckStatus(dma);

	mutexUnlock(common.mutex);
	return err;
}


int flashdrv_readraw(flashdrv_dma_t *dma, uint32_t paddr, uint32_t pageOffs,
		void *data, size_t size)
{
	unsigned int target = dma->hdr.target;
	const flashdrv_info_t *info = &common.info[target];
	const nandfctrl2_desc_t *desc;
	int err;

	if ((pageOffs + size) > (info->writesz + info->sparesz)) {
		return -EINVAL;
	}

	mutexLock(common.mutex);

	dma_chainReset(dma);
	dma_addDesc(dma, cmd_read, paddr, (uint16_t)pageOffs, (uint32_t)size, (uint32_t)va2pa(data), DCMD_ED);
	(void)dma_run(dma);

	desc = dma_descAt(dma, 0U);
	err = flashdrv_checkDescHwErr(desc);

	mutexUnlock(common.mutex);
	return err;
}


int flashdrv_isbad(flashdrv_dma_t *dma, uint32_t paddr)
{
	unsigned int target = dma->hdr.target;
	const flashdrv_info_t *info = &common.info[target];
	const nandfctrl2_desc_t *desc;
	void *scratch;
	int err;

	mutexLock(common.mutex);

	scratch = dma_scratch(dma);

	/* Read the bad-block marker byte: first byte of spare area (column = writesz) */
	dma_chainReset(dma);
	dma_addDesc(dma, cmd_read, paddr, (uint16_t)info->writesz, 1U, (uint32_t)va2pa(scratch), DCMD_ED);
	(void)dma_run(dma);

	desc = dma_descAt(dma, 0U);
	err = flashdrv_checkDescHwErr(desc);

	mutexUnlock(common.mutex);

	if (err < 0) {
		return 1; /* Read error: assume bad */
	}

	/* 0x00 in first spare byte = bad block marker */
	return (((const uint8_t *)scratch)[0] == 0x00U) ? 1 : 0;
}


int flashdrv_markbad(flashdrv_dma_t *dma, uint32_t paddr)
{
	unsigned int target = dma->hdr.target;
	const flashdrv_info_t *info = &common.info[target];
	void *scratch;
	int err;

	mutexLock(common.mutex);

	scratch = dma_scratch(dma);
	((uint8_t *)scratch)[0] = 0x00U;

	dma_chainReset(dma);
	dma_addDesc(dma, cmd_page_program, paddr, (uint16_t)info->writesz, 1U, (uint32_t)va2pa(scratch), DCMD_ED);
	dma_addStatusRead(dma);
	err = dma_runAndCheckStatus(dma);

	mutexUnlock(common.mutex);
	return err;
}


int flashdrv_init(void)
{
	unsigned int target;
	flashdrv_dma_t *dma;
	uintptr_t base;
	uintptr_t pageOff;
	int err;

	/* Discover NANDFCTRL2 via AMBA Plug-and-Play */
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

	/* Map APB registers (page-aligned) */
	base = (uintptr_t)dev.info.apb.base;
	pageOff = base & (_PAGE_SIZE - 1U);
	base &= ~(_PAGE_SIZE - 1U);

	void *mapped = mmap(NULL, _PAGE_SIZE + pageOff, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);

	if (mapped == MAP_FAILED) {
		LOG_ERROR("failed to map NANDFCTRL2 registers");
		return -ENOMEM;
	}

	common.regs = (volatile nandfctrl2_regs_t *)((uint8_t *)mapped + pageOff);

	/* Create synchronisation primitives */
	err = mutexCreate(&common.mutex);
	if (err < 0) {
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	err = mutexCreate(&common.wait_mutex);
	if (err < 0) {
		resourceDestroy(common.mutex);
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	err = condCreate(&common.cond);
	if (err < 0) {
		resourceDestroy(common.wait_mutex);
		resourceDestroy(common.mutex);
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	atomic_store(&common.sts1, 0U);
	atomic_store(&common.irqDone, 0);

	/* Register interrupt handler.  The kernel signals fctrl2_common.cond when
	 * the handler returns >= 0. */
	err = interrupt((int)dev.irqn, nandfctrl2_irqHandler, NULL,
			common.cond, &common.irqHandle);
	if (err < 0) {
		LOG_ERROR("failed to register interrupt %u: %d", dev.irqn, err);
		resourceDestroy(common.cond);
		resourceDestroy(common.wait_mutex);
		resourceDestroy(common.mutex);
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	/* Reset controller to known state */
	fctrl2Reset();

	/* Enable LLM + EDAC + BBM preservation */
	common.regs->ctrl0 |= CTRL0_LLM | CTRL0_EE;

	/* Enable interrupts: main + DS + error sources */
	common.regs->ctrl1 = CTRL1_OPS_IRQ_MASK;

	/* Allocate a temporary DMA context for per-die setup (use target 0 as a stub) */
	dma = flashdrv_dmanew(0U);
	if (dma == NULL) {
		LOG_ERROR("failed to allocate init DMA buffer");
		return -ENOMEM;
	}

	for (target = 0U; target < NAND_DIE_CNT; target++) {
		dma->hdr.target = target;
		err = flashdrv_setupDie(dma, target);
		if (err < 0) {
			flashdrv_dmadestroy(dma);
			LOG_ERROR("failed to setup die %u: %d", target, err);
			return err;
		}
	}

	flashdrv_dmadestroy(dma);
	LOG("initialized %u die(s)", NAND_DIE_CNT);
	return 0;
}


const flashdrv_info_t *flashdrv_info(unsigned int target)
{
	if (target >= NAND_DIE_CNT) {
		return NULL;
	}
	return &common.info[target];
}


unsigned int flashdrv_ndies(void)
{
	return NAND_DIE_CNT;
}
