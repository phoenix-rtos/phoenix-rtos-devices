/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver.
 *
 * Low-level userspace driver using DMA Linked List Mode.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdatomic.h>
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
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif

#include "nandfctrl2-flashdrv.h"
#include "nandfctrl2-onfi.h"


/* clang-format off */
#define LOG(str_, ...)       do { printf("nandfctrl2-flash: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG("error: %s:%d: " str_, __FILE__, __LINE__, ##__VA_ARGS__)
/* clang-format on */


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


typedef struct {
	uint8_t manufacturerId;
	uint8_t deviceId;
	uint8_t bytes[3];
} __attribute__((packed)) flash_id_t;


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


/* ============================== Interrupt handler ============================== */


static int irqHandler(unsigned int n, void *data)
{
	(void)n;
	(void)data;

	/* Read and clear all pending interrupt flags */
	uint32_t sts = common.regs->sts1;
	common.regs->sts1 = sts;

	/* Store for main thread inspection */
	common.sts1 = sts;
	common.irqDone = 1;

	return 0;
}


/* ============================== DMA chain helpers ============================== */


static flashdrv_dma_t *dmaAlloc(void)
{
	flashdrv_dma_t *dma;

	dma = mmap(NULL, FLASHDRV_DMA_ALLOC_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (dma == MAP_FAILED) {
		return NULL;
	}

	dma->hdr.ndesc = 0U;

	return dma;
}


static void dmaFree(flashdrv_dma_t *dma)
{
	if ((dma != NULL) && (dma != MAP_FAILED)) {
		(void)munmap(dma, FLASHDRV_DMA_ALLOC_SIZE);
	}
}


static nandfctrl2_desc_t *dmaDescAt(flashdrv_dma_t *dma, unsigned int i)
{
	return &dma->descs[i];
}


static void *dmaScratch(flashdrv_dma_t *dma)
{
	return dma->dataBuf;
}


/* Reset the descriptor chain (reuse the dma buffer for a new operation). */
static void dmaChainReset(flashdrv_dma_t *dma)
{
	dma->hdr.ndesc = 0;
}


/*
 * Add one descriptor to the chain for 'target', filling all routing and
 * command fields.  Returns a pointer to the freshly initialised descriptor.
 */
static void dma_addDesc(flashdrv_dma_t *dma, unsigned int target, const nand_op_t *op)
{
	unsigned int idx = dma->hdr.ndesc;
	const nand_cmd_t *cmd = &nand_commands[op->cmd];
	nandfctrl2_desc_t *desc;
	nandfctrl2_desc_t *prev;

	assert(idx < FLASHDRV_DMA_MAX_DESCS);

	desc = dmaDescAt(dma, idx);
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
		prev = dmaDescAt(dma, idx - 1U);
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
	if (dma->hdr.ndesc == 0U) {
		return 0U;
	}

	/* Mark last descriptor to generate an interrupt */
	dmaDescAt(dma, dma->hdr.ndesc - 1U)->dcmd |= DCMD_IRQ_DS;

	/* Prepare for interrupt */
	common.irqDone = 0;
	common.sts1 = 0U;

	/* Load first descriptor address and trigger */
	common.regs->llpl = (uint32_t)va2pa(dmaDescAt(dma, 0U));
	common.regs->ctrl2 = CTRL2_DT;

	/* Wait for interrupt */
	mutexLock(common.irqLock);
	while (common.irqDone == 0) {
		condWait(common.irqCond, common.irqLock, 0);
	}
	mutexUnlock(common.irqLock);

	uint32_t sts = common.sts1;
	return sts;
}


/* ============================== Status helpers ============================== */


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

	if ((dsts & (DSTS_UCERR_DL | DSTS_UCERR_UL | DSTS_ABORT | DSTS_STOP_LL | DSTS_INV)) != 0U) {
		return -EIO;
	}

	return 0;
}


/*
 * Add a read_status descriptor to the chain.
 * The status byte is DMA'd to the scratch buffer.
 * SRB=0 so controller waits for R/B# before issuing the status command.
 */
static void dma_addStatusRead(flashdrv_dma_t *dma, unsigned int target)
{
	void *scratch = dmaScratch(dma);
	nand_op_t op = {
		.cmd = cmd_read_status,
		.rowAddr = 0U,
		.colAddr = 0U,
		.transferSize = 1U,
		.data = (uint32_t)va2pa(scratch),
		.skipAddr = 1U,
		.disEcc = 1U,
	};

	dma_addDesc(dma, target, &op);
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

	last = dmaDescAt(dma, ndesc - 1U);

	err = flashdrv_checkDescHwErr(last);
	if (err < 0) {
		return err;
	}

	scratch = (const uint8_t *)dmaScratch(dma);
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


static void resetNandfctrl2(void)
{
	common.regs->ctrl2 = CTRL2_ABORT;

	/* Wait for abort + ready */
	while (((common.regs->sts1 & STS1_ABORT) == 0U) || ((common.regs->sts0 & STS0_RDY) == 0U)) { }

	/* Clear status register */
	common.regs->sts1 = common.regs->sts1;

	common.regs->ctrl2 = CTRL2_RST;

	/* Preserve BBM; LLM and EE will be set after timing configuration */
	common.regs->ctrl0 = CTRL0_BBM;
	common.regs->ctrl1 = 0U;
	common.regs->ctrl3 = 0U; /* Disable write protection */
	common.regs->ctrl4 = 0U;

	/* Timing mode 0 until ONFI page is read */
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


/* Probe die via ONFI, configure shared timing registers, fill the info struct. */
static int setupFlash(flashdrv_dma_t *dma, unsigned int target, flashdrv_info_t *info)
{
	unsigned int timingMode;
	bool edoEn;
	int err;

	/* Reset die */
	dmaChainReset(dma);

	nand_op_t op = {
		.cmd = cmd_reset,
		.rowAddr = 0U,
		.colAddr = 0U,
		.transferSize = 0U,
		.data = 0U,
		.skipAddr = 1U,
		.disEcc = 1U,
	};
	dma_addDesc(dma, target, &op);

	op.cmd = cmd_read_id;
	op.rowAddr = 0U;
	op.colAddr = 0U;
	op.transferSize = sizeof(flash_id_t);
	op.data = (addr_t)dmaScratch(dma);
	op.skipAddr = 0U;
	op.disEcc = 1U;

	dma_addDesc(dma, target, &op);
	(void)dma_run(dma);

	flash_id_t *flashId = (flash_id_t *)dmaScratch(dma);

	if ((flashId->manufacturerId == 0x2cU) && (flashId->deviceId == 0xacU)) {
		info->name = "Micron MT29F4G08ABBFA";
	}
	else {
		info->name = "Unknown NAND flash";
	}

	/* Read ONFI parameter page */
	dmaChainReset(dma);

	op.cmd = cmd_read_parameter_page;
	op.rowAddr = 0U;
	op.colAddr = 0U;
	op.transferSize = sizeof(onfi_paramPage_t);
	op.data = (addr_t)va2pa(dmaScratch(dma));
	op.skipAddr = 0U;
	op.disEcc = 1U;

	dma_addDesc(dma, target, &op);
	(void)dma_run(dma);

	onfi_paramPage_t page;
	onfi_deserializeParamPage(&page, dmaScratch(dma));

	if ((page.signature[0] != 'O') || (page.signature[1] != 'N') || (page.signature[2] != 'F') || (page.signature[3] != 'I')) {
		LOG_ERROR("no ONFI signature on die %u", target);
		return -EIO;
	}

	timingMode = onfi_calcTimingMode(&page);
	edoEn = timingMode > 3;

	setOnfiTimingMode(timingMode, &page, SYSCLK_FREQ, edoEn);

	err = setMemorySelect(page.bytesPerPage, page.spareBytesPerPage);
	if (err < 0) {
		return err;
	}

	info->writesz = page.bytesPerPage;
	info->sparesz = page.spareBytesPerPage;
	info->spareavail = NAND_SPARE_AVAIL;
	info->pagesPerBlock = page.pagesPerBlock;
	info->erasesz = page.bytesPerPage * page.pagesPerBlock;
	info->size = (uint64_t)info->erasesz * page.blocksPerLun * page.numLuns;

	return 0;
}


/* ============================== Public API ============================== */


nand_die_t *flashdrv_dieAlloc(unsigned int target)
{
	nand_die_t *die;

	if (target >= NAND_DIE_CNT) {
		return NULL;
	}

	die = malloc(sizeof(*die));
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


int flashdrv_reset(nand_die_t *die)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	dmaChainReset(dma);

	nand_op_t op = {
		.cmd = cmd_reset,
		.rowAddr = 0U,
		.colAddr = 0U,
		.skipAddr = 1U,
		.disEcc = 1U,
		.transferSize = 0U,
		.data = 0U,
	};
	dma_addDesc(dma, target, &op);
	(void)dma_run(dma);

	int err = flashdrv_checkDescHwErr(dmaDescAt(dma, 0U));

	mutexUnlock(common.hwLock);
	return err;
}


int flashdrv_write(nand_die_t *die, uint32_t paddr, const void *data)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	nand_op_t op = {
		.cmd = cmd_page_program,
		.rowAddr = paddr,
		.colAddr = 0U,
		.transferSize = die->info.writesz,
		.data = (addr_t)va2pa((void *)data),
		.skipAddr = 0U,
		.disEcc = 0U,
	};
	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	dma_addStatusRead(dma, target);
	int err = dma_runAndCheckStatus(dma);

	mutexUnlock(common.hwLock);
	return err;
}


int flashdrv_read(nand_die_t *die, uint32_t page, void *data, flashdrv_eccStatus_t *eccStatus)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	dmaChainReset(dma);

	nand_op_t op = {
		.cmd = cmd_read,
		.rowAddr = page,
		.colAddr = 0U,
		.transferSize = die->info.writesz,
		.data = va2pa(data),
		.disEcc = 0U,
		.skipAddr = 0U,
	};

	dma_addDesc(dma, target, &op);
	(void)dma_run(dma);

	const nandfctrl2_desc_t *desc = dmaDescAt(dma, 0U);

	int err = flashdrv_checkDescHwErr(desc);
	if (err < 0) {
		mutexUnlock(common.hwLock);
		return err;
	}

	uint32_t cec = (desc->deccsts & DECCSTS_CEC_MSK) >> DECCSTS_CEC_SHFT;
	eccStatus->worstBitflips = cec;

	if ((desc->dsts & (DSTS_UE | DSTS_ECFAIL_MSK)) != 0U) {
		eccStatus->eccState = flash_ecc_uncorrectable;
	}
	else if (cec > 0U) {
		eccStatus->eccState = flash_ecc_corrected;
	}
	else {
		eccStatus->eccState = flash_ecc_ok;
	}


	mutexUnlock(common.hwLock);
	return err;
}


int nandfctrl2_writemeta(nand_die_t *die, uint32_t page, const void *data, size_t size)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;
	uint32_t tagOfs = die->info.writesz + die->info.sparesz - die->info.spareavail;

	nand_op_t op = {
		.cmd = cmd_page_program,
		.rowAddr = page,
		.colAddr = (uint16_t)tagOfs,
		.transferSize = size,
		.data = (addr_t)va2pa((void *)data),
		.skipAddr = 0U,
		.disEcc = 1U,
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	dma_addStatusRead(dma, target);
	int err = dma_runAndCheckStatus(dma);

	mutexUnlock(common.hwLock);
	return err;
}


int nandfctrl2_readmeta(nand_die_t *die, uint32_t page, void *data, size_t size)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;
	uint32_t tagOfs = die->info.writesz + die->info.sparesz - die->info.spareavail;

	nand_op_t op = {
		.cmd = cmd_read,
		.rowAddr = page,
		.colAddr = (uint16_t)tagOfs,
		.transferSize = size,
		.data = (addr_t)va2pa(data),
		.skipAddr = 0U,
		.disEcc = 1U,
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	(void)dma_run(dma);

	const nandfctrl2_desc_t *desc = dmaDescAt(dma, 0U);
	int err = flashdrv_checkDescHwErr(desc);

	mutexUnlock(common.hwLock);
	return err;
}


int flashdrv_writeraw(nand_die_t *die, uint32_t paddr, uint32_t pageOffs, const void *data, size_t size)
{
	if ((pageOffs + size) > (die->info.writesz + die->info.sparesz)) {
		return -EINVAL;
	}

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	nand_op_t op = {
		.cmd = cmd_page_program,
		.rowAddr = paddr,
		.colAddr = (uint16_t)pageOffs,
		.transferSize = size,
		.data = (addr_t)va2pa((void *)data),
		.skipAddr = 0U,
		.disEcc = 1U,
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	dma_addStatusRead(dma, target);
	int err = dma_runAndCheckStatus(dma);

	mutexUnlock(common.hwLock);
	return err;
}


int flashdrv_readraw(nand_die_t *die, uint32_t paddr, uint32_t pageOffs, void *data, size_t size)
{
	if ((pageOffs + size) > (die->info.writesz + die->info.sparesz)) {
		return -EINVAL;
	}

	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;


	nand_op_t op = {
		.cmd = cmd_read,
		.rowAddr = paddr,
		.colAddr = (uint16_t)pageOffs,
		.transferSize = size,
		.data = (addr_t)va2pa(data),
		.skipAddr = 0U,
		.disEcc = 1U,
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	(void)dma_run(dma);

	const nandfctrl2_desc_t *desc = dmaDescAt(dma, 0U);
	int err = flashdrv_checkDescHwErr(desc);

	mutexUnlock(common.hwLock);
	return err;
}


int flashdrv_erase(nand_die_t *die, uint32_t paddr)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	nand_op_t op = {
		.cmd = cmd_block_erase,
		.rowAddr = paddr,
		.colAddr = 0U,
		.transferSize = 0U,
		.data = 0U,
		.skipAddr = 0U,
		.disEcc = 1U,
	};

	mutexLock(common.hwLock);

	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	dma_addStatusRead(dma, target);
	int err = dma_runAndCheckStatus(dma);

	mutexUnlock(common.hwLock);
	return err;
}


int flashdrv_isbad(const nand_die_t *die, uint32_t paddr)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	void *scratch = dmaScratch(dma);

	/* Read the bad-block marker byte: first byte of spare area (column = writesz) */
	nand_op_t op = {
		.cmd = cmd_read,
		.rowAddr = paddr,
		.colAddr = (uint16_t)die->info.writesz,
		.transferSize = 1U,
		.data = (addr_t)va2pa(scratch),
		.skipAddr = 0U,
		.disEcc = 1U,
	};
	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	(void)dma_run(dma);

	const nandfctrl2_desc_t *desc = dmaDescAt(dma, 0U);
	int err = flashdrv_checkDescHwErr(desc);

	mutexUnlock(common.hwLock);

	if (err < 0) {
		return 1; /* Read error: assume bad */
	}

	/* 0x00 in first spare byte = bad block marker */
	return (((const uint8_t *)scratch)[0] == 0x00U) ? 1 : 0;
}


int flashdrv_markbad(const nand_die_t *die, uint32_t paddr)
{
	flashdrv_dma_t *dma = die->dma;
	unsigned int target = die->target;

	mutexLock(common.hwLock);

	void *scratch = dmaScratch(dma);
	((uint8_t *)scratch)[0] = 0x00U;

	nand_op_t op = {
		.cmd = cmd_page_program,
		.rowAddr = paddr,
		.colAddr = (uint16_t)die->info.writesz,
		.transferSize = 1U,
		.data = (addr_t)va2pa(scratch),
		.skipAddr = 0U,
		.disEcc = 1U,
	};
	dmaChainReset(dma);
	dma_addDesc(dma, target, &op);
	dma_addStatusRead(dma, target);
	int err = dma_runAndCheckStatus(dma);

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

	void *mapped = mmap(NULL, _PAGE_SIZE + pageOff, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);

	if (mapped == MAP_FAILED) {
		LOG_ERROR("failed to map NANDFCTRL2 registers");
		return -ENOMEM;
	}

	common.regs = (volatile nandfctrl2_regs_t *)((uint8_t *)mapped + pageOff);

	int err = mutexCreate(&common.hwLock);
	if (err < 0) {
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	err = mutexCreate(&common.irqLock);
	if (err < 0) {
		resourceDestroy(common.hwLock);
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	err = condCreate(&common.irqCond);
	if (err < 0) {
		resourceDestroy(common.irqLock);
		resourceDestroy(common.hwLock);
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	err = interrupt(dev.irqn, irqHandler, NULL, common.irqCond, &common.irqHandle);
	if (err < 0) {
		LOG_ERROR("failed to register interrupt %u: %d", dev.irqn, err);
		resourceDestroy(common.irqCond);
		resourceDestroy(common.irqLock);
		resourceDestroy(common.hwLock);
		(void)munmap(mapped, _PAGE_SIZE + pageOff);
		return err;
	}

	/* Reset controller to known state */
	resetNandfctrl2();

	/* Enable LLM + EDAC + BBM preservation */
	common.regs->ctrl0 |= CTRL0_LLM | CTRL0_EE;

	/* Enable interrupts: main + DS + error sources */
	common.regs->ctrl1 = CTRL1_OPS_IRQ_MASK;

	/* Allocate a temporary DMA buffer for die probing */
	flashdrv_dma_t *dma = dmaAlloc();
	if (dma == NULL) {
		LOG_ERROR("failed to allocate init DMA buffer");
		return -ENOMEM;
	}

	/* Probe die 0 for ONFI geometry and configure timing registers */
	err = setupFlash(dma, 0U, &common.info);
	if (err < 0) {
		dmaFree(dma);
		LOG_ERROR("failed to setup die 0: %d", err);
		return err;
	}

	/* Reset remaining dies (same geometry) */
	for (unsigned int target = 1U; target < NAND_DIE_CNT; target++) {
		nand_op_t op = {
			.cmd = cmd_reset,
			.skipAddr = 1U,
			.disEcc = 1U,
		};
		dmaChainReset(dma);
		dma_addDesc(dma, target, &op);
		(void)dma_run(dma);
	}

	dmaFree(dma);

	LOG("initialized %u die(s)", NAND_DIE_CNT);

	return 0;
}
