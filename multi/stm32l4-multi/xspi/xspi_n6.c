/*
 * Phoenix-RTOS
 *
 * STM32 XSPI Flash driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "xspi_n6.h"

#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/pwman.h>
#include <sys/time.h>
#include <libmulti/libdma.h>

#include "../common.h"
#include "../stm32l4-multi.h"
#include "../ext_flash.h"

#include <board_config.h>


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#include "chip_setup.h"

#define XSPI1_REG_SIZE (256 * 1024 * 1024)
#define XSPI2_REG_SIZE (256 * 1024 * 1024)
#define XSPI3_REG_SIZE (256 * 1024 * 1024)

#define REGION_NOT_ENCRYPTED XSPI_MCE_REGIONS

#define CTRL_SHIFT 2


enum {
	ipclk_sel_hclk5 = 0x0,
	ipclk_sel_per_ck = 0x1,
	ipclk_sel_ic3_ck = 0x2,
	ipclk_sel_ic4_ck = 0x3,
};


static const xspi_ctrlParams_t xspi_ctrlParams[XSPI_N_CONTROLLERS] = {
	{
		.start = XSPI2_REG_BASE,
		.size = XSPI2_REG_SIZE,
		.clksel = { .sel = pctl_ipclk_xspi2sel, .val = ipclk_sel_hclk5 },
		.divider_slow = 6,
		.divider = XSPI2_CLOCK_DIV,
		.dev = pctl_xspi2,
		.irq = xspi2_irq,
		.ctrl = XSPI2_BASE,
		.resetPin = { pctl_gpion, 12 },
		.enabled = XSPI2_STORAGE,
		.spiPort = XSPIM_PORT2,
		.chipSelect = XSPI_CHIPSELECT_NCS1,
		.isHyperbus = XSPI2_IS_HYPERBUS,
		.mceDev = pctl_mce2,
	},
};


typedef struct {
	uint32_t ccr;
	uint32_t tcr;
	uint32_t ir;
} xspi_cmdRegs_t;


typedef struct {
	xspi_cmdRegs_t reg;
	u32 addr;
	u32 dataLen;
	u8 isRead;
} flash_opDefinition_t;


static struct {
	struct xspi_ctrlSetup {
		bool configured;
		handle_t mutex;
		handle_t irq;
		handle_t irq_mutex;
		handle_t irq_cond;
		const struct libdma_per *dma;
		struct xspi_mem {
			char name[32];
			uint8_t jedecID[6];
			uint8_t log_chipSize;
			uint8_t log_eraseSize;
			uint8_t log_pageSize;
			uint32_t eraseTimeoutMs;
			uint32_t chipEraseTimeoutMs;
			xspi_cmdRegs_t read;
			xspi_cmdRegs_t write;
			xspi_cmdRegs_t erase;
			flash_opDefinition_t chipErase;
			flash_opDefinition_t writeEnable;
			flash_opDefinition_t writeDisable;
			flash_opDefinition_t readStatus;
		} mem;
		struct encr_region {
			u8 enabled;
			size_t granularity;
			size_t start;
			size_t end;
		} region[XSPI_MCE_REGIONS];
	} ctrl[XSPI_N_CONTROLLERS];
} xspi_common = { 0 };


static inline int getDevClk(int dev)
{
	platformctl_t pctl;
	pctl.action = pctl_get;
	pctl.type = pctl_devclk;
	pctl.devclk.dev = dev;
	if (platformctl(&pctl) < 0) {
		return 0;
	}

	return pctl.devclk.state;
}


/* Note: currently libphoenix doesn't implement pthread_rwlock*, so for now we use regular mutexes
 * to implement mutual exclusion. The function is there in case additional actions need to be taken
 * when switching controller modes (maybe some way to block accesses to Flash memory?). */
static int xspi_rwlock_acquire(unsigned int minor, bool doWrite)
{
	(void)doWrite;
	return mutexLock(xspi_common.ctrl[minor].mutex);
}


static void xspi_rwlock_release(unsigned int minor)
{
	mutexUnlock(xspi_common.ctrl[minor].mutex);
}


static inline bool xspi_isValidMinor(unsigned int minor)
{
	return (minor < XSPI_N_CONTROLLERS) && (xspi_ctrlParams[minor].enabled != 0) && (xspi_common.ctrl[minor].configured);
}


/* -1 - address range crosses a region boundary
 * [0 .. (XSPI_MCE_REGIONS - 1)] - number of encrypted region
 *  REGION_NOT_ENCRYPTED - not encrypted
 */
static int xspi_getRegion(unsigned int minor, u32 off, size_t end)
{
	struct encr_region *r;
	int i;
	for (i = 0; i < XSPI_MCE_REGIONS; i++) {
		r = &xspi_common.ctrl[minor].region[i];
		if (r->enabled == 0) {
			continue;
		}

		if (((off < r->start) && (end > r->start)) || ((off < r->end) && (end > r->end))) {
			return -1;
		}

		if ((r->start <= off) && (end <= r->end)) {
			return i;
		}
	}

	return REGION_NOT_ENCRYPTED;
}


static int xspi_validateAndGetRegion(unsigned int minor, u32 off, size_t size)
{
	size_t fsize;
	size_t end = off + size;
	int region;

	if (!xspi_isValidMinor(minor)) {
		return -ENODEV;
	}

	if (end < off) {
		/* Integer overflow has occurred */
		return -EINVAL;
	}

	fsize = 1UL << xspi_common.ctrl[minor].mem.log_chipSize;
	if (!((off < fsize) && ((off + size) <= fsize))) {
		return -EINVAL;
	}

	region = xspi_getRegion(minor, off, end);
	if (region < 0) {
		return -EINVAL;
	}

	return region;
}


static inline void xspi_waitBusy(unsigned int minor)
{
	while ((*(xspi_ctrlParams[minor].ctrl + xspi_sr) & XSPI_SR_BUSY) != 0) {
		/* Wait for controller to become ready */
	}
}


/* Transfer data in indirect read or write mode using the FIFO.
 * `data`: output (if isRead == 0) or input (if isRead != 0) data buffer
 * `len`: length of data to transfer
 * `isRead`: == 0 if this is a write operation, != 0 otherwise */
int xspi_transferFifo(unsigned int minor, u8 *data, size_t len, u8 isRead)
{
	const xspi_ctrlParams_t *p = &xspi_ctrlParams[minor];
	int i = 0, j = 0;
	u32 status;
	time_t deadline;

	while (i < len) {
		status = *(p->ctrl + xspi_sr);
		/* Optimization - check status first, then calculate deadline.
		 * Because XSPI is very fast we can get bytes in FIFO within a few CPU clock cycles. */
		if ((status & (XSPI_SR_FTF | XSPI_SR_TCF)) == 0) {
			/* Timeout is very generous - at least 1 ms per byte. Device should never time out
			 * unless there is a hardware failure. */
			time_t now;
			gettime(&now, NULL);
			deadline = now + 1000;
			do {
				gettime(&now, NULL);
				status = *(p->ctrl + xspi_sr);
				if (now > deadline) {
					*(p->ctrl + xspi_cr) |= (1 << 1); /* Abort operation in progress */
					xspi_waitBusy(minor);
					return -ETIME;
				}
			} while ((status & (XSPI_SR_FTF | XSPI_SR_TCF)) == 0);
		}

		if (isRead != 0) {
			j = XSPI_FIFO_SIZE - ((status >> 8) & 0x7f);
			for (; j < XSPI_FIFO_SIZE && (i < len); j++, i++) {
				/* This controller allows byte and halfword reads from the XSPI_DR register */
				data[i] = *(volatile u8 *)(p->ctrl + xspi_dr);
			}
		}
		else {
			j = (status >> 8) & 0x7f;
			/* In indirect write mode, writing data triggers the operation */
			for (; j < XSPI_FIFO_SIZE && (i < len); j++, i++) {
				/* This controller allows byte and halfword writes to the XSPI_DR register */
				*(volatile u8 *)(p->ctrl + xspi_dr) = data[i];
			}
		}
	}

	while ((*(p->ctrl + xspi_sr) & XSPI_SR_TCF) == 0) {
		/* Wait for transfer completion */
	}

	*(p->ctrl + xspi_fcr) = XSPI_SR_TCF;
	return EOK;
}


static inline int xspi_opHasAddr(const flash_opDefinition_t *opDef)
{
	return ((opDef->reg.ccr >> 8) & 0x7) != 0;
}


static u32 xspi_changeCtrlMode(unsigned int minor, u32 new_mode, int doMemWriting)
{
	const xspi_ctrlParams_t *p = &xspi_ctrlParams[minor];
	struct xspi_mem *mem = &xspi_common.ctrl[minor].mem;
	u32 prev_mode, v;

	v = *(p->ctrl + xspi_cr);
	prev_mode = (v & XSPI_CR_MODE_MASK);
	if (prev_mode == new_mode) {
		return prev_mode;
	}

	if ((prev_mode == XSPI_CR_MODE_MEMORY) || (prev_mode == XSPI_CR_MODE_AUTOPOLL)) {
		dataBarier();
		*(p->ctrl + xspi_cr) = v | (1 << 1); /* Abort operation in progress */
		xspi_waitBusy(minor);
	}

	v &= ~XSPI_CR_MODE_MASK;
	if (new_mode == XSPI_CR_MODE_MEMORY) {
		*(p->ctrl + xspi_cr) = v;
		dataBarier();
		xspi_waitBusy(minor);

		*(p->ctrl + xspi_ccr) = mem->read.ccr;
		*(p->ctrl + xspi_tcr) = mem->read.tcr;
		*(p->ctrl + xspi_ir) = mem->read.ir;
		/* Clear prefetch and timeout bits */
		v &= ~(XSPI_CR_NOPREF_AXI | XSPI_CR_NOPREF | XSPI_CR_TCEN);
		if (doMemWriting != 0) {
			*(p->ctrl + xspi_wccr) = mem->write.ccr;
			*(p->ctrl + xspi_wtcr) = mem->write.tcr;
			*(p->ctrl + xspi_wir) = mem->write.ir;
			/* Leave prefetch and timeout bits cleared to enable prefetch and disable timeout */
		}
		else {
			/* Disable writing. Attempting to write will cause an exception. */
			*(p->ctrl + xspi_wccr) = 0;
			v |= XSPI_DEFAULT_PREFETCH | XSPI_DEFAULT_TIMEOUT;
		}

		dataBarier();
	}

	v |= new_mode;
	*(p->ctrl + xspi_cr) = v;
	xspi_waitBusy(minor);

	/* Clear any flags that may have been set (e.g. in autopoll or memory-mapped mode) */
	*(p->ctrl + xspi_fcr) = XSPI_SR_TCF | XSPI_SR_SMF;
	return prev_mode;
}


static int xspi_performOp(unsigned int minor, const flash_opDefinition_t *opDef, unsigned char *data)
{
	const xspi_ctrlParams_t *p = &xspi_ctrlParams[minor];
	xspi_changeCtrlMode(minor, (opDef->isRead != 0) ? XSPI_CR_MODE_IREAD : XSPI_CR_MODE_IWRITE, 0);
	if (opDef->dataLen != 0) {
		*(p->ctrl + xspi_dlr) = opDef->dataLen - 1;
	}

	*(p->ctrl + xspi_ccr) = opDef->reg.ccr;
	*(p->ctrl + xspi_tcr) = opDef->reg.tcr;
	/* If indirect read mode and no address, this write triggers the operation */
	*(p->ctrl + xspi_ir) = opDef->reg.ir;
	if (xspi_opHasAddr(opDef)) {
		/* If indirect read mode with address, this write triggers the operation */
		*(p->ctrl + xspi_ar) = opDef->addr;
	}

	return xspi_transferFifo(minor, data, opDef->dataLen, opDef->isRead);
}


static int xspi_waitForWriteCompletion(unsigned int minor, const flash_opDefinition_t *opDef, u32 timeoutMs)
{
	int ret = 0;
	const xspi_ctrlParams_t *p = &xspi_ctrlParams[minor];
	struct xspi_ctrlSetup *s = &xspi_common.ctrl[minor];

	xspi_changeCtrlMode(minor, XSPI_CR_MODE_AUTOPOLL, 0);
	*(p->ctrl + xspi_psmkr) = 0x01; /* Check bit 0 */
	*(p->ctrl + xspi_psmar) = 0x00; /* Check until it's cleared */
	*(p->ctrl + xspi_pir) = 0x10;
	*(p->ctrl + xspi_dlr) = opDef->dataLen - 1;
	*(p->ctrl + xspi_ccr) = opDef->reg.ccr;
	*(p->ctrl + xspi_tcr) = opDef->reg.tcr;
	/* If command has no address, writing XSPI_IR starts the operation */
	*(p->ctrl + xspi_ir) = opDef->reg.ir;
	if (xspi_opHasAddr(opDef)) {
		/* If command has address, writing XSPI_AR starts the operation */
		*(p->ctrl + xspi_ar) = opDef->addr;
	}

	mutexLock(s->irq_mutex);
	*(p->ctrl + xspi_cr) |= XSPI_CR_SMIE;
	time_t now, end;
	gettime(&now, NULL);
	end = now + timeoutMs;
	while (1) {
		condWait(s->irq_cond, s->irq_mutex, (time_t)timeoutMs * 1000);
		if ((*(p->ctrl + xspi_sr) & XSPI_SR_SMF) != 0) {
			break;
		}

		gettime(&now, NULL);
		if (end <= now) {
			*(p->ctrl + xspi_cr) |= (1 << 1); /* Abort auto-polling */
			ret = -ETIME;
			break;
		}
	}

	mutexUnlock(s->irq_mutex);
	*(p->ctrl + xspi_fcr) = XSPI_SR_SMF;
	return ret;
}


static int xspi_writeEnable(unsigned int minor, bool enable)
{
	/* Depending on command status may be 1 or 2 bytes long.
	 * We only care about the first byte, but need to allocate a large enough buffer. */
	u8 status[2];
	const flash_opDefinition_t *opDef =
			enable ?
			&xspi_common.ctrl[minor].mem.writeEnable :
			&xspi_common.ctrl[minor].mem.writeDisable;
	unsigned retries = 10;
	int ret;
	enable = (enable != 0) ? 1 : 0;

	/* Set flag and verify until it's set - required according to Macronix datasheet */
	do {
		if (retries == 0) {
			return -EIO;
		}

		retries--;
		ret = xspi_performOp(minor, opDef, NULL);
		if (ret < 0) {
			return ret;
		}

		ret = xspi_performOp(minor, &xspi_common.ctrl[minor].mem.readStatus, status);
		if (ret < 0) {
			return ret;
		}
	} while (((status[0] >> 1) & 1) != enable);

	return EOK;
}


/* Perform a write-like operation (program or erase) */
static ssize_t xspi_performWriteOp(
		unsigned int minor,
		const flash_opDefinition_t *opDef,
		const void *data,
		u32 timeout)
{
	int ret;
	if (xspi_writeEnable(minor, 1) < 0) {
		return -EIO;
	}

	ret = xspi_performOp(minor, opDef, (void *)data);
	if (ret < 0) {
		return ret;
	}

	return xspi_waitForWriteCompletion(minor, &xspi_common.ctrl[minor].mem.readStatus, timeout);
}


static int xspi_invalCache(const void *addr, size_t sz)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_invalDCache;
	pctl.opDCache.addr = (void *)addr;
	pctl.opDCache.sz = sz;
	return platformctl(&pctl);
}


static int xspi_memcpy(const struct libdma_per *per, void *dst, const void *src, size_t len)
{
	int ret = 0;
	/* Here we optimize the transfer - we pick a maximum data width for both source and destination
	 * depending on pointer alignment and transfer length (up to 8 bytes).
	 * Transfer length must be a multiple of burst length and AXI supports max 16 data units.
	 * If MCE is in block mode we assume that destination and length are multiples of 16
	 * (it should have been verified previously), so logDat_dst == 3, burst_dst == 2.
	 */
	size_t logDat_dst = min(__builtin_ctz((addr_t)dst | len), 3);
	size_t logDat_src = min(__builtin_ctz((addr_t)src | len), 3);
	size_t logBurst = min(__builtin_ctz(len), 4);
	/* Calculate how many data units are to a burst. */
	uint8_t burst_src = 1U << (logBurst - logDat_src);
	uint8_t burst_dst = 1U << (logBurst - logDat_dst);

	libdma_peripheral_config_t cfg = {
		.addr = dst,
		.elSize_log = logDat_dst,
		.burstSize = burst_dst,
		.increment = 1,
	};

	libdma_transfer_buffer_t bufs[2];
	bufs[0] = (libdma_transfer_buffer_t) {
		.buf = (void *)src,
		.bufSize = len,
		.elSize_log = logDat_src,
		.burstSize = burst_src,
		.increment = 1,
		.isCached = 1,
		/* Packed mode ensures the same byte sequence will result regardless of source/destination element size. */
		.transform = LIBXPDMA_TRANSFORM_PACK,
	};
	size_t nBufs = 1;

	/* If ending is misaligned, handle the write-end bug */
	void *end = dst + len;
	u32 misalign = 4 - ((addr_t)end % 4);
	u8 misalignData[4];
	if (misalign < 4) {
		/* This read must occur before any writes */
		memcpy(misalignData, end, misalign);
		bufs[1] = (libdma_transfer_buffer_t) {
			.buf = (void *)misalignData,
			.bufSize = misalign,
			.elSize_log = 0,
			.burstSize = 1,
			.increment = 1,
			.isCached = 1,
			/* Packed mode ensures the same byte sequence will result regardless of source/destination element size. */
			.transform = LIBXPDMA_TRANSFORM_PACK,
		};
		nBufs = 2;
	}

	/* TODO: add option to wait without interrupts */
	volatile int done;
	ret = (ret < 0) ? ret : libxpdma_configurePeripheral(per, dma_mem2per, &cfg);
	ret = (ret < 0) ? ret : libxpdma_configureMemory(per, dma_mem2per, 0, bufs, nBufs);
	ret = (ret < 0) ? ret : libxpdma_startTransferWithFlag(per, dma_mem2per, &done);
	ret = (ret < 0) ? ret : libxpdma_waitForTransaction(per, &done, NULL, 0);
	ret = (ret < 0) ? ret : xspi_invalCache(dst, len);
	return ret;
}


int xspi_isr(unsigned int irq, void *arg)
{
	unsigned int minor = (unsigned int)arg;
	u32 status = *(xspi_ctrlParams[minor].ctrl + xspi_sr);
	status &= (XSPI_SR_TOF | XSPI_SR_SMF | XSPI_SR_FTF | XSPI_SR_TCF | XSPI_SR_TEF);
	*(xspi_ctrlParams[minor].ctrl + xspi_cr) &= ~(status << 16);
	return 1;
}


static void xspi_chipSetup_copyCmdRegs(xspi_cmdRegs_t *mem, struct xspi_commandRegs_v1 *cs)
{
	mem->ccr = cs->ccr;
	mem->tcr = cs->tcr;
	mem->ir = cs->ir;
}


static void xspi_chipSetup_deserialize(unsigned int minor, struct xspi_chipSetup_v1 *cs)
{
	struct xspi_mem *mem = &xspi_common.ctrl[minor].mem;
	mem->log_chipSize = cs->log_chipSize;
	mem->log_eraseSize = cs->log_eraseSize;
	mem->log_pageSize = cs->log_pageSize;
	mem->eraseTimeoutMs = cs->eraseTimeoutMs;
	mem->chipEraseTimeoutMs = cs->chipEraseTimeoutMs;
	strncpy(mem->name, cs->name, min(sizeof(mem->name), sizeof(cs->name)));
	mem->name[sizeof(mem->name) - 1] = '\0';
	memcpy(mem->jedecID, cs->jedecID, sizeof(mem->jedecID));
	xspi_chipSetup_copyCmdRegs(&mem->read, &cs->read);
	xspi_chipSetup_copyCmdRegs(&mem->write, &cs->write);
	xspi_chipSetup_copyCmdRegs(&mem->erase, &cs->erase);

	xspi_chipSetup_copyCmdRegs(&mem->chipErase.reg, &cs->chipErase);
	mem->chipErase.addr = 0;
	mem->chipErase.dataLen = 0;
	mem->chipErase.isRead = 0;

	xspi_chipSetup_copyCmdRegs(&mem->writeEnable.reg, &cs->writeEnable);
	mem->writeEnable.addr = 0;
	mem->writeEnable.dataLen = 0;
	mem->writeEnable.isRead = 0;

	xspi_chipSetup_copyCmdRegs(&mem->writeDisable.reg, &cs->writeDisable);
	mem->writeDisable.addr = 0;
	mem->writeDisable.dataLen = 0;
	mem->writeDisable.isRead = 0;

	xspi_chipSetup_copyCmdRegs(&mem->readStatus.reg, &cs->readStatus);
	mem->readStatus.addr = cs->readStatus_addr;
	mem->readStatus.dataLen = cs->readStatus_dataLen;
	mem->readStatus.isRead = 1;
}


static int xspi_chipSetup_readFromFile(unsigned int minor, struct xspi_chipSetup_v1 *cs)
{
	char memconfig_path[32];
	unsigned int portNum;
	switch (xspi_ctrlParams[minor].spiPort) {
		case XSPIM_PORT1: portNum = 1; break;
		case XSPIM_PORT2: portNum = 2; break;
		default: return -EINVAL;
	}

	int ret = snprintf(memconfig_path, sizeof(memconfig_path), "/syspage/" FLASHCS_FORMAT, portNum);
	if (ret >= sizeof(memconfig_path)) {
		return -ENAMETOOLONG;
	}

	FILE *memconfig_file = fopen(memconfig_path, "rb");
	if (memconfig_file == NULL) {
		return -ENOENT;
	}

	if (fread(&cs->version, sizeof(cs->version), 1, memconfig_file) != 1) {
		fclose(memconfig_file);
		return -EIO;
	}

	if (cs->version != CHIP_SETUP_VER_1) {
		return -ENOTSUP;
	}

	if (fseek(memconfig_file, 0, SEEK_SET) < 0) {
		fclose(memconfig_file);
		return -EIO;
	}

	if (fread(cs, sizeof(*cs), 1, memconfig_file) != 1) {
		fclose(memconfig_file);
		return -EIO;
	}

	fclose(memconfig_file);

	return EOK;
}


static int xspi_chipSetup_readFile(unsigned int minor)
{
	struct xspi_chipSetup_v1 *cs;
	cs = malloc(sizeof(*cs));
	if (cs == NULL) {
		return -ENOMEM;
	}

	int ret = xspi_chipSetup_readFromFile(minor, cs);
	if (ret < 0) {
		free(cs);
		return ret;
	}


	xspi_chipSetup_deserialize(minor, cs);
	free(cs);
	cs = NULL;
	return EOK;
}


static int xspi_initOne(unsigned int minor)
{
	if (minor >= XSPI_N_CONTROLLERS) {
		return -EINVAL;
	}

	const xspi_ctrlParams_t *p = &xspi_ctrlParams[minor];
	struct xspi_ctrlSetup *s = &xspi_common.ctrl[minor];
	if (s->configured) {
		return -EALREADY;
	}

	if (p->enabled == 0) {
		return -ENODEV;
	}

	/* Currently we don't have support for Hyperbus Flash devices */
	if (p->isHyperbus != 0) {
		return -ENODEV;
	}

	if (getDevClk(p->dev) == 0) {
		/* TODO: support initialization from scratch */
		return -EIO;
	}

	u32 crValue = *(p->ctrl + xspi_cr);
	if (((crValue & 1) == 0) || ((crValue & XSPI_CR_MODE_MASK) != XSPI_CR_MODE_MEMORY)) {
		/* Controller may not be initialized. TODO: support initialization from scratch */
		return -EIO;
	}

	/* Controller is already initialized and in memory-mapped mode. */
	int ret = xspi_chipSetup_readFile(minor);
	if (ret < 0) {
		fprintf(stderr, "read file error %d\n", ret);
		return ret;
	}

	if (s->mem.log_chipSize > 31) {
		fprintf(stderr, "Chip has size over 2 GB - full capacity not supported\n");
		s->mem.log_chipSize = 31;
	}

	if ((1UL << s->mem.log_chipSize) > p->size) {
		fprintf(stderr, "Chip has size larger than memory map - full capacity not supported\n");
		s->mem.log_chipSize = ffs(p->size);
	}

	const struct libdma_per *per;
	ret = libxpdma_acquirePeripheral(dma_memTransfer, memTransfer_perIsDst, LIBXPDMA_ACQUIRE_2D_MEM2PER, &per);
	if (ret < 0) {
		return ret;
	}

	ret = libxpdma_configureChannel(per, dma_mem2per, dma_priorityHigh, NULL);
	if (ret < 0) {
		return ret;
	}

	s->dma = per;

	if (mutexCreate(&s->mutex) < 0) {
		return -ENOMEM;
	}

	if (mutexCreate(&s->irq_mutex) < 0) {
		resourceDestroy(s->mutex);
		return -ENOMEM;
	}

	if (condCreate(&s->irq_cond) < 0) {
		resourceDestroy(s->mutex);
		resourceDestroy(s->irq_mutex);
		return -ENOMEM;
	}

	if (interrupt(p->irq, xspi_isr, (void *)minor, s->irq_cond, &s->irq) < 0) {
		resourceDestroy(s->mutex);
		resourceDestroy(s->irq_mutex);
		resourceDestroy(s->irq_cond);
		return -ENOMEM;
	}

	s->configured = true;
	return EOK;
}


int extFlash_fn_init(void)
{
	for (unsigned int i = 0; i < XSPI_N_CONTROLLERS; i++) {
		int ret = xspi_initOne(i);
		if ((ret < 0) && (ret != -ENODEV)) {
			return ret;
		}
	}

	return EOK;
}


ssize_t extFlash_fn_read(unsigned int minor, addr_t offs, void *buff, size_t len)
{
	int region = xspi_validateAndGetRegion(minor, offs, len);
	if (region < 0) {
		return region;
	}

	if (len == 0) {
		return 0;
	}

	if (xspi_rwlock_acquire(minor, false) < 0) {
		return -EBUSY;
	}

	/* TODO: for large reads consider using mem-to-mem DMA transfer - it may free up CPU for other tasks */
	memcpy(buff, xspi_ctrlParams[minor].start + offs, len);
	xspi_rwlock_release(minor);
	return len;
}


static int xspi_write_page(unsigned int minor, void *dest, const void *src, size_t *i, size_t n)
{
	int ret;
	ret = xspi_writeEnable(minor, 1);
	if (ret < 0) {
		return ret;
	}

	xspi_changeCtrlMode(minor, XSPI_CR_MODE_MEMORY, 1);
	ret = xspi_memcpy(xspi_common.ctrl[minor].dma, dest + *i, src + *i, n);
	if (ret < 0) {
		return ret;
	}

	ret = xspi_waitForWriteCompletion(minor, &xspi_common.ctrl[minor].mem.readStatus, 10000);
	if (ret < 0) {
		return ret;
	}

	*i += n;
	return 0;
}


static ssize_t xspi_write_internal(unsigned int minor, addr_t offs, const void *buff, size_t len)
{
	const xspi_ctrlParams_t *p = &xspi_ctrlParams[minor];
	/* Limit page size to 64 KB. In reality we are unlikely to ever hit this limit
	 * (typically pages are 256 B). */
	size_t page_size = min(1UL << xspi_common.ctrl[minor].mem.log_pageSize, (1UL << 16));

	size_t i = 0;
	int ret;

	if ((offs % page_size) != 0) {
		ret = xspi_write_page(minor, p->start + offs, buff, &i, min(page_size - (offs % page_size), len));
		if (ret < 0) {
			return ret;
		}
	}

	while ((len - i) >= page_size) {
		ret = xspi_write_page(minor, p->start + offs, buff, &i, page_size);
		if (ret < 0) {
			return ret;
		}
	}

	if ((len - i) > 0) {
		ret = xspi_write_page(minor, p->start + offs, buff, &i, len - i);
		if (ret < 0) {
			return ret;
		}
	}

	return (ssize_t)i;
}


ssize_t extFlash_fn_write(unsigned int minor, addr_t offs, const void *buff, size_t len)
{
	int region = xspi_validateAndGetRegion(minor, offs, len);
	if (region < 0) {
		return region;
	}

	if (region != REGION_NOT_ENCRYPTED) {
		size_t granularity = xspi_common.ctrl[minor].region[region].granularity;
		if (granularity > 1) {
			if (((offs % granularity) != 0) || ((len % granularity) != 0)) {
				return -EINVAL;
			}
		}
	}

	if (len == 0) {
		return 0;
	}

	if (xspi_rwlock_acquire(minor, true) < 0) {
		return -EBUSY;
	}

	u32 prev_mode = xspi_changeCtrlMode(minor, XSPI_CR_MODE_IWRITE, 0);
	ssize_t ret = xspi_write_internal(minor, offs, buff, len);
	xspi_changeCtrlMode(minor, prev_mode, 0);
	xspi_invalCache(xspi_ctrlParams[minor].start + offs, len);
	xspi_rwlock_release(minor);
	return ret;
}


static ssize_t xspi_erase_internal(unsigned int minor, addr_t offs, size_t len)
{
	flash_opDefinition_t op;
	size_t eraseSize = 1UL << xspi_common.ctrl[minor].mem.log_eraseSize;
	int ret = 0;
	ssize_t len_ret = (ssize_t)len;
	op.reg = xspi_common.ctrl[minor].mem.erase;
	op.isRead = 0;
	op.dataLen = 0;
	for (; len != 0; offs += eraseSize, len -= eraseSize) {
		op.addr = offs;
		ret = xspi_performWriteOp(minor, &op, NULL, xspi_common.ctrl[minor].mem.eraseTimeoutMs);
		if (ret < 0) {
			return ret;
		}
	}

	return (ssize_t)len_ret;
}


ssize_t extFlash_fn_erase(unsigned int minor, addr_t offs, size_t len)
{
	int region = xspi_validateAndGetRegion(minor, offs, len);
	if (region < 0) {
		return region;
	}

	size_t eraseSize = 1UL << xspi_common.ctrl[minor].mem.log_eraseSize;
	if (((offs % eraseSize) != 0) || ((len % eraseSize) != 0)) {
		return -EINVAL;
	}

	if (xspi_rwlock_acquire(minor, true) < 0) {
		return -EBUSY;
	}

	u32 prev_mode = xspi_changeCtrlMode(minor, XSPI_CR_MODE_IWRITE, 0);
	ssize_t ret = xspi_erase_internal(minor, offs, len);
	xspi_changeCtrlMode(minor, prev_mode, 0);
	xspi_invalCache(xspi_ctrlParams[minor].start + offs, len);
	xspi_rwlock_release(minor);
	return ret;
}


int extFlash_fn_sync(unsigned int minor)
{
	if (!xspi_isValidMinor(minor)) {
		return -ENODEV;
	}

	if (xspi_rwlock_acquire(minor, false) < 0) {
		return -EBUSY;
	}

	xspi_invalCache(xspi_ctrlParams[minor].start, (1UL << xspi_common.ctrl[minor].mem.log_chipSize));
	xspi_rwlock_release(minor);
	return EOK;
}


int extFlash_fn_getInfo(unsigned int minor, extFlashDef_t *info)
{
	info->nDevices = XSPI_N_CONTROLLERS;
	if (!xspi_isValidMinor(minor)) {
		return -ENODEV;
	}

	/* No locking required - there are no interactions with memory. */
	struct xspi_mem *mem = &xspi_common.ctrl[minor].mem;
	strncpy(info->name, mem->name, sizeof(info->name));
	info->name[sizeof(info->name) - 1] = '\0';
	info->eraseSize = (1UL << mem->log_eraseSize);
	info->writeSize = 16; /* TODO: determine this based on MCE settings */
	info->pageSize = (1UL << mem->log_pageSize);
	info->totalSize = (1UL << mem->log_chipSize);
	return EOK;
}
