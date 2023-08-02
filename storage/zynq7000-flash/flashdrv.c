/*
 * Phoenix-RTOS
 *
 * Zynq-7000 nor flash driver
 *
 * Copyright 2021, 2022 Phoenix Systems
 * Author: Hubert Buczynski, Malgorzata Wrobel
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "flashdrv.h"
#include "flashcfg.h"
#include "qspi.h"

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <sys/threads.h>
#include <cache.h>

/* Value determined empirically, contains time for communication via qspi and system calls invocations. */
#define TIMEOUT_CMD_MS 1000
#define MAX_SIZE_CMD   0x20

#define MTD_DEFAULT_ERASESZ 0x10000

#define BLK_CACHE_SECNUM 16 /* Maximum number of cached sectors in a region */


/* Cached device context definition */
struct cache_devCtx_s {
	off_t start;     /* storage start */
	unsigned int id; /* flash device memory id */
};


typedef struct {
	/* Buffers for command transactions */
	uint8_t cmdRx[MAX_SIZE_CMD];
	uint8_t cmdTx[MAX_SIZE_CMD];

	handle_t lock;
	off_t start;

	cachectx_t *cache;
	cache_devCtx_t cacheCtx;
} flash_reg_t;


struct {
	flash_reg_t *regs;

	flash_info_t info;     /* CFI structure for NOR flash memory */
	unsigned int initRegs; /* Number of initialized regions */
} fdrv_common;


/* Auxiliary functions */

static inline time_t flashdrv_timeMsGet(void)
{
	time_t now;
	gettime(&now, NULL);

	return now / 1000;
}


static size_t flashdrv_regStart(int id)
{
	int i;
	size_t regStart = 0;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;

	for (i = 0; i < id; ++i) {
		regStart += CFI_SIZE_REGION(cfi->regs[i].size, cfi->regs[i].count);
	}

	return regStart;
}


static inline void flashdrv_serializeTxCmd(uint8_t *buff, flash_cmd_t cmd, addr_t offs)
{
	memset(buff, 0, cmd.size);
	buff[0] = cmd.opCode;

	if (fdrv_common.info.addrMode == flash_4byteAddr) {
		buff[1] = (offs >> 24) & 0xff;
		buff[2] = (offs >> 16) & 0xff;
		buff[3] = (offs >> 8) & 0xff;
		buff[4] = offs & 0xff;
	}
	else {
		buff[1] = (offs >> 16) & 0xff;
		buff[2] = (offs >> 8) & 0xff;
		buff[3] = offs & 0xff;
	}
}


/* Auxiliary flash commands */

static int flashdrv_cfiRead(flash_cfi_t *cfi)
{
	ssize_t res;
	size_t cmdSz, dataSz;
	static uint8_t cmdRx[MAX_SIZE_CMD];
	static uint8_t cmdTx[MAX_SIZE_CMD];
	/* RDID instruction is a generic for each flash memory */
	flash_cmd_t cmd;

	flashcfg_jedecIDGet(&cmd);

	/* Cmd size is rounded to 4 bytes, it includes dummy cycles [b]. */
	cmdSz = (cmd.size + (cmd.dummyCyc * cmd.dataLines) / 8 + 0x3) & ~0x3;
	/* Size of the data which is received during cmd transfer */
	dataSz = cmdSz - cmd.size;

	memset(cmdTx, 0, cmdSz);
	cmdTx[0] = cmd.opCode;

	qspi_start();
	res = qspi_transfer(cmdTx, cmdRx, cmdSz, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}

	/* Copy data received after dummy bytes */
	memcpy(cfi, (cmdRx + cmd.size), dataSz);
	res = qspi_transfer(NULL, (uint8_t *)cfi + dataSz, sizeof(flash_cfi_t) - dataSz, 5 * TIMEOUT_CMD_MS);
	qspi_stop();

	return res;
}


static int _flashdrv_statusRegGet(unsigned int id, unsigned int *val)
{
	ssize_t res;
	size_t cmdSz;
	const flash_cmd_t cmd = fdrv_common.info.cmds[flash_cmd_rdsr1];

	*val = 0;

	cmdSz = cmd.size + (cmd.dummyCyc * cmd.dataLines) / 8;
	memset(fdrv_common.regs[id].cmdTx, 0, cmdSz);

	fdrv_common.regs[id].cmdTx[0] = cmd.opCode;

	qspi_start();
	res = qspi_transfer(fdrv_common.regs[id].cmdTx, (uint8_t *)val, cmdSz, TIMEOUT_CMD_MS);
	qspi_stop();

	return res;
}


static int _flashdrv_wipCheck(unsigned int id, time_t timeout)
{
	int res;
	unsigned int st;
	time_t start = flashdrv_timeMsGet();
	int lastTry = 0;

	do {
		res = _flashdrv_statusRegGet(id, &st);
		if (res < 0) {
			return res;
		}

		if ((flashdrv_timeMsGet() - start) >= timeout) {
			/* Check one last time to prevent the possibility of starvation. */
			if (lastTry == 0) {
				lastTry = 1;
			}
			else {
				return -ETIME;
			}
		}
	} while ((st >> 8) & 0x1);

	return EOK;
}


static int _flashdrv_welSet(unsigned int id, unsigned int cmdID)
{
	ssize_t res;
	flash_cmd_t cmd;

	if (cmdID != flash_cmd_wrdi && cmdID != flash_cmd_wren) {
		return -EINVAL;
	}

	cmd = fdrv_common.info.cmds[cmdID];

	memset(fdrv_common.regs[id].cmdTx, 0, cmd.size);
	fdrv_common.regs[id].cmdTx[0] = cmd.opCode;

	qspi_start();
	res = qspi_transfer(fdrv_common.regs[id].cmdTx, fdrv_common.regs[id].cmdRx, cmd.size, TIMEOUT_CMD_MS);
	qspi_stop();

	return res;
}


static ssize_t _flashdrv_pageProgram(unsigned int id, addr_t offs, const void *buff, size_t len)
{
	ssize_t res;
	time_t timeout;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;
	const flash_cmd_t cmd = fdrv_common.info.cmds[fdrv_common.info.ppCmd];

	res = _flashdrv_welSet(id, flash_cmd_wren);
	if (res < 0) {
		return res;
	}

	flashdrv_serializeTxCmd(fdrv_common.regs[id].cmdTx, cmd, offs);

	qspi_start();
	res = qspi_transfer(fdrv_common.regs[id].cmdTx, fdrv_common.regs[id].cmdRx, cmd.size, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}

	/* Transmit data to write */
	res = qspi_transfer(buff, NULL, len, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}
	qspi_stop();

	len = res;

	timeout = CFI_TIMEOUT_MAX_PROGRAM(cfi->timeoutTypical.pageWrite, cfi->timeoutMax.pageWrite) + TIMEOUT_CMD_MS;
	res = _flashdrv_wipCheck(id, timeout);
	if (res < 0) {
		return res;
	}

	return len;
}


static int _flashdrv_sectorErase(unsigned int id, addr_t offs)
{
	ssize_t res;
	size_t sectorSz;
	time_t timeout;
	flash_cmd_t cmd;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;

	/* Check offset alligment */
	sectorSz = CFI_SIZE_SECTION(cfi->regs[id].size);
	if (offs & (sectorSz - 1)) {
		return -EINVAL;
	}

	switch (sectorSz) {
		case 0x1000:
			cmd = (fdrv_common.info.addrMode == flash_4byteAddr) ? fdrv_common.info.cmds[flash_cmd_4p4e] : fdrv_common.info.cmds[flash_cmd_p4e];
			break;
		case 0x10000:
			cmd = (fdrv_common.info.addrMode == flash_4byteAddr) ? fdrv_common.info.cmds[flash_cmd_4p64e] : fdrv_common.info.cmds[flash_cmd_p64e];
			break;
		default:
			return -EINVAL;
	}

	res = _flashdrv_welSet(id, flash_cmd_wren);
	if (res < 0) {
		return res;
	}

	flashdrv_serializeTxCmd(fdrv_common.regs[id].cmdTx, cmd, offs);

	qspi_start();
	res = qspi_transfer(fdrv_common.regs[id].cmdTx, fdrv_common.regs[id].cmdRx, cmd.size, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}
	qspi_stop();

	timeout = CFI_TIMEOUT_MAX_ERASE(cfi->timeoutTypical.sectorErase, cfi->timeoutMax.sectorErase) + TIMEOUT_CMD_MS;

	return _flashdrv_wipCheck(id, timeout);
}


static int _flashdrv_chipErase(unsigned int id)
{
	ssize_t res;
	size_t cmdSz;
	time_t timeout;

	const flash_cfi_t *cfi = &fdrv_common.info.cfi;
	const flash_cmd_t cmd = fdrv_common.info.cmds[flash_cmd_be];

	res = _flashdrv_welSet(id, flash_cmd_wren);
	if (res < 0) {
		return res;
	}

	cmdSz = cmd.size + (cmd.dummyCyc * cmd.dataLines) / 8;
	memset(fdrv_common.regs[id].cmdTx, 0, cmdSz);

	fdrv_common.regs[id].cmdTx[0] = cmd.opCode;

	qspi_start();
	res = qspi_transfer(fdrv_common.regs[id].cmdTx, NULL, cmdSz, TIMEOUT_CMD_MS);
	qspi_stop();

	if (res < 0) {
		return res;
	}

	timeout = CFI_TIMEOUT_MAX_ERASE(cfi->timeoutTypical.chipErase, cfi->timeoutMax.chipErase) + TIMEOUT_CMD_MS;

	return _flashdrv_wipCheck(id, timeout);
}


static ssize_t _flashdrv_read(unsigned int id, addr_t offs, void *buff, size_t len)
{
	ssize_t res;
	size_t cmdSz, dataSz, transferSz, paddedCmdSz, dummySz;
	const flash_cmd_t cmd = fdrv_common.info.cmds[fdrv_common.info.readCmd];

	flashdrv_serializeTxCmd(fdrv_common.regs[id].cmdTx, cmd, offs);

	dummySz = (cmd.dummyCyc * cmd.dataLines) / 8;
	cmdSz = cmd.size + dummySz;

	/* Send 0xff as dummy byte as some flashes require dummy byte that starts with 0xf. */
	memset(fdrv_common.regs[id].cmdTx + cmdSz, 0xff, dummySz);

	/* Cmd size is rounded to 4 bytes, it includes dummy cycles [b]. */
	paddedCmdSz = (cmdSz + 0x3) & ~0x3;

	/* Size of the data which is received during cmd transfer */
	transferSz = paddedCmdSz - cmdSz;
	dataSz = (transferSz > len) ? len : transferSz;

	qspi_start();
	res = qspi_transfer(fdrv_common.regs[id].cmdTx, fdrv_common.regs[id].cmdRx, paddedCmdSz, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}
	/* Copy data received after dummy bytes */
	memcpy(buff, (fdrv_common.regs[id].cmdRx + cmdSz), dataSz);

	res = qspi_transfer(NULL, (uint8_t *)buff + dataSz, len - dataSz, TIMEOUT_CMD_MS * len);
	qspi_stop();

	return (res < 0) ? res : (res + dataSz);
}


/* cache_readCb_t - wrapper for _flashdrv_read (*/
static ssize_t _flashdrv_readCb(uint64_t offs, void *buff, size_t len, cache_devCtx_t *ctx)
{
	return _flashdrv_read(ctx->id, offs + ctx->start, buff, len);
}

/* Block device interface */

/* cache_writeCb_t */
static ssize_t _flashdrv_writeCb(uint64_t offs, const void *buff, size_t len, cache_devCtx_t *ctx)
{
	uint8_t *src;
	addr_t dst;
	ssize_t res;
	unsigned int i, regID = ctx->id, pgNb;

	size_t pageSz, sectSz;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;


	sectSz = CFI_SIZE_SECTION(cfi->regs[regID].size);
	pageSz = CFI_SIZE_PAGE(cfi->pageSize);
	pgNb = sectSz / pageSz;

	offs += ctx->start;

	res = _flashdrv_sectorErase(regID, offs);

	for (i = 0; i < pgNb; ++i) {
		dst = offs + i * pageSz;
		src = (uint8_t *)buff + i * pageSz;

		res = _flashdrv_pageProgram(regID, dst, src, pageSz);
		if (res < 0) {
			return res;
		}
	}

	return sectSz;
}


static ssize_t flashdrv_blkRead(struct _storage_t *strg, off_t start, void *data, size_t size)
{
	ssize_t res;
	unsigned int regID = strg->dev->ctx->id;

	mutexLock(fdrv_common.regs[regID].lock);
	res = cache_read(fdrv_common.regs[regID].cache, start - strg->start, data, size);
	mutexUnlock(fdrv_common.regs[regID].lock);

	return res;
}


static ssize_t flashdrv_blkWrite(struct _storage_t *strg, off_t start, const void *data, size_t size)
{
	ssize_t res;
	unsigned int regID = strg->dev->ctx->id;

	mutexLock(fdrv_common.regs[regID].lock);
	res = cache_write(fdrv_common.regs[regID].cache, start - strg->start, data, size, LIBCACHE_WRITE_BACK);
	mutexUnlock(fdrv_common.regs[regID].lock);

	return res;
}


static int flashdrv_blkSync(struct _storage_t *strg)
{
	int res;
	unsigned int regID = strg->dev->ctx->id;

	mutexLock(fdrv_common.regs[regID].lock);
	res = cache_flush(fdrv_common.regs[regID].cache, 0, strg->size);
	mutexUnlock(fdrv_common.regs[regID].lock);

	return res;
}


const static storage_blkops_t blkOps = {
	.read = flashdrv_blkRead,
	.write = flashdrv_blkWrite,
	.sync = flashdrv_blkSync
};


/* MTD interface */

static int flashdrv_mtdErase(struct _storage_t *strg, off_t offs, size_t size)
{
	int res;
	size_t len = 0;
	size_t secSz = CFI_SIZE_SECTION(fdrv_common.info.cfi.regs[strg->dev->ctx->id].size);
	off_t beg, end;

	if ((offs == 0) && (size == CFI_SIZE_FLASH(fdrv_common.info.cfi.chipSize))) {
		mutexLock(fdrv_common.regs[strg->dev->ctx->id].lock);
		res = _flashdrv_chipErase(strg->dev->ctx->id);
		mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);
		return res;
	}

	mutexLock(fdrv_common.regs[strg->dev->ctx->id].lock);
	while (len < size) {
		res = _flashdrv_sectorErase(strg->dev->ctx->id, offs + len);
		if (res < 0) {
			mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);
			return res;
		}

		len += secSz;
	}

	beg = offs - strg->start;
	end = beg + size;

	/* Invalidate block device cache for coherence */
	res = cache_invalidate(fdrv_common.regs[strg->dev->ctx->id].cache, beg, end);

	mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);

	return res < 0 ? res : EOK;
}


static int flashdrv_mtdRead(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	int res;
	off_t beg, end;

	beg = offs - strg->start;
	end = beg + len;

	mutexLock(fdrv_common.regs[strg->dev->ctx->id].lock);

	/* Flush block device cache for coherence */
	res = cache_flush(fdrv_common.regs[strg->dev->ctx->id].cache, beg, end);
	if (res < 0) {
		mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);
		return res;
	}

	res = _flashdrv_read(strg->dev->ctx->id, offs, data, len);
	mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);

	if (res >= 0) {
		*retlen = res;
	}

	return res < 0 ? res : EOK;
}


static int flashdrv_mtdWrite(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	ssize_t res = 0;
	size_t tempSz = 0, chunkSz = 0;
	off_t beg, end;

	beg = offs - strg->start;
	end = beg + len;

	mutexLock(fdrv_common.regs[strg->dev->ctx->id].lock);

	/* Clean block device cache for coherence */
	res = cache_clean(fdrv_common.regs[strg->dev->ctx->id].cache, beg, end);
	if (res < 0) {
		mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);
		return res;
	}

	chunkSz = offs % strg->dev->mtd->writeBuffsz;
	if (chunkSz != 0) {
		chunkSz = len < (strg->dev->mtd->writeBuffsz - chunkSz) ? len : (strg->dev->mtd->writeBuffsz - chunkSz);
		res = _flashdrv_pageProgram(strg->dev->ctx->id, offs, data, chunkSz);
		if (res < 0) {
			*retlen = tempSz;
			mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);
			return res;
		}

		tempSz = res;
	}

	while (tempSz < len) {
		chunkSz = ((len - tempSz) > strg->dev->mtd->writeBuffsz) ? strg->dev->mtd->writeBuffsz : (len - tempSz);
		res = _flashdrv_pageProgram(strg->dev->ctx->id, offs + tempSz, (const char *)data + tempSz, chunkSz);
		if (res < 0) {
			*retlen = tempSz;
			mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);
			return res;
		}

		tempSz += res;
	}

	*retlen = tempSz;
	mutexUnlock(fdrv_common.regs[strg->dev->ctx->id].lock);

	return EOK;
}


const static storage_mtdops_t mtdOps = {
	.erase = flashdrv_mtdErase,
	.unPoint = NULL,
	.point = NULL,
	.read = flashdrv_mtdRead,
	.write = flashdrv_mtdWrite,

	.meta_read = NULL,
	.meta_write = NULL,

	.sync = NULL,
	.lock = NULL,
	.unLock = NULL,
	.isLocked = NULL,

	.block_isBad = NULL,
	.block_isReserved = NULL,
	.block_markBad = NULL,
	.block_maxBadNb = NULL,

	.suspend = NULL,
	.resume = NULL,
	.reboot = NULL,
};


/* Flash driver interface */

int flashdrv_done(storage_t *strg)
{
	int i, res = -1;

	/* Only root device has allocated resources */
	if (strg->parent == NULL) {
		res = cache_deinit(fdrv_common.regs[strg->dev->ctx->id].cache);
		if (res < 0) {
			return res;
		}
		fdrv_common.regs[strg->dev->ctx->id].cache = NULL;

		resourceDestroy(fdrv_common.regs[strg->dev->ctx->id].lock);
		free(strg->dev->mtd);
		free(strg->dev->blk);
		free(strg->dev->ctx);
		free(strg->dev);
	}

	for (i = 0; i < fdrv_common.info.cfi.regsCount; ++i) {
		if (fdrv_common.regs[i].cache != NULL) {
			return res;
		}
	}

	fdrv_common.initRegs = 0;
	free(fdrv_common.regs);

	do {
		if (res < 0) {
			qspi_deinit();
			break;
		}

		res = qspi_deinit();
		if (res < 0) {
			break;
		}
	} while (0);

	return res;
}


int flashdrv_devInit(storage_t *strg)
{
	int res;
	flash_reg_t *reg;
	size_t secSz;
	cache_ops_t cacheOps;

	int id = fdrv_common.initRegs;
	flash_info_t *info = &fdrv_common.info;

	/* Initialize region data */
	reg = &fdrv_common.regs[id];
	res = mutexCreate(&reg->lock);
	if (res < 0) {
		return res;
	}

	secSz = CFI_SIZE_SECTION(info->cfi.regs[id].size);
	reg->start = flashdrv_regStart(id);

	/* Initialize dev structure for new region */
	strg->dev = malloc(sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		resourceDestroy(reg->lock);
		return -ENOMEM;
	}

	strg->dev->ctx = malloc(sizeof(storage_devCtx_t));
	if (strg->dev->ctx == NULL) {
		resourceDestroy(reg->lock);
		free(strg->dev);
		return -ENOMEM;
	}

	/* Initialize storage device properties */
	strg->parent = NULL;
	strg->start = reg->start;
	strg->size = CFI_SIZE_REGION(info->cfi.regs[id].size, info->cfi.regs[id].count);

	/* NOR flash driver supports MTD interface */
	strg->dev->mtd = malloc(sizeof(storage_mtd_t));
	if (strg->dev->mtd == NULL) {
		resourceDestroy(reg->lock);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

/* In case of non-standard erase size, warning information is printed */
#ifndef NDEBUG
	if (secSz > MTD_DEFAULT_ERASESZ) {
		printf("flashdrv: non-standard erase size, file system image properties have to be changed to suit greater size.");
	}
#endif

	strg->dev->ctx->id = id;
	strg->dev->mtd->ops = &mtdOps;
	strg->dev->mtd->type = mtd_norFlash;
	strg->dev->mtd->name = info->name;
	strg->dev->mtd->metaSize = 0;
	strg->dev->mtd->oobAvail = 0;
	strg->dev->mtd->oobSize = 0;
	strg->dev->mtd->writesz = 0x1;
	strg->dev->mtd->writeBuffsz = CFI_SIZE_PAGE(info->cfi.pageSize);
	strg->dev->mtd->erasesz = (secSz < MTD_DEFAULT_ERASESZ) ? MTD_DEFAULT_ERASESZ : secSz;


	/* NOR flash supports block device interface */
	strg->dev->blk = malloc(sizeof(storage_blk_t));
	if (strg->dev->blk == NULL) {
		resourceDestroy(reg->lock);
		free(strg->dev->ctx);
		free(strg->dev->mtd);
		free(strg->dev);
		return -ENOMEM;
	}
	strg->dev->blk->ops = &blkOps;

	cacheOps.readCb = _flashdrv_readCb;
	cacheOps.writeCb = _flashdrv_writeCb;
	reg->cacheCtx.start = strg->start;
	reg->cacheCtx.id = id;
	cacheOps.ctx = &reg->cacheCtx;

	/* cache maps from 0 strg->size */
	reg->cache = cache_init(strg->size, secSz, BLK_CACHE_SECNUM, &cacheOps);
	if (reg->cache == NULL) {
		resourceDestroy(reg->lock);
		free(strg->dev->mtd);
		free(strg->dev->blk);
		free(strg->dev);
		return -ENOMEM;
	}

	fdrv_common.initRegs++;

	return (info->cfi.regsCount - fdrv_common.initRegs);
}


int flashdrv_init(storage_t *strg)
{
	int res;
	flash_cmd_t readCmd;

	if (strg == NULL) {
		return -EINVAL;
	}

	/* Initialize driver components - performs only once */
	if (fdrv_common.initRegs == 0) {
		res = qspi_init();
		if (res < 0) {
			return res;
		}

		res = flashdrv_cfiRead(&fdrv_common.info.cfi);
		if (res < 0) {
			return res;
		}

		res = flashcfg_infoResolve(&fdrv_common.info);
		if (res < 0) {
			return res;
		}

		readCmd = fdrv_common.info.cmds[fdrv_common.info.readCmd];
		/* Only used for assert. */
		(void)readCmd;

		/* Require data RXed due to dummy cycles to be byte aligned. */
		/* This requirement is in place to simplify implementation of read. */
		assert((readCmd.dummyCyc != CFI_DUMMY_CYCLES_NOT_SET) && (((readCmd.dummyCyc * readCmd.dataLines) % 8) == 0));

		if (fdrv_common.info.init != NULL) {
			res = fdrv_common.info.init(&fdrv_common.info);
			if (res < 0) {
				return res;
			}
		}

		fdrv_common.regs = malloc(fdrv_common.info.cfi.regsCount * sizeof(flash_reg_t));
		if (fdrv_common.regs == NULL) {
			return -ENOMEM;
		}
	}

	/* All regions are initialized */
	if (fdrv_common.initRegs >= fdrv_common.info.cfi.regsCount) {
		return 0;
	}

	res = flashdrv_devInit(strg);
	if (res < 0) {
		free(fdrv_common.regs);
	}

	return res;
}
