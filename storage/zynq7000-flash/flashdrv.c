/*
 * Phoenix-RTOS
 *
 * Zynq-7000 nor flash driver
 *
 * Copyright 2021, 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "qspi.h"
#include "flashcfg.h"

#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>

#define TIMEOUT_CMD_MS 0x05
#define MAX_SIZE_CMD   0x20


struct {
	/* Buffers for command transactions */
	uint8_t cmdRx[MAX_SIZE_CMD];
	uint8_t cmdTx[MAX_SIZE_CMD];

	/* Data caching in sector's buffer */
	uint8_t *buff;
	uint32_t buffPos;
	uint32_t regID;
	uint32_t sectID;

	flash_info_t info;
} fdrv_common;


/* Auxiliary functions */

static inline time_t flashdrv_timeMsGet(void)
{
	time_t now;
	gettime(&now, NULL);

	return now / 1000;
}


static int flashdrv_regionFind(addr_t offs, uint32_t *id)
{
	int i;
	size_t sz;
	addr_t start = 0;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;

	for (i = 0; i < cfi->regsCount; ++i) {
		sz = CFI_SIZE_REGION(cfi->regs[i].size, cfi->regs[i].count);
		if (offs >= start && offs < (start + sz)) {
			*id = i;
			return EOK;
		}

		start = sz;
	}

	return -EINVAL;
}


static size_t flashdrv_regStart(int id)
{
	int i;
	size_t regStart = 0;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;

	for (i = 0; i < id; ++i)
		regStart += CFI_SIZE_REGION(cfi->regs[i].size, cfi->regs[i].count);

	return regStart;
}


static inline void flashdrv_initCmdBuff(const flash_cmd_t *cmd, addr_t offs)
{
	memset(fdrv_common.cmdTx, 0, cmd->size);
	fdrv_common.cmdTx[0] = cmd->opCode;
	fdrv_common.cmdTx[1] = (offs >> 16) & 0xff;
	fdrv_common.cmdTx[2] = (offs >> 8) & 0xff;
	fdrv_common.cmdTx[3] = offs & 0xff;
}


/* Auxiliary flash commands */

static int flashdrv_cfiRead(flash_cfi_t *cfi)
{
	ssize_t res;
	size_t cmdSz, dataSz;
	/* RDID instruction is a generic for each flash memory */
	flash_cmd_t cmd;

	flashcfg_jedecIDGet(&cmd);

	/* Cmd size is rounded to 4 bytes, it includes dummy cycles [b]. */
	cmdSz = (cmd.size + cmd.dummyCyc / 8 + 0x3) & ~0x3;
	/* Size of the data which is received during cmd transfer */
	dataSz = cmdSz - cmd.size;

	memset(fdrv_common.cmdTx, 0, cmdSz);
	fdrv_common.cmdTx[0] = cmd.opCode;

	qspi_start();
	res = qspi_transfer(fdrv_common.cmdTx, fdrv_common.cmdRx, cmdSz, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}

	/* Copy data received after dummy bytes */
	memcpy(cfi, (fdrv_common.cmdRx + cmd.size), dataSz);
	res = qspi_transfer(NULL, (uint8_t *)cfi + dataSz, sizeof(flash_cfi_t) - dataSz, 5 * TIMEOUT_CMD_MS);
	qspi_stop();

	return res;
}


static int flashdrv_statusRegGet(unsigned int *val)
{
	ssize_t res;
	size_t cmdSz;
	const flash_cmd_t *cmd = &fdrv_common.info.cmds[flash_cmd_rdsr1];

	*val = 0;

	cmdSz = cmd->size + cmd->dummyCyc / 8;
	memset(fdrv_common.cmdTx, 0, cmdSz);

	fdrv_common.cmdTx[0] = cmd->opCode;

	qspi_start();
	res = qspi_transfer(fdrv_common.cmdTx, (uint8_t *)val, cmdSz, TIMEOUT_CMD_MS);
	qspi_stop();

	return res;
}


static int flashdrv_wipCheck(time_t timeout)
{
	int res;
	unsigned int st;
	time_t start = flashdrv_timeMsGet();

	do {
		res = flashdrv_statusRegGet(&st);
		if (res < 0)
			return res;

		if ((flashdrv_timeMsGet() - start) >= timeout)
			return -ETIME;

	} while ((st >> 8) & 0x1);

	return EOK;
}


static int flashdrv_welSet(unsigned int cmdID)
{
	ssize_t res;
	const flash_cmd_t *cmd;

	if (cmdID != flash_cmd_wrdi && cmdID != flash_cmd_wren)
		return -EINVAL;

	cmd = &fdrv_common.info.cmds[cmdID];

	memset(fdrv_common.cmdTx, 0, cmd->size);
	fdrv_common.cmdTx[0] = cmd->opCode;

	qspi_start();
	res = qspi_transfer(fdrv_common.cmdTx, fdrv_common.cmdRx, cmd->size, TIMEOUT_CMD_MS);
	qspi_stop();

	return res;
}


static ssize_t flashdrv_pageProgram(addr_t offs, const void *buff, size_t len)
{
	ssize_t res;
	time_t timeout;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;
	const flash_cmd_t *cmd = &fdrv_common.info.cmds[flash_cmd_pp];

	if (len == 0)
		return 0;

	if (buff == NULL || len % CFI_SIZE_PAGE(cfi->pageSize))
		return -EINVAL;

	res = flashdrv_welSet(flash_cmd_wren);
	if (res < 0)
		return res;

	flashdrv_initCmdBuff(cmd, offs);

	qspi_start();
	res = qspi_transfer(fdrv_common.cmdTx, fdrv_common.cmdRx, cmd->size, TIMEOUT_CMD_MS);
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
	res = flashdrv_wipCheck(timeout);
	if (res < 0)
		return res;

	return len;
}


/* Device interface */

const flash_info_t *flashdrv_flashInfo(void)
{
	return &fdrv_common.info;
}


int flashdrv_sync(void)
{
	uint8_t *src;
	addr_t dst;
	ssize_t res;
	unsigned int i, pgNb;

	size_t pageSz, sectSz, regStart;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;

	if (fdrv_common.buffPos == 0)
		return EOK;

	regStart = flashdrv_regStart(fdrv_common.regID);
	sectSz = CFI_SIZE_SECTION(cfi->regs[fdrv_common.regID].size);
	pageSz = CFI_SIZE_PAGE(cfi->pageSize);
	pgNb = sectSz / pageSz;

	for (i = 0; i < pgNb; ++i) {
		dst = regStart + fdrv_common.sectID * sectSz + i * pageSz;
		src = fdrv_common.buff + i * pageSz;

		res = flashdrv_pageProgram(dst, src, pageSz);
		if (res < 0)
			return res;
	}

	fdrv_common.regID = (uint32_t)-1;
	fdrv_common.sectID = (uint32_t)-1;
	fdrv_common.buffPos = 0;

	return EOK;
}


int flashdrv_sectorErase(addr_t offs)
{
	ssize_t res;
	uint32_t id = 0;
	size_t sectorSz;
	time_t timeout;
	const flash_cmd_t *cmd;
	const flash_cfi_t *cfi = &fdrv_common.info.cfi;

	if (offs >= CFI_SIZE_FLASH(cfi->chipSize))
		return -EINVAL;

	res = flashdrv_regionFind(offs, &id);
	if (res < 0)
		return res;

	/* Check offset alligment */
	sectorSz = CFI_SIZE_SECTION(cfi->regs[id].size);
	if (offs & (sectorSz - 1))
		return -EINVAL;

	switch (sectorSz) {
		case 0x1000:
			cmd = &fdrv_common.info.cmds[flash_cmd_p4e];
			break;
		case 0x10000:
			cmd = &fdrv_common.info.cmds[flash_cmd_p64e];
			break;
		default:
			return -EINVAL;
	}

	res = flashdrv_welSet(flash_cmd_wren);
	if (res < 0)
		return res;

	flashdrv_initCmdBuff(cmd, offs);

	qspi_start();
	res = qspi_transfer(fdrv_common.cmdTx, fdrv_common.cmdRx, cmd->size, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}
	qspi_stop();

	timeout = CFI_TIMEOUT_MAX_ERASE(cfi->timeoutTypical.sectorErase, cfi->timeoutMax.sectorErase) + TIMEOUT_CMD_MS;

	return flashdrv_wipCheck(timeout);
}


int flashdrv_chipErase(void)
{
	ssize_t res;
	size_t cmdSz;
	time_t timeout;

	const flash_cfi_t *cfi = &fdrv_common.info.cfi;
	const flash_cmd_t *cmd = &fdrv_common.info.cmds[flash_cmd_be];

	res = flashdrv_welSet(flash_cmd_wren);
	if (res < 0)
		return res;

	cmdSz = cmd->size + cmd->dummyCyc / 8;
	memset(fdrv_common.cmdTx, 0, cmdSz);

	fdrv_common.cmdTx[0] = cmd->opCode;

	qspi_start();
	res = qspi_transfer(fdrv_common.cmdTx, NULL, cmdSz, TIMEOUT_CMD_MS);
	qspi_stop();

	if (res < 0)
		return res;

	timeout = CFI_TIMEOUT_MAX_ERASE(cfi->timeoutTypical.chipErase, cfi->timeoutMax.chipErase) + TIMEOUT_CMD_MS;

	return flashdrv_wipCheck(timeout);
}


ssize_t flashdrv_read(addr_t offs, void *buff, size_t len, time_t timeout)
{
	ssize_t res;
	size_t cmdSz, dataSz, transferSz;
	const flash_cmd_t *cmd = &fdrv_common.info.cmds[flash_cmd_qior];

	if (len == 0)
		return 0;

	if (buff == NULL || (offs + len) > CFI_SIZE_FLASH(fdrv_common.info.cfi.chipSize))
		return -EINVAL;

	flashdrv_initCmdBuff(cmd, offs);

	/* Cmd size is rounded to 4 bytes, it includes dummy cycles [b]. */
	cmdSz = (cmd->size + cmd->dummyCyc / 8 + 0x3) & ~0x3;

	/* Size of the data which is received during cmd transfer */
	transferSz = cmdSz - cmd->size - cmd->dummyCyc / 8;
	dataSz = (transferSz > len) ? len : transferSz;

	qspi_start();
	res = qspi_transfer(fdrv_common.cmdTx, fdrv_common.cmdRx, cmdSz, TIMEOUT_CMD_MS);
	if (res < 0) {
		qspi_stop();
		return res;
	}
	/* Copy data received after dummy bytes */
	memcpy(buff, (fdrv_common.cmdRx + cmd->size + cmd->dummyCyc / 8), dataSz);

	res = qspi_transfer(NULL, (uint8_t *)buff + dataSz, len - dataSz, timeout);
	qspi_stop();

	return (res < 0) ? res : (res + dataSz);
}


ssize_t flashdrv_write(addr_t offs, const void *buff, size_t len)
{
	ssize_t res;
	time_t timeout;
	size_t chunkSz = 0, freeSz = 0, saveSz = 0;
	static const uint32_t timeoutFactor = 0x100;

	uint32_t regID = 0, sectID = 0;
	size_t sectSz, regStart;

	const flash_cfi_t *cfi = &fdrv_common.info.cfi;

	if (len == 0)
		return 0;

	if (buff == NULL || (offs + len) > CFI_SIZE_FLASH(cfi->chipSize))
		return -EINVAL;

	while (saveSz < len) {
		offs += chunkSz;

		res = flashdrv_regionFind(offs, &regID);
		if (res < 0)
			return -EINVAL;

		regStart = flashdrv_regStart(regID);
		sectSz = CFI_SIZE_SECTION(cfi->regs[regID].size);
		sectID = (offs - regStart) / sectSz;

		if (regID != fdrv_common.regID || sectID != fdrv_common.sectID) {
			res = flashdrv_sync();
			if (res < 0)
				return res;

			/* Read operation timeout depends on sector size. Factor value selected empirically. */
			timeout = (sectSz * TIMEOUT_CMD_MS) / timeoutFactor;
			res = flashdrv_read(regStart + (sectSz * sectID), fdrv_common.buff, sectSz, timeout);
			if (res < 0)
				return res;

			res = flashdrv_sectorErase(regStart + (sectSz * sectID));
			if (res < 0)
				return res;

			fdrv_common.regID = regID;
			fdrv_common.sectID = sectID;
			fdrv_common.buffPos = offs - regStart - (sectID * sectSz);
		}

		freeSz = sectSz - fdrv_common.buffPos;
		chunkSz = (freeSz > (len - saveSz)) ? (len - saveSz) : freeSz;
		memcpy(fdrv_common.buff + fdrv_common.buffPos, (const uint8_t *)buff + saveSz, chunkSz);

		saveSz += chunkSz;
		fdrv_common.buffPos += chunkSz;

		if (fdrv_common.buffPos < sectSz)
			continue;

		res = flashdrv_sync();
		if (res < 0)
			return res;
	}

	return saveSz;
}


int flashdrv_done(void)
{
	int res = EOK;

	do {
		res = flashdrv_sync();
		if (res < 0) {
			qspi_deinit();
			break;
		}

		res = qspi_deinit();
		if (res < 0)
			break;
	} while (0);

	free(fdrv_common.buff);

	return res;
}


int flashdrv_init(void)
{
	int i, res;
	size_t sectorSz = 0;
	flash_info_t *info = &fdrv_common.info;

	fdrv_common.regID = (uint32_t)-1;
	fdrv_common.sectID = (uint32_t)-1;
	fdrv_common.buffPos = 0;

	res = qspi_init();
	if (res < 0)
		return res;

	res = flashdrv_cfiRead(&info->cfi);
	if (res < 0)
		return res;

	res = flashcfg_infoResolve(info);
	if (res < 0)
		return res;

	for (i = 0; i < info->cfi.regsCount; ++i) {
		if (CFI_SIZE_SECTION(info->cfi.regs[i].size) > sectorSz)
			sectorSz = CFI_SIZE_SECTION(info->cfi.regs[i].size);
	}

	fdrv_common.buff = (uint8_t *)malloc(sectorSz);
	if (fdrv_common.buff == NULL)
		return -ENOMEM;

	return EOK;
}
