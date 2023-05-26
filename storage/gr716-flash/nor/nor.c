/*
 * Phoenix-RTOS
 *
 * GR716 flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <sys/time.h>

#include "flash.h"
#include "nor.h"


/* clang-format off */
#define FLASH_ID(vid, pid) (((((vid) & 0xffu) << 16) | ((pid) & 0xff00u) | ((pid) & 0xffu)) << 8)


enum { write_disable = 0, write_enable };


static const char *nor_vendors[] = {
	"\xef" " Winbond",
	"\x20" " Micron",
	"\x9d" " ISSI",
	"\xc2" " Macronix",
	NULL
};
/* clang-format on */


static const struct nor_info flashInfo[] = {
	/* Macronix (MXIX) */
	{ FLASH_ID(0xc2u, 0x2019u), "MX25L25635F", 32 * 1024 * 1024, 0x100, 0x1000, 2, 120, 150 * 1000 },
};


static int nor_readId(spimctrl_t *spimctrl, uint32_t *id)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { FLASH_CMD_RDID, FLASH_CMD_NOP, FLASH_CMD_NOP, 0x00u };

	xfer.type = xfer_opRead;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.rxData = (uint8_t *)id;
	xfer.dataLen = 3;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int nor_readStatus(spimctrl_t *spimctrl, uint8_t *status)
{
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_RDSR;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = status;
	xfer.dataLen = 1;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int nor_writeEnable(spimctrl_t *spimctrl, int enable)
{
	int res;
	struct xferOp xfer;
	uint8_t status = 0;
	const uint8_t cmd = (enable == 1) ? FLASH_CMD_WREN : FLASH_CMD_WRDI;

	res = nor_waitBusy(spimctrl, 0);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_readStatus(spimctrl, &status);
	if (res < EOK) {
		return res;
	}

	status = (status & FLASH_SR_WEL) ? 1 : 0;

	if (status != enable) {
		return -EIO;
	}

	return EOK;
}


static int nor_readEAR(spimctrl_t *spimctrl, uint8_t *status)
{
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_RDEAR;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = status;
	xfer.dataLen = 1;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int nor_writeEAR(spimctrl_t *spimctrl, uint8_t value)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_WREAR;

	nor_writeEnable(spimctrl, write_enable);

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = &value;
	xfer.dataLen = 1;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_readEAR(spimctrl, &spimctrl->ear);
	if (res < EOK) {
		return res;
	}

	if (spimctrl->ear != value) {
		return -EIO;
	}

	return EOK;
}


static int nor_validateEar(spimctrl_t *spimctrl, uint32_t addr)
{
	int res = EOK;
	const uint8_t desiredEar = (addr >> 24) & 0xffu;

	if (desiredEar != spimctrl->ear) {
		res = nor_writeEAR(spimctrl, desiredEar);
	}
	return res;
}


int nor_waitBusy(spimctrl_t *spimctrl, time_t timeout)
{
	int res;
	uint8_t status = 0;
	time_t now, end;
	(void)gettime(&end, NULL);

	end += timeout * 1000;

	do {
		res = nor_readStatus(spimctrl, &status);
		if (res < EOK) {
			return res;
		}

		(void)gettime(&now, NULL);
		if ((timeout > 0) && (now > end)) {
			return -ETIME;
		}
	} while ((status & FLASH_SR_WIP) != 0);

	return EOK;
}


int nor_eraseChip(spimctrl_t *spimctrl, time_t timeout)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_CE;

	res = nor_writeEnable(spimctrl, write_enable);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	return nor_waitBusy(spimctrl, timeout);
}


int nor_eraseSector(spimctrl_t *spimctrl, addr_t addr, time_t timeout)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd[4] = { FLASH_CMD_SE, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	res = nor_validateEar(spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	res = nor_writeEnable(spimctrl, write_enable);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_waitBusy(spimctrl, timeout);

	return res;
}


int nor_pageProgram(spimctrl_t *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { FLASH_CMD_PP, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	int res = nor_validateEar(spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	res = nor_writeEnable(spimctrl, write_enable);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.txData = src;
	xfer.dataLen = len;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_waitBusy(spimctrl, timeout);

	return res;
}


static ssize_t nor_readCmd(spimctrl_t *spimctrl, addr_t addr, void *data, size_t size)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { FLASH_CMD_READ, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	int res = nor_validateEar(spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opRead;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.rxData = data;
	xfer.dataLen = size;

	res = spimctrl_xfer(spimctrl, &xfer);

	return res < EOK ? res : (ssize_t)size;
}


static ssize_t nor_readAhb(spimctrl_t *spimctrl, addr_t addr, void *data, size_t size)
{
	int res = nor_validateEar(spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	(void)memcpy(data, (void *)addr + spimctrl->ahbStartAddr, size);

	return (res < EOK) ? res : (ssize_t)size;
}


ssize_t nor_readData(spimctrl_t *spimctrl, addr_t addr, void *data, size_t size)
{
	if (((addr & 0xff000000) == 0) && (((addr + size) & 0xff000000) != 0)) {
		/* If we'd have to change EAR register during read,
		 * read data through command (can be read without EAR change)
		 */
		return nor_readCmd(spimctrl, addr, data, size);
	}
	else {
		/* Direct copy */
		return nor_readAhb(spimctrl, addr, data, size);
	}
}


int nor_probe(spimctrl_t *spimctrl, const struct nor_info **nor, const char **pVendor)
{
	int res;
	uint32_t jedecId = 0;

	res = nor_readId(spimctrl, &jedecId);
	if (res < EOK) {
		return res;
	}

	res = nor_readEAR(spimctrl, &spimctrl->ear);
	if (res < EOK) {
		return res;
	}

	res = -ENXIO;
	for (size_t i = 0; i < sizeof(flashInfo) / sizeof(flashInfo[0]); ++i) {
		if (flashInfo[i].jedecId == jedecId) {
			*nor = &flashInfo[i];
			res = EOK;
			break;
		}
	}

	if (res != EOK) {
		return res;
	}

	*pVendor = "Unknown";

	for (size_t i = 0; nor_vendors[i]; ++i) {
		if (*(uint8_t *)nor_vendors[i] == (jedecId >> 24)) {
			*pVendor = &nor_vendors[i][2];
			break;
		}
	}

	return EOK;
}
