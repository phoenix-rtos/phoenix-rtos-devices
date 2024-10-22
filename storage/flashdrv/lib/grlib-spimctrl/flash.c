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


#define FLASH_CMD_RDID 0x9fu

/* Status register */

#define FLASH_SR_WIP 0x01u /* Write in progress */
#define FLASH_SR_WEL 0x02u /* Write enable latch */


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

};


static int flash_readStatus(struct nor_dev *dev, struct spimctrl *spimctrl, uint8_t *status)
{
	struct xferOp xfer;
	const uint8_t cmd = dev->cmds->rdsr;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = status;
	xfer.dataLen = 1;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int flash_writeEnable(struct nor_dev *dev, struct spimctrl *spimctrl, int enable)
{
	int res;
	struct xferOp xfer;
	uint8_t status = 0;
	const uint8_t cmd = (enable == 1) ? dev->cmds->wren : dev->cmds->wrdi;

	res = flash_waitBusy(dev, spimctrl, 0);
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

	res = flash_readStatus(dev, spimctrl, &status);
	if (res < EOK) {
		return res;
	}

	status = (status & FLASH_SR_WEL) ? 1 : 0;

	if (status != enable) {
		return -EIO;
	}

	return EOK;
}


static int flash_readEAR(struct nor_dev *dev, struct spimctrl *spimctrl, uint8_t *status)
{
	struct xferOp xfer;
	const uint8_t cmd = dev->cmds->rdear;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = status;
	xfer.dataLen = 1;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int flash_writeEAR(struct nor_dev *dev, struct spimctrl *spimctrl, uint8_t value)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = dev->cmds->wrear;

	flash_writeEnable(dev, spimctrl, write_enable);

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = &value;
	xfer.dataLen = 1;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = flash_readEAR(spimctrl, &spimctrl->ear);
	if (res < EOK) {
		return res;
	}

	if (spimctrl->ear != value) {
		return -EIO;
	}

	return EOK;
}


static int flash_validateEar(struct nor_dev *dev, struct spimctrl *spimctrl, uint32_t addr)
{
	int res = EOK;
	const uint8_t desiredEar = (addr >> 24) & 0xffu;

	if (desiredEar != spimctrl->ear) {
		res = flash_writeEAR(dev, spimctrl, desiredEar);
	}
	return res;
}


int flash_waitBusy(struct nor_dev *dev, struct spimctrl *spimctrl, time_t timeout)
{
	int res;
	uint8_t status = 0;
	time_t now, end;
	(void)gettime(&end, NULL);

	end += timeout * 1000;

	do {
		res = flash_readStatus(dev, spimctrl, &status);
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


int flash_eraseChip(struct nor_dev *dev, struct spimctrl *spimctrl, time_t timeout)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = dev->cmds->ce;

	res = flash_writeEnable(dev, spimctrl, write_enable);
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

	return flash_waitBusy(spimctrl, timeout);
}


int flash_eraseSector(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, time_t timeout)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd[4] = { dev->cmds->se, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	res = flash_validateEar(dev, spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	res = flash_writeEnable(dev, spimctrl, write_enable);
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

	res = flash_waitBusy(dev, spimctrl, timeout);

	return res;
}


int flash_pageProgram(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { dev->cmds->pp, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	int res = flash_validateEar(dev, spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	res = flash_writeEnable(dev, spimctrl, write_enable);
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

	res = flash_waitBusy(dev, spimctrl, timeout);

	return res;
}


static ssize_t flash_readCmd(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { dev->cmds->read, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	int res = flash_validateEar(dev, spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opRead;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.rxData = data;
	xfer.dataLen = size;

	res = spimctrl_xfer(spimctrl, &xfer);

	return (res < EOK) ? res : (ssize_t)size;
}


static ssize_t flash_readAhb(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
	int res = flash_validateEar(dev, spimctrl, addr);
	if (res < EOK) {
		return res;
	}

	(void)memcpy(data, (void *)addr + spimctrl->ahbStartAddr, size);

	return (res < EOK) ? res : (ssize_t)size;
}


ssize_t flash_readData(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
	if (((addr & 0xff000000) == 0) && (((addr + size) & 0xff000000) != 0)) {
		/* If we'd have to change EAR register during read,
		 * read data through command (can be read without EAR change)
		 */
		return flash_readCmd(dev, spimctrl, addr, data, size);
	}
	else {
		/* Direct copy */
		return flash_readAhb(dev, spimctrl, addr, data, size);
	}
}


static int flash_readId(struct nor_dev *dev, struct spimctrl *spimctrl, cfi_info_t *info)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { FLASH_CMD_RDID, 0x00u, 0x00u, 0x00u };

	xfer.type = xfer_opRead;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.rxData = (uint8_t *)info;
	xfer.dataLen = sizeof(*info);

	return spimctrl_xfer(spimctrl, &xfer);
}


static int flash_query(struct nor_dev *dev, struct spimctrl *spimctrl, const struct nor_info **nor, const char **pVendor)
{
	int res;
	uint32_t jedecId = 0;

	res = flash_readId(dev, spimctrl, &dev->cfi);
	if (res < EOK) {
		return res;
	}

	res = flash_readEAR(dev, spimctrl, &spimctrl->ear);
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


int flash_init(struct _storage_devCtx_t *ctx)
{
}


void flash_destroy(struct _storage_devCtx_t *ctx)
{
}
