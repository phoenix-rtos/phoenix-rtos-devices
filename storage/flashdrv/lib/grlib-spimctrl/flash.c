/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash driver
 *
 * Copyright 2024 Phoenix Systems
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
#include "flashdrv.h"


#define FLASH_CMD_RDID 0x9fu

/* Status register */

#define FLASH_SR_WIP 0x01u /* Write in progress */
#define FLASH_SR_WEL 0x02u /* Write enable latch */

#define VID_MACRONIX 0xc2u
#define VID_SPANSION 0x01u

/* clang-format off */

enum { write_disable = 0, write_enable };


static const struct flash_cmds flash_spansionCmds = {
	.rdsr = 0x05u, .wren = 0x06u, .wrdi = 0x04u, .rdear = 0x16u,
	.wrear = 0x17u, .ce = 0x60u, .se = 0xd8u, .pp = 0x02u, .read = 0x03u
};
/* clang-format on */


static const struct flash_dev flash_devs[] = {
	{ "S25FL128S", VID_SPANSION, 0x2018u, &flash_spansionCmds }
};


static int flash_readStatus(const struct flash_dev *dev, struct spimctrl *spimctrl, uint8_t *status)
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


static int flash_writeEnable(const struct flash_dev *dev, struct spimctrl *spimctrl, int enable)
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
	if (res < 0) {
		return res;
	}

	res = flash_readStatus(dev, spimctrl, &status);
	if (res < 0) {
		return res;
	}

	status = (status & FLASH_SR_WEL) ? 1 : 0;

	if (status != enable) {
		return -EIO;
	}

	return 0;
}


static int flash_readEAR(const struct flash_dev *dev, struct spimctrl *spimctrl, uint8_t *status)
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


static int flash_writeEAR(const struct flash_dev *dev, struct spimctrl *spimctrl, uint8_t value)
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
	if (res < 0) {
		return res;
	}

	res = flash_readEAR(dev, spimctrl, &spimctrl->ear);
	if (res < 0) {
		return res;
	}

	if (spimctrl->ear != value) {
		return -EIO;
	}

	return 0;
}


static int flash_validateEar(const struct flash_dev *dev, struct spimctrl *spimctrl, uint32_t addr)
{
	int res = 0;
	const uint8_t desiredEar = (addr >> 24) & 0xffu;

	if (desiredEar != spimctrl->ear) {
		res = flash_writeEAR(dev, spimctrl, desiredEar);
	}
	return res;
}


int flash_waitBusy(const struct flash_dev *dev, struct spimctrl *spimctrl, time_t timeout)
{
	int res;
	uint8_t status = 0;
	time_t now, end;
	(void)gettime(&end, NULL);

	end += timeout * 1000;

	do {
		res = flash_readStatus(dev, spimctrl, &status);
		if (res < 0) {
			return res;
		}

		(void)gettime(&now, NULL);
		if ((timeout > 0) && (now > end)) {
			return -ETIME;
		}
	} while ((status & FLASH_SR_WIP) != 0);

	return 0;
}


int flash_chipErase(const struct flash_dev *dev, struct spimctrl *spimctrl, time_t timeout)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = dev->cmds->ce;

	res = flash_writeEnable(dev, spimctrl, write_enable);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < 0) {
		return res;
	}

	return flash_waitBusy(dev, spimctrl, timeout);
}


int flash_sectorErase(const struct flash_dev *dev, struct spimctrl *spimctrl, addr_t addr, time_t timeout)
{
	int res;
	struct xferOp xfer;
	const addr_t eraseAddr = addr + spimctrl->moffs;
	const uint8_t cmd[4] = { dev->cmds->se, (eraseAddr >> 16) & 0xff, (eraseAddr >> 8) & 0xff, eraseAddr & 0xff };

	res = flash_validateEar(dev, spimctrl, eraseAddr);
	if (res < 0) {
		return res;
	}

	res = flash_writeEnable(dev, spimctrl, write_enable);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < 0) {
		return res;
	}

	res = flash_waitBusy(dev, spimctrl, timeout);

	return res;
}


int flash_pageProgram(const struct flash_dev *dev, struct spimctrl *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout)
{
	struct xferOp xfer;
	const addr_t programAddr = addr + spimctrl->moffs;
	const uint8_t cmd[4] = { dev->cmds->pp, (programAddr >> 16) & 0xff, (programAddr >> 8) & 0xff, programAddr & 0xff };

	int res = flash_validateEar(dev, spimctrl, programAddr);
	if (res < 0) {
		return res;
	}

	res = flash_writeEnable(dev, spimctrl, write_enable);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.txData = src;
	xfer.dataLen = len;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < 0) {
		return res;
	}

	res = flash_waitBusy(dev, spimctrl, timeout);

	return res;
}


static ssize_t flash_readCmd(const struct flash_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { dev->cmds->read, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	int res = flash_validateEar(dev, spimctrl, addr);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opRead;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.rxData = data;
	xfer.dataLen = size;

	res = spimctrl_xfer(spimctrl, &xfer);

	return (res < 0) ? res : (ssize_t)size;
}


// static ssize_t flash_readAhb(const struct flash_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
// {
// 	int res = flash_validateEar(dev, spimctrl, addr);
// 	if (res < 0) {
// 		return res;
// 	}

// 	(void)memcpy(data, (void *)addr + spimctrl->ahbStartAddr, size);

// 	return (res < 0) ? res : (ssize_t)size;
// }


ssize_t flash_readData(const struct flash_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
	addr_t readAddr = addr + spimctrl->moffs;
	return flash_readCmd(dev, spimctrl, readAddr, data, size);
}


static int flash_readId(const struct spimctrl *spimctrl, cfi_info_t *cfi)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { FLASH_CMD_RDID, 0x00u, 0x00u, 0x00u };

	xfer.type = xfer_opRead;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.rxData = (uint8_t *)cfi;
	xfer.dataLen = sizeof(*cfi);

	return spimctrl_xfer(spimctrl, &xfer);
}


static uint16_t flash_deserialize16(uint16_t value)
{
	return ((value & 0xff) << 8) | ((value >> 8) & 0xff);
}


static int flash_query(const struct flash_dev **dev, struct spimctrl *spimctrl, cfi_info_t *cfi)
{
	int res = flash_readId(spimctrl, cfi);
	if (res < 0) {
		return res;
	}

	res = -ENXIO;

	uint16_t device;
	memcpy(&device, &cfi->vendorData[1], sizeof(device));
	device = flash_deserialize16(device);

	for (size_t i = 0; i < sizeof(flash_devs) / sizeof(flash_devs[0]); ++i) {
		const struct flash_dev *flash = &flash_devs[i];
		if ((cfi->vendorData[0] == flash->vendor) && (device == flash->device)) {
			*dev = flash;
			res = 0;
			break;
		}
	}

	if (res != 0) {
		return res;
	}

	res = flash_readEAR(*dev, spimctrl, &spimctrl->ear);
	if (res < 0) {
		return res;
	}

	return 0;
}


int flash_init(struct _storage_devCtx_t *ctx)
{
	spimctrl_init(ctx->spimctrl);
	int res = flash_query(&ctx->dev, ctx->spimctrl, &ctx->cfi);

	if (res < 0) {
		return res;
	}

	LOG("Flash: %s", ctx->dev->name);

	return 0;
}


void flash_destroy(struct _storage_devCtx_t *ctx)
{
	spimctrl_destroy(ctx->spimctrl);
}
