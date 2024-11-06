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
#include <sys/mman.h>
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


static struct {
	void *base;
} common;


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


static int flash_waitBusy(const struct flash_dev *dev, struct spimctrl *spimctrl, time_t timeout)
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


int spimctrl_flash_chipErase(const struct _storage_devCtx_t *ctx, time_t timeout)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = ctx->dev->cmds->ce;

	res = flash_writeEnable(ctx->dev, ctx->spimctrl, write_enable);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(ctx->spimctrl, &xfer);
	if (res < 0) {
		return res;
	}

	return flash_waitBusy(ctx->dev, ctx->spimctrl, timeout);
}


int spimctrl_flash_sectorErase(const struct _storage_devCtx_t *ctx, addr_t addr, time_t timeout)
{
	int res = 0;
	struct xferOp xfer;
	uint8_t cmd[4] = { ctx->dev->cmds->se, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };
	size_t regionEraseSz = 0, regionEnd = 0;

	for (size_t i = 0; i < ctx->cfi.regionCnt; i++) {
		regionEnd += (ctx->cfi.regions[i].count + 1) * CFI_REGION_SIZE(ctx->cfi.regions[i].size);
		if (addr < regionEnd) {
			regionEraseSz = ctx->cfi.regions[i].size;
			break;
		}
	}

	if (regionEraseSz == 0) {
		return -EINVAL;
	}

	for (size_t i = 0; i < ctx->sectorsz / regionEraseSz; i++) {
		res = flash_validateEar(ctx->dev, ctx->spimctrl, addr);
		if (res < 0) {
			return res;
		}

		res = flash_writeEnable(ctx->dev, ctx->spimctrl, write_enable);
		if (res < 0) {
			return res;
		}

		xfer.type = xfer_opWrite;
		xfer.cmd = cmd;
		xfer.cmdLen = 4;
		xfer.txData = NULL;
		xfer.dataLen = 0;

		res = spimctrl_xfer(ctx->spimctrl, &xfer);
		if (res < 0) {
			return res;
		}

		res = flash_waitBusy(ctx->dev, ctx->spimctrl, timeout);
		if (res < 0) {
			return res;
		}

		addr += regionEraseSz;
		cmd[1] = (addr >> 16) & 0xff;
		cmd[2] = (addr >> 8) & 0xff;
		cmd[3] = addr & 0xff;
	}

	return 0;
}


int spimctrl_flash_pageProgram(const struct _storage_devCtx_t *ctx, addr_t addr, const void *src, size_t len, time_t timeout)
{
	struct xferOp xfer;
	const uint8_t cmd[4] = { ctx->dev->cmds->pp, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff };

	int res = flash_validateEar(ctx->dev, ctx->spimctrl, addr);
	if (res < 0) {
		return res;
	}

	res = flash_writeEnable(ctx->dev, ctx->spimctrl, write_enable);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = cmd;
	xfer.cmdLen = 4;
	xfer.txData = src;
	xfer.dataLen = len;

	res = spimctrl_xfer(ctx->spimctrl, &xfer);
	if (res < 0) {
		return res;
	}

	res = flash_waitBusy(ctx->dev, ctx->spimctrl, timeout);

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


static ssize_t flash_readAhb(const struct flash_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
	int res = flash_validateEar(dev, spimctrl, addr);
	if (res < 0) {
		return res;
	}

	(void)memcpy(data, (void *)(addr + (uintptr_t)common.base), size);

	return (res < 0) ? res : (ssize_t)size;
}


ssize_t spimctrl_flash_readData(const struct _storage_devCtx_t *ctx, addr_t addr, void *data, size_t size)
{
	if (((addr & 0xff000000) == 0) && (((addr + size) & 0xff000000) != 0)) {
		/* If we'd have to change EAR register during read,
		 * read data through command (can be read without EAR change)
		 */
		return flash_readCmd(ctx->dev, ctx->spimctrl, addr, data, size);
	}
	else {
		/* Direct copy */
		return flash_readAhb(ctx->dev, ctx->spimctrl, addr, data, size);
	}
}


static int flash_readId(const struct spimctrl *spimctrl, cfi_info_t *cfi)
{
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_RDID;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = (uint8_t *)cfi;
	xfer.dataLen = sizeof(*cfi);

	return spimctrl_xfer(spimctrl, &xfer);
}


static uint16_t flash_deserialize16(uint16_t value)
{
	return ((value & 0xff) << 8) | ((value >> 8) & 0xff);
}


static const struct flash_dev *flash_query(struct spimctrl *spimctrl, cfi_info_t *cfi)
{
	if (flash_readId(spimctrl, cfi) < 0) {
		return NULL;
	}

	uint16_t device;
	memcpy(&device, &cfi->vendorData[1], sizeof(device));
	device = flash_deserialize16(device);

	for (size_t i = 0; i < sizeof(flash_devs) / sizeof(flash_devs[0]); ++i) {
		const struct flash_dev *flash = &flash_devs[i];
		if ((cfi->vendorData[0] == flash->vendor) && (device == flash->device)) {

			if (flash_readEAR(flash, spimctrl, &spimctrl->ear) < 0) {
				return NULL;
			}

			return flash;
		}
	}

	return NULL;
}


void spimctrl_flash_printInfo(const struct _storage_devCtx_t *ctx)
{
	LOG("configured %s %u MB flash", ctx->dev->name, CFI_SIZE(ctx->cfi.chipSz) / (1024 * 1024));
}


int spimctrl_flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase)
{
	ctx->dev = flash_query(ctx->spimctrl, &ctx->cfi);

	if (ctx->dev == NULL) {
		return -1;
	}

	ctx->sectorsz = 0;

	for (uint8_t reg = 0; reg < ctx->cfi.regionCnt; ++reg) {
		if (ctx->sectorsz < CFI_REGION_SIZE(ctx->cfi.regions[reg].size)) {
			ctx->sectorsz = CFI_REGION_SIZE(ctx->cfi.regions[reg].size);
		}
	}

	/* Map entire flash */
	common.base = mmap(NULL, CFI_SIZE(ctx->cfi.chipSz), PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, flashBase);
	if (common.base == MAP_FAILED) {
		LOG_ERROR("failed to map flash");
		return -ENOMEM;
	}

	return 0;
}


void spimctrl_flash_destroy(struct _storage_devCtx_t *ctx)
{
	spimctrl_destroy(ctx->spimctrl);
	(void)munmap(common.base, CFI_SIZE(ctx->cfi.chipSz));
}
