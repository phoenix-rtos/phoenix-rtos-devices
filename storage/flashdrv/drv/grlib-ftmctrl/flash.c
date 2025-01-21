/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Copyright 2023-2025 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>

#include <flashdrv/common.h>

#include "cmds.h"
#include "flash.h"
#include "flashdrv.h"
#include "ftmctrl.h"

#define FLASH_DEVICES 2


typedef union {
	uint8_t b;
	uint16_t w;
} flash_word_t;


static struct {
	volatile uint8_t *base;
	size_t nmodels;
	const flash_dev_t *devs[FLASH_DEVICES];
} common;


/* Timeout in us */
static int flash_statusPoll(const struct _storage_devCtx_t *ctx, const flash_word_t src, volatile uint8_t *dst, time_t timeout)
{
	bool ready;
	time_t start;
	(void)gettime(&start, NULL);

	for (;;) {
		switch (ftmctrl_portWidth(ctx->ftmctrl)) {
			case 8:
				ready = (src.b == *dst);
				break;

			case 16: {
				uint16_t val = dst[0] | (dst[1] << 8);
				ready = (src.w == val);
				break;
			}

			default:
				return -EINVAL;
		}

		if (ready) {
			break;
		}

		if (timeout > 0u) {
			time_t now;
			(void)gettime(&now, NULL);
			if ((now - start) > timeout) {
				return -ETIME;
			}
		}

		usleep(100);
	}

	return 0;
}


static int flash_statusWait(const struct _storage_devCtx_t *ctx, time_t timeout)
{
	time_t start;
	(void)gettime(&start, NULL);

	while ((ctx->dev->ops->statusRead(common.base) & ctx->dev->statusRdyMask) == 0) {
		if (timeout > 0u) {
			time_t now;
			(void)gettime(&now, NULL);
			if ((now - start) > timeout) {
				return -ETIME;
			}
		}
		usleep(100);
	}

	return 0;
}


static uint16_t flash_deserialize16(uint16_t value)
{
	return ((value & 0xff) << 8) | ((value >> 8) & 0xff);
}


int ftmctrl_flash_writeBuffer(const struct _storage_devCtx_t *ctx, off_t offs, const uint8_t *data, size_t len, time_t timeout)
{
	uint16_t val;
	const int portWidth = ftmctrl_portWidth(ctx->ftmctrl);

	size_t i;
	for (i = 0; i < len; i += portWidth / 8) {
		if (portWidth == 8) {
			if (data[i] != 0xffu) {
				break;
			}
		}
		else if (portWidth == 16) {
			val = data[i] | (data[i + 1] << 8);
			if (val != 0xffffu) {
				break;
			}
		}
		else {
			return -EINVAL;
		}
	}

	len -= i;
	offs += i;
	data += i;

	if (len == 0) {
		return 0;
	}

	off_t sectorOffs = common_getSectorOffset(ctx->sectorsz, offs);

	ctx->dev->ops->issueWriteBuffer(common.base, sectorOffs, offs, len);

	i = 0;
	while (i < len) {
		switch (portWidth) {
			case 8:
				*(common.base + offs + i) = data[i];
				i++;
				break;

			case 16:
				val = data[i] | (data[i + 1] << 8);
				*(uint16_t *)(common.base + offs + i) = val;
				i += 2;
				break;

			default:
				break;
		}
	}

	ctx->dev->ops->issueWriteConfirm(common.base, sectorOffs);

	int res;
	if (ctx->dev->usePolling != 0) {
		flash_word_t word;
		switch (portWidth) {
			case 8:
				word.b = data[len - 1];
				break;

			case 16:
				word.w = data[len - 2] | (data[len - 1] << 8);
				break;

			default:
				return -EINVAL;
		}
		res = flash_statusPoll(ctx, word, common.base + offs + len - (portWidth / 8), timeout);
	}
	else {
		res = flash_statusWait(ctx, timeout);
	}

	int status = ctx->dev->ops->statusCheck(common.base, "write buffer");

	ctx->dev->ops->statusClear(common.base);

	return (res == 0) ? status : res;
}


int ftmctrl_flash_sectorErase(const struct _storage_devCtx_t *ctx, off_t sectorOffs, time_t timeout)
{
	ctx->dev->ops->issueSectorErase(common.base, sectorOffs);

	int res;
	if (ctx->dev->usePolling != 0) {
		flash_word_t word;
		int portWidth = ftmctrl_portWidth(ctx->ftmctrl);
		switch (portWidth) {
			case 8:
				word.b = 0xffu;
				break;

			case 16:
				word.w = 0xffffu;
				break;

			default:
				return -EINVAL;
		}
		res = flash_statusPoll(ctx, word, common.base + sectorOffs + ctx->sectorsz - (portWidth / 8), timeout);
	}
	else {
		res = flash_statusWait(ctx, timeout);
	}

	int status = ctx->dev->ops->statusCheck(common.base, "sector erase");

	ctx->dev->ops->statusClear(common.base);

	return (res == 0) ? status : res;
}


int ftmctrl_flash_chipErase(const struct _storage_devCtx_t *ctx, time_t timeout)
{
	if (ctx->dev->ops->issueChipErase == NULL) {
		return -ENOSYS;
	}

	ctx->dev->ops->issueChipErase(common.base);

	int res = flash_statusWait(ctx, timeout);

	int status = ctx->dev->ops->statusCheck(common.base, "chip erase");

	ctx->dev->ops->statusClear(common.base);

	return (res == 0) ? status : res;
}


void ftmctrl_flash_read(const struct _storage_devCtx_t *ctx, off_t offs, void *buff, size_t len)
{
	ctx->dev->ops->issueReset(common.base);

	memcpy(buff, (void *)(common.base + offs), len);
}


void ftmctrl_flash_printInfo(const struct _storage_devCtx_t *ctx)
{
	LOG("configured %s %u MB flash", ctx->dev->name, CFI_SIZE(ctx->cfi.chipSz) / (1024 * 1024));
}


static void flash_reset(void)
{
	/* Issue both commands as we don't know yet which flash we have */
	*common.base = AMD_CMD_RESET;

	/* Small delay */
	for (volatile int i = 0; i < 1000; i++) { }

	*common.base = INTEL_CMD_RESET;
}


static const flash_dev_t *flash_query(cfi_info_t *cfi)
{
	for (size_t i = 0; i < common.nmodels; i++) {
		const addr_t queryOffs = 0x0;
		const flash_dev_t *dev = common.devs[i];

		flash_reset();
		dev->ops->enterQuery(common.base, queryOffs);

		/* Check 'QRY' header */
		uint8_t buf[3];
		for (size_t j = 0; j < sizeof(cfi->qry); ++j) {
			const size_t offs = offsetof(cfi_info_t, qry);
			buf[j] = *(common.base + queryOffs + (offs + j) * 2);
		}

		if ((buf[0] != 'Q') || (buf[1] != 'R') || (buf[2] != 'Y')) {
			continue;
		}

		uint8_t *ptr = (uint8_t *)cfi;
		for (size_t j = 0; j < sizeof(cfi_info_t); ++j) {
			ptr[j] = *(common.base + queryOffs + j * 2);
		}

		uint16_t device;
		memcpy(&device, &cfi->vendorData[1], sizeof(device));
		/* x8 flash */
		device = flash_deserialize16(device) & 0xff;

		if ((cfi->vendorData[0] != dev->vendor) || (device != (dev->device & 0xff))) {
			/* Command succeeded, but this is not the flash we're checking now */
			dev->ops->exitQuery(common.base);
			continue;
		}

		cfi->cmdSet1 = flash_deserialize16(cfi->cmdSet1);
		cfi->addrExt1 = flash_deserialize16(cfi->addrExt1);
		cfi->cmdSet2 = flash_deserialize16(cfi->cmdSet2);
		cfi->addrExt2 = flash_deserialize16(cfi->addrExt2);
		cfi->fdiDesc = flash_deserialize16(cfi->fdiDesc);
		cfi->bufSz = flash_deserialize16(cfi->bufSz);

		for (size_t j = 0; j < cfi->regionCnt; j++) {
			cfi->regions[j].count = flash_deserialize16(cfi->regions[j].count);
			cfi->regions[j].size = flash_deserialize16(cfi->regions[j].size);
		}

		dev->ops->exitQuery(common.base);

		return dev;
	}

	return NULL;
}


int ftmctrl_flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase)
{
	ftmctrl_amd_register();
	ftmctrl_intel_register();

	/* Temporarily map one page on flash as uncached to be able to read status */
	common.base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, flashBase);
	if (common.base == MAP_FAILED) {
		LOG_ERROR("failed to map flash");
		return -ENOMEM;
	}

	const flash_dev_t *dev = flash_query(&ctx->cfi);

	munmap((void *)common.base, _PAGE_SIZE);

	if (dev == NULL) {
		LOG_ERROR("failed to query flash");
		return -ENODEV;
	}

	ctx->dev = dev;
	ctx->sectorsz = CFI_SIZE(ctx->cfi.chipSz) / (ctx->cfi.regions[0].count + 1);

	/* Map entire flash */
	common.base = mmap(NULL, CFI_SIZE(ctx->cfi.chipSz), PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, flashBase);
	if (common.base == MAP_FAILED) {
		LOG_ERROR("failed to map flash");
		return -ENOMEM;
	}

	return 0;
}


void ftmctrl_flash_destroy(struct _storage_devCtx_t *ctx)
{
	(void)munmap((void *)common.base, CFI_SIZE(ctx->cfi.chipSz));
}


void ftmctrl_flash_register(const flash_dev_t *dev)
{
	if (common.nmodels >= FLASH_DEVICES) {
		LOG("Too many flashes: %s not registered. Please increase FLASH_DEVICES", dev->name);
	}
	else {
		common.devs[common.nmodels++] = dev;
	}
}
