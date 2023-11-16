/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * GR712RC Flash on PROM interface driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>

#include "cmds.h"
#include "flash.h"
#include "flashdrv.h"
#include "ftmctrl.h"

/* Physical addresses */
#define FTMCTRL_BASE 0x80000000
#define FLASH_BASE   0x0

/* Flash status register */
#define STS_WSM_READY      (1 << 7) /* Write state machine ready */
#define STS_ERASE_SUSP     (1 << 6) /* Erase operation suspended */
#define STS_ERASE_CLR_LOCK (1 << 5) /* Erase/clear lock-bit error */
#define STS_PROG_SET_LOCK  (1 << 4) /* Program/set lock-bit error */
#define STS_PROG_VOLT_ERR  (1 << 3) /* Low programming voltage detected */
#define STS_PROG_SUSP      (1 << 2) /* Program operation suspended */
#define STS_DEV_PROTECT    (1 << 1) /* Block lock-bit detected */
#define STS_FULL_CHECK     (STS_ERASE_CLR_LOCK | STS_PROG_SET_LOCK | STS_PROG_VOLT_ERR | STS_DEV_PROTECT)
#define XSR_WRBUF_RDY      (1 << 7) /* Write buffer ready */


static const struct {
	const uint8_t id;
	const char *name;
} flash_vendors[] = {
	{ .id = 0x89, .name = "Intel" }
};


static struct {
	volatile uint8_t *base;
} flash_common;


static uint8_t flash_readStatusCmd(void)
{
	*flash_common.base = FLASH_RD_STATUS;

	return *flash_common.base;
}


static uint8_t flash_readStatusDirect(void)
{
	return *flash_common.base;
}


static void flash_clearStatus(void)
{
	*flash_common.base = FLASH_CLR_STATUS;
}


static int flash_statusCheck(uint8_t status, const char *msg)
{
	int ret = ((status & STS_FULL_CHECK) == 0) ? EOK : -EIO;

	if (ret < 0) {
		LOG_ERROR("%s error: status 0x%x", msg, status);
	}

	flash_clearStatus();

	return ret;
}

/* Timeout in us */
static int flash_statusWait(uint8_t (*readStatus)(void), time_t timeout)
{
	time_t start;
	(void)gettime(&start, NULL);

	while ((readStatus() & STS_WSM_READY) == 0) {
		if (timeout > 0u) {
			time_t now;
			(void)gettime(&now, NULL);
			if ((now - start) > timeout) {
				return -ETIME;
			}
		}
	}

	return EOK;
}


static int flash_clearLockBits(void)
{
	*flash_common.base = FLASH_UNLOCK_CYC1;
	*flash_common.base = FLASH_UNLOCK_CYC2;

	(void)flash_statusWait(flash_readStatusDirect, 0u);

	return flash_statusCheck(flash_readStatusCmd(), "clear lock bits");
}


static uint16_t flash_deserialize16(uint16_t value)
{
	return ((value & 0xff) << 8) | ((value >> 8) & 0xff);
}


int flash_writeBuffer(const struct _storage_devCtx_t *ctx, off_t offs, const uint8_t *data, uint8_t len)
{
	size_t i;

	for (i = 0; i < len; ++i) {
		if (data[i] != 0xff) {
			break;
		}
	}

	if (i == len) {
		return EOK;
	}

	uint8_t xsr;
	do {
		*(flash_common.base + offs) = FLASH_WR_BUF;
		xsr = *flash_common.base;
	} while ((xsr & XSR_WRBUF_RDY) == 0);

	*(flash_common.base + offs) = len - 1;

	for (i = 0; i < len; ++i) {
		*(flash_common.base + offs + i) = data[i];
	}

	*(flash_common.base + offs) = FLASH_WR_CONFIRM;

	if (flash_statusWait(flash_readStatusDirect,
			CFI_TIMEOUT_MAX_PROGRAM(ctx->cfi.toutTypical.bufWrite, ctx->cfi.toutMax.bufWrite)) < 0) {
		return -ETIME;
	}

	return flash_statusCheck(flash_readStatusDirect(), "write buffer");
}


int flash_blockErase(off_t blkAddr, time_t timeout)
{
	*(flash_common.base + blkAddr) = FLASH_BE_CYC1;
	*(flash_common.base + blkAddr) = FLASH_WR_CONFIRM;

	if (flash_statusWait(flash_readStatusDirect, timeout) < 0) {
		return -ETIME;
	}

	return flash_statusCheck(flash_readStatusDirect(), "block erase");
}


void flash_read(const struct _storage_devCtx_t *ctx, off_t offs, void *buff, size_t len)
{
	*flash_common.base = FLASH_RD_ARRAY;

	memcpy(buff, (void *)(flash_common.base + offs), len);
}


void flash_printInfo(flash_cfi_t *cfi)
{
	size_t i;
	const char *vendor;
	for (i = 0; i < sizeof(flash_vendors) / sizeof(flash_vendors[0]); i++) {
		if (flash_vendors[i].id == cfi->vendorData[0]) {
			vendor = flash_vendors[i].name;
			break;
		}
	}

	if (vendor == NULL) {
		vendor = "Unknown";
	}

	LOG("configured %s %u MB flash", vendor, CFI_SIZE(cfi->chipSz) / (1024 * 1024));
}


static void flash_query(flash_cfi_t *cfi)
{
	size_t i;
	uint8_t *ptr = (uint8_t *)cfi;

	*flash_common.base = FLASH_RD_QUERY;
	for (i = 0; i < sizeof(flash_cfi_t); ++i) {
		ptr[i] = *(flash_common.base + i * 2);
	}

	cfi->cmdSet1 = flash_deserialize16(cfi->cmdSet1);
	cfi->addrExt1 = flash_deserialize16(cfi->addrExt1);
	cfi->cmdSet2 = flash_deserialize16(cfi->cmdSet2);
	cfi->addrExt2 = flash_deserialize16(cfi->addrExt2);
	cfi->fdiDesc = flash_deserialize16(cfi->fdiDesc);
	cfi->bufSz = flash_deserialize16(cfi->bufSz);

	for (i = 0; i < cfi->regionCnt; i++) {
		cfi->regions[i].count = flash_deserialize16(cfi->regions[i].count);
		cfi->regions[i].size = flash_deserialize16(cfi->regions[i].size);
	}
}


int flash_init(struct _storage_devCtx_t *ctx)
{
	/* Map one page on flash as uncached to be able to read status */
	flash_common.base = mmap(
		NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, FLASH_BASE);
	if (flash_common.base == MAP_FAILED) {
		LOG_ERROR("failed to map flash");
		return -ENOMEM;
	}

	flash_clearLockBits();

	/* Reset */
	*flash_common.base = FLASH_RD_ARRAY;

	flash_query(&ctx->cfi);

	ctx->blockSz = CFI_SIZE(ctx->cfi.chipSz) / (ctx->cfi.regions[0].count + 1);

	(void)munmap((void *)flash_common.base, _PAGE_SIZE);
	flash_common.base = mmap(
		NULL, CFI_SIZE(ctx->cfi.chipSz), PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, FLASH_BASE);
	if (flash_common.base == MAP_FAILED) {
		LOG_ERROR("failed to map flash");
		return -ENOMEM;
	}

	return EOK;
}
