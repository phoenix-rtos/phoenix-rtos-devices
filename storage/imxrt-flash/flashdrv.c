/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash driver
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <phoenix/arch/imxrt.h>

#include "lut.h"
#include "rom_api.h"
#include "flashdrv.h"
#include "flash_config.h"


#define QSPI_FREQ_133MHZ 0xc0000008



/* Flash config commands */

static int flash_setWEL(flash_context_t *ctx, uint32_t dstAddr)
{
	flexspi_xfer_t xfer;

	xfer.baseAddress = dstAddr;
	xfer.operation = kFlexSpiOperation_Command;
	xfer.seqId = WRITE_ENABLE_SEQ_ID;
	xfer.seqNum = 1;
	xfer.isParallelModeEnable = 0;

	return -flexspi_norFlashExecuteSeq(ctx->instance, &xfer);
}


static int flash_writeBytes(flash_context_t *ctx, uint32_t dstAddr, uint32_t *src, uint32_t size)
{
	flexspi_xfer_t xfer;

	xfer.txSize = size;
	xfer.txBuffer = src;
	xfer.baseAddress = dstAddr;
	xfer.operation = kFlexSpiOperation_Write;
	xfer.seqId = PAGE_PROGRAM_SEQ_ID;
	xfer.seqNum = 1;

	return -flexspi_norFlashExecuteSeq(ctx->instance, &xfer);
}


static int flash_waitBusBusy(flash_context_t *ctx)
{
	int err = EOK;
	uint32_t buff;
	uint8_t retrans;
	flexspi_xfer_t xfer;
	const uint8_t MAX_RETRANS = 8;

	retrans = 0;
	xfer.rxSize = 4;
	xfer.rxBuffer = &buff;
	xfer.baseAddress = ctx->instance;
	xfer.operation = kFlexSpiOperation_Read;
	xfer.seqId = READ_STATUS_REG_SEQ_ID;
	xfer.seqNum = 1;
	xfer.isParallelModeEnable = 0;

	do {
		err = flexspi_norFlashExecuteSeq(ctx->instance, &xfer);
		usleep(10000);
	} while ((buff & (0x1 << 1)) && (++retrans < MAX_RETRANS) && (err == 0));

	return err;
}


static int flash_getVendorID(flash_context_t *ctx, uint32_t *manID)
{
	flexspi_xfer_t xfer;

	/* READ JEDEC ID*/
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_RDID, LUT_CMD_READ, LUT_PAD1, FLASH_SPANSION_CMD_WRDI); // 0x2404049f
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, READ_JEDEC_ID_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID], 1);

	xfer.rxSize = 3;
	xfer.rxBuffer = manID;
	xfer.baseAddress = ctx->instance;
	xfer.operation = kFlexSpiOperation_Read;
	xfer.seqId = READ_JEDEC_ID_SEQ_ID;
	xfer.seqNum = 1;
	xfer.isParallelModeEnable = 0;

	return flexspi_norFlashExecuteSeq(ctx->instance, &xfer);
}


/* Read/write/erase operation via ROM API */

static int flash_isValidAddress(flash_context_t *context, uint32_t addr, size_t size)
{
	if ((addr + size) <= context->properties.size)
		return 0;

	return 1;
}


ssize_t flash_readData(flash_context_t *ctx, uint32_t offset, char *buff, size_t size)
{
	if (flash_isValidAddress(ctx, offset, size))
		return -1;

	if (flexspi_norFlashRead(ctx->instance, &ctx->config, buff, offset, size) < 0)
		return -1;

	return size;
}


ssize_t flash_directBytesWrite(flash_context_t *ctx, uint32_t offset, const char *buff, size_t size)
{
	int err;
	uint32_t chunk;
	uint32_t len = size;

	while (len) {
		if ((chunk = ctx->properties.page_size - (offset & 0xff)) > len)
			chunk = len;

		if ((err = flash_waitBusBusy(ctx)) < 0)
			return -1;

		if ((err = flash_setWEL(ctx, offset)) < 0)
			return -1;

		if ((err = flash_writeBytes(ctx, offset, (uint32_t *)buff, chunk)) < 0)
			return -1;

		if ((err = flash_waitBusBusy(ctx)) < 0)
			return -1;

		offset += chunk;
		len -= chunk;
		buff = (char *)buff + chunk;
	}

	return size - len;
}


ssize_t flash_bufferedPagesWrite(flash_context_t *ctx, uint32_t offset, const char *buff, size_t size)
{
	uint32_t pageAddr;
	uint16_t sector_id;
	size_t savedBytes = 0;

	if (size % ctx->properties.page_size)
		return -1;

	if (flash_isValidAddress(ctx, offset, size))
		return -1;

	while (savedBytes < size) {
		pageAddr = offset + savedBytes;
		sector_id = pageAddr / ctx->properties.sector_size;

		/* If sector_id has changed, data from previous sector have to be saved and new sector is read. */
		if (sector_id != ctx->sectorID) {
			flash_sync(ctx);

			if (flash_readData(ctx, ctx->properties.sector_size * sector_id, ctx->buff, ctx->properties.sector_size) <= 0)
				return savedBytes;

			if (flexspi_norFlashErase(ctx->instance, &ctx->config, ctx->properties.sector_size * sector_id, ctx->properties.sector_size) != 0)
				return savedBytes;

			ctx->sectorID = sector_id;
			ctx->counter = offset - ctx->properties.sector_size * ctx->sectorID;
		}

		memcpy(ctx->buff + ctx->counter, buff + savedBytes, ctx->properties.page_size);

		savedBytes += ctx->properties.page_size;
		ctx->counter += ctx->properties.page_size;

		/* Save filled buffer */
		if (ctx->counter >= ctx->properties.sector_size)
			flash_sync(ctx);
	}

	return size;
}


int flash_chipErase(flash_context_t *ctx)
{
	return flexspi_norFlashEraseAll(ctx->instance, &ctx->config);
}


int flash_sectorErase(flash_context_t *ctx, uint32_t offset)
{
	if (offset % ctx->properties.sector_size)
		return -1;

	return flexspi_norFlashErase(ctx->instance, &ctx->config, offset, ctx->properties.sector_size);
}


void flash_sync(flash_context_t *ctx)
{
	int i;
	uint32_t dstAddr;
	const uint32_t *src;
	const uint32_t pagesNumber = ctx->properties.sector_size / ctx->properties.page_size;

	if (ctx->counter == 0)
		return;

	for (i = 0; i < pagesNumber; ++i) {
		dstAddr = ctx->sectorID * ctx->properties.sector_size + i * ctx->properties.page_size;
		src = (const uint32_t *)(ctx->buff + i * ctx->properties.page_size);

		flexspi_norFlashPageProgram(ctx->instance, &ctx->config, dstAddr, src);
	}

	ctx->counter = 0;
	ctx->sectorID = -1;
}


/* Init functions */

static int flash_defineFlexSPI(flash_context_t *ctx)
{
	switch (ctx->address) {
		case FLASH_EXT_DATA_ADDRESS:
			ctx->instance = 0;
			ctx->option.option0 = QSPI_FREQ_133MHZ;
			ctx->option.option1 = 0;
			break;

		case FLASH_INTERNAL_DATA_ADDRESS:
			ctx->instance = 1;
			ctx->option.option0 = QSPI_FREQ_133MHZ;
			ctx->option.option1 = 0;
			break;

		default:
			return -ENODEV;
	}

	return EOK;
}


int flash_init(flash_context_t *ctx)
{
	int res = EOK;

	ctx->sectorID = -1;
	ctx->counter = 0;
	ctx->buff = NULL;

	ctx->config.ipcmdSerialClkFreq = 8;
	ctx->config.mem.serialClkFreq = 8;
	ctx->config.mem.sflashPadType = 4;


	if ((res = flash_defineFlexSPI(ctx)) < 0)
		return res;

	if (flexspi_norGetConfig(ctx->instance, &ctx->config, &ctx->option) != 0)
		return -ENXIO;

	if (flexspi_norFlashInit(ctx->instance, &ctx->config) != 0)
		return -ENXIO;

	if (flash_getVendorID(ctx, &ctx->flashID) != 0)
		return -ENXIO;

	if (flash_getConfig(ctx) != 0)
		return -ENXIO;

	return res;
}


void flash_contextDestroy(flash_context_t *ctx)
{
	flash_sync(ctx);
	free(ctx->buff);
}
