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


#include "flashdrv.h"
#include "rom_api.h"

#include "flash_config.h"


#define QSPI_FREQ_133MHZ 0xc0000008


#define QUAD_FAST_READ_SEQ_ID      0
#define READ_STATUS_REG_SEQ_ID     1
#define WRITE_ENABLE_SEQ_ID        3
#define SECTOR_ERASE_SEQ_ID        5
#define BLOCK_ERASE_SEQ_ID         8
#define PAGE_PROGRAM_SEQ_ID        9
#define CHIP_ERASE_SEQ_ID          11
#define READ_JEDEC_ID_SEQ_ID       12


static int flash_setWEL(flash_context_t *context, uint32_t dstAddr)
{
	flexspi_xfer_t xfer;

	xfer.baseAddress = dstAddr;
	xfer.operation = kFlexSpiOperation_Command;
	xfer.seqId = WRITE_ENABLE_SEQ_ID;
	xfer.seqNum = 1;
	xfer.isParallelModeEnable = 0;

	return -flexspi_norFlashExecuteSeq(context->instance, &xfer);
}


static int flash_writeBytes(flash_context_t *context, uint32_t dstAddr, uint32_t *src, uint32_t size)
{
	flexspi_xfer_t xfer;

	xfer.txSize = size;
	xfer.txBuffer = src;
	xfer.baseAddress = dstAddr;
	xfer.operation = kFlexSpiOperation_Write;
	xfer.seqId = PAGE_PROGRAM_SEQ_ID;
	xfer.seqNum = 1;

	return -flexspi_norFlashExecuteSeq(context->instance, &xfer);
}


static int flash_waitBusBusy(flash_context_t *context)
{
	int err = EOK;
	uint32_t buff;
	uint8_t retrans;
	flexspi_xfer_t xfer;
	const uint8_t MAX_RETRANS = 8;

	retrans = 0;
	xfer.rxSize = 4;
	xfer.rxBuffer = &buff;
	xfer.baseAddress = context->instance;
	xfer.operation = kFlexSpiOperation_Read;
	xfer.seqId = READ_STATUS_REG_SEQ_ID;
	xfer.seqNum = 1;
	xfer.isParallelModeEnable = 0;

	do {
		err = flexspi_norFlashExecuteSeq(context->instance, &xfer);
		usleep(10000);
	} while ((buff & (0x1 << 1)) && (++retrans < MAX_RETRANS) && (err == 0));

	return err;
}


static int flash_getVendorID(uint32_t instance, uint32_t *manID)
{
	flexspi_xfer_t xfer;

	xfer.rxSize = 3;
	xfer.rxBuffer = manID;
	xfer.baseAddress = instance;
	xfer.operation = kFlexSpiOperation_Read;
	xfer.seqId = READ_JEDEC_ID_SEQ_ID;
	xfer.seqNum = 1;
	xfer.isParallelModeEnable = 0;

	return flexspi_norFlashExecuteSeq(instance, &xfer);
}


static int flash_defineFlexSPI(flash_context_t *context)
{
	switch (context->address) {
		case FLASH_EXT_DATA_ADDRESS:
			context->instance = 0;
			context->option.option0 = QSPI_FREQ_133MHZ;
			break;

		case FLASH_INTERNAL_DATA_ADDRESS:
			context->instance = 1;
			context->option.option0 = QSPI_FREQ_133MHZ;
			break;

		default:
			return -ENODEV;
	}

	return EOK;
}


static void flash_setLutTable(flash_context_t *context)
{
	/* QUAD Fast Read */
	context->config.memConfig.lookupTable[4 * QUAD_FAST_READ_SEQ_ID] = 0xa1804eb;
	context->config.memConfig.lookupTable[4 * QUAD_FAST_READ_SEQ_ID + 1] = 0x26043206;
	context->config.memConfig.lookupTable[4 * QUAD_FAST_READ_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * QUAD_FAST_READ_SEQ_ID + 3] = 0;

	/* Read Status Register */
	context->config.memConfig.lookupTable[4 * READ_STATUS_REG_SEQ_ID] = 0x24040405;
	context->config.memConfig.lookupTable[4 * READ_STATUS_REG_SEQ_ID + 1] = 0;
	context->config.memConfig.lookupTable[4 * READ_STATUS_REG_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * READ_STATUS_REG_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(context->instance, READ_STATUS_REG_SEQ_ID, (const uint32_t *)&context->config.memConfig.lookupTable[4 * READ_STATUS_REG_SEQ_ID], 1);


	/* Write Enable */
	context->config.memConfig.lookupTable[4 * WRITE_ENABLE_SEQ_ID] = 0x406;
	context->config.memConfig.lookupTable[4 * WRITE_ENABLE_SEQ_ID + 1] = 0;
	context->config.memConfig.lookupTable[4 * WRITE_ENABLE_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * WRITE_ENABLE_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(context->instance, WRITE_ENABLE_SEQ_ID, (const uint32_t *)&context->config.memConfig.lookupTable[4 * WRITE_ENABLE_SEQ_ID], 1);

	/* Sector Erase */
	context->config.memConfig.lookupTable[4 * SECTOR_ERASE_SEQ_ID] = 0x8180420;
	context->config.memConfig.lookupTable[4 * SECTOR_ERASE_SEQ_ID + 1] = 0;
	context->config.memConfig.lookupTable[4 * SECTOR_ERASE_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * SECTOR_ERASE_SEQ_ID + 3] = 0;

	/* Block Erase */
	context->config.memConfig.lookupTable[4 * BLOCK_ERASE_SEQ_ID] = 0x81804d8;
	context->config.memConfig.lookupTable[4 * BLOCK_ERASE_SEQ_ID + 1] = 0;
	context->config.memConfig.lookupTable[4 * BLOCK_ERASE_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * BLOCK_ERASE_SEQ_ID + 3] = 0;

	/* Page Program */
	context->config.memConfig.lookupTable[4 * PAGE_PROGRAM_SEQ_ID] = 0x8180402;
	context->config.memConfig.lookupTable[4 * PAGE_PROGRAM_SEQ_ID + 1] = 0x2004;
	context->config.memConfig.lookupTable[4 * PAGE_PROGRAM_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * PAGE_PROGRAM_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(context->instance, PAGE_PROGRAM_SEQ_ID, (const uint32_t *)&context->config.memConfig.lookupTable[4 * PAGE_PROGRAM_SEQ_ID], 1);

	/* Chip Erase */
	context->config.memConfig.lookupTable[4 * CHIP_ERASE_SEQ_ID] = 0x0460;
	context->config.memConfig.lookupTable[4 * CHIP_ERASE_SEQ_ID + 1] = 0x2004;
	context->config.memConfig.lookupTable[4 * CHIP_ERASE_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * CHIP_ERASE_SEQ_ID + 3] = 0;

	/* READ JEDEC ID*/
	context->config.memConfig.lookupTable[4 * READ_JEDEC_ID_SEQ_ID] = 0x2404049f;
	context->config.memConfig.lookupTable[4 * READ_JEDEC_ID_SEQ_ID + 1] = 0;
	context->config.memConfig.lookupTable[4 * READ_JEDEC_ID_SEQ_ID + 2] = 0;
	context->config.memConfig.lookupTable[4 * READ_JEDEC_ID_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(context->instance, READ_JEDEC_ID_SEQ_ID, (const uint32_t *)&context->config.memConfig.lookupTable[4 * READ_JEDEC_ID_SEQ_ID], 1);
}


static int flash_isValidAddress(flash_context_t *context, uint32_t addr, size_t size)
{
	if ((addr + size) <= context->properties.size)
		return 0;

	return 1;
}


ssize_t flash_readData(flash_context_t *context, uint32_t offset, char *buff, size_t size)
{
	if (flash_isValidAddress(context, offset, size))
		return -1;

	if (flexspi_norFlashRead(context->instance, &context->config, (uint32_t *)buff, offset, size) < 0)
		return -1;

	return size;
}


ssize_t flash_directBytesWrite(flash_context_t *context, uint32_t offset, const char *buff, size_t size)
{
	int err;
	uint32_t chunk;
	uint32_t len = size;

	while (len) {
		if ((chunk = context->properties.page_size - (offset & 0xff)) > len)
			chunk = len;

		if ((err = flash_waitBusBusy(context)) < 0)
			return -1;

		if ((err = flash_setWEL(context, offset)) < 0)
			return -1;

		if ((err = flash_writeBytes(context, offset, (uint32_t *)buff, chunk)) < 0)
			return -1;

		if ((err = flash_waitBusBusy(context)) < 0)
			return -1;

		offset += chunk;
		len -= chunk;
		buff = (char *)buff + chunk;
	}

	return size - len;
}


ssize_t flash_bufferedPagesWrite(flash_context_t *context, uint32_t offset, const char *buff, size_t size)
{
	uint32_t pageAddr;
	uint16_t sector_id;
	size_t savedBytes = 0;

	if (size % context->properties.page_size)
		return -1;

	if (flash_isValidAddress(context, offset, size))
		return -1;

	while (savedBytes < size) {
		pageAddr = offset + savedBytes;
		sector_id = pageAddr / context->properties.sector_size;

		/* If sector_id has changed, data from previous sector have to be saved and new sector is read. */
		if (sector_id != context->sectorID) {
			flash_sync(context);

			if (flash_readData(context, context->properties.sector_size * sector_id, context->buff, context->properties.sector_size) <= 0)
				return savedBytes;

			if (flexspi_norFlashErase(context->instance, &context->config, context->properties.sector_size * sector_id, context->properties.sector_size) != 0)
				return savedBytes;

			context->sectorID = sector_id;
			context->counter = offset - context->properties.sector_size * context->sectorID;
		}

		memcpy(context->buff + context->counter, buff + savedBytes, context->properties.page_size);

		savedBytes += context->properties.page_size;
		context->counter += context->properties.page_size;

		/* Save filled buffer */
		if (context->counter >= context->properties.sector_size)
			flash_sync(context);
	}

	return size;
}


int flash_chipErase(flash_context_t *context)
{
	return flexspi_norFlashEraseAll(context->instance, &context->config);
}


int flash_sectorErase(flash_context_t *context, uint32_t offset)
{
	if (offset % context->properties.sector_size)
		return -1;

	return flexspi_norFlashErase(context->instance, &context->config, offset, context->properties.sector_size);
}


void flash_sync(flash_context_t *context)
{
	int i;
	uint32_t dstAddr;
	const uint32_t *src;
	const uint32_t pagesNumber = context->properties.sector_size / context->properties.page_size;

	if (context->counter == 0)
		return;

	for (i = 0; i < pagesNumber; ++i) {
		dstAddr = context->sectorID * context->properties.sector_size + i * context->properties.page_size;
		src = (const uint32_t *)(context->buff + i * context->properties.page_size);

		flexspi_norFlashPageProgram(context->instance, &context->config, dstAddr, src);
	}

	context->counter = 0;
	context->sectorID = -1;
}


int flash_init(flash_context_t *context)
{
	int res = EOK;

	context->sectorID = -1;
	context->counter = 0;
	context->buff = NULL;

	if ((res = flash_defineFlexSPI(context)) < 0)
		return res;

	if (flexspi_norGetConfig(context->instance, &context->config, &context->option) != 0)
		return -ENXIO;

	if (flexspi_norFlashInit(context->instance, &context->config) != 0)
		return -ENXIO;

	/* Basic LUT table uses by ROM API. */
	flash_setLutTable(context);

	if (flash_getVendorID(context->instance, &context->flashID) != 0)
		return -ENXIO;

	if (flash_getConfig(context) != 0)
		return -ENXIO;

	return res;
}


void flash_contextDestroy(flash_context_t *context)
{
	flash_sync(context);
	free(context->buff);
}
