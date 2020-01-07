/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <phoenix/arch/imxrt.h>


#include "flashdrv.h"
#include "rom_api.h"

#include "flash_config.h"


#define QSPI_FREQ_133MHZ 0xc0000008


static int flash_defineFlexSPI(flash_context_t *context)
{
	switch (context->address) {
		case FLEXSPI_DATA_ADDRESS :
			context->instance = 0;
			context->option.option0 = QSPI_FREQ_133MHZ;
			break;

		case FLEXSPI2_DATA_ADDRESS :
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
	context->config.memConfig.lookupTable[0] = 0xa1804eb;
	context->config.memConfig.lookupTable[1] = 0x26043206;
	context->config.memConfig.lookupTable[2] = 0;
	context->config.memConfig.lookupTable[3] = 0;

	/* Read Status Register */
	context->config.memConfig.lookupTable[4] = 0x24040405;
	context->config.memConfig.lookupTable[5] = 0;
	context->config.memConfig.lookupTable[6] = 0;
	context->config.memConfig.lookupTable[7] = 0;

	/* Sector Erase */
	context->config.memConfig.lookupTable[20] = 0x8180420;
	context->config.memConfig.lookupTable[21] = 0;
	context->config.memConfig.lookupTable[22] = 0;
	context->config.memConfig.lookupTable[23] = 0;

	/* Block Erase */
	context->config.memConfig.lookupTable[32] = 0x81804d8;
	context->config.memConfig.lookupTable[33] = 0;
	context->config.memConfig.lookupTable[34] = 0;
	context->config.memConfig.lookupTable[35] = 0;

	/* Page Program */
	context->config.memConfig.lookupTable[36] = 0x8180402;
	context->config.memConfig.lookupTable[37] = 0x2004;
	context->config.memConfig.lookupTable[38] = 0;
	context->config.memConfig.lookupTable[39] = 0;

	/* Chip Erase */
	context->config.memConfig.lookupTable[44] = 0x460;
	context->config.memConfig.lookupTable[45] = 0;
	context->config.memConfig.lookupTable[46] = 0;
	context->config.memConfig.lookupTable[47] = 0;
}


static int flash_isValidAddress(flash_context_t *context, uint32_t addr, size_t size)
{
	if ((addr + size) <= context->properties.size)
		return 0;

	return 1;
}


size_t flash_readData(flash_context_t *context, uint32_t offset, char *buff, size_t size)
{
	if (flash_isValidAddress(context, offset, size))
		return 0;

	if (flexspi_norFlashRead(context->instance, &context->config, (uint32_t *)buff, offset, size) < 0)
		return 0;

	return size;
}


size_t flash_writeDataPage(flash_context_t *context, uint32_t offset, const char *buff, size_t size)
{
	uint32_t pageAddr;
	uint16_t sector_id;
	size_t savedBytes = 0;

	if (size % context->properties.page_size)
		return 0;

	if (flash_isValidAddress(context, offset, size))
		return 0;

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

	if ((res = flash_defineFlexSPI(context)) < 0)
		return res;

	if (flexspi_norGetConfig(context->instance, &context->config, &context->option) != 0)
		return -ENXIO;

	if (flexspi_norFlashInit(context->instance, &context->config) != 0)
		return -ENXIO;

	/* Basic LUT table uses by ROM API. */
	flash_setLutTable(context);

	if (flexspi_getVendorID(context->instance, &context->flashID) != 0)
		return -ENXIO;

	if (flash_getConfig(context) != 0)
		return -ENXIO;

	context->sectorID = -1;
	context->counter = 0;

	return res;
}


void flash_contextDestroy(flash_context_t *context)
{
	flash_sync(context);
	free(context->buff);
}
