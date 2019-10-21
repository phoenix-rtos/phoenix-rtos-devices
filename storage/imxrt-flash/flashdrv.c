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

#include "flashdrv.h"
#include "rom_api.h"


static int flash_getProperties(flash_context_t* context)
{
	switch (context->address) {
		case FLEXSPI_DATA_ADDRESS :
			/* TO DO */
			break;

		case FLEXSPI2_DATA_ADDRESS :
			context->instance = 1;
			context->option.option0.U = 0xC0000008;

			context->properties.size = 0x400000;
			context->properties.page_size = 0x100;
			context->properties.sector_size = 0x1000;

			context->buff = malloc(context->properties.sector_size);

			if (context->buff == NULL)
				return -EFAULT;

			break;

		default:
			return -ENODEV;
	}

	return EOK;
}


static int flash_isValidAddress(flash_context_t* context, uint32_t addr, size_t size)
{
	if ((addr + size) <= context->properties.size)
		return EOK;

	return -EFAULT;
}


size_t flash_readData(flash_context_t* context, uint32_t offset, char *buff, size_t size)
{
	if (flash_isValidAddress(context, offset, size) < 0)
		return 0;

	volatile uint32_t *addr = (void *)(context->address + offset);
	memcpy(buff, (void* )addr, size);

	return size;
}


size_t flash_writeDataPage(flash_context_t *context, uint32_t offset, const char *buff, size_t size)
{
	uint32_t pageAddr;
	uint16_t sector_id;
	size_t savedBytes = 0;

	if (size % context->properties.page_size)
		return 0;

	if (flash_isValidAddress(context, offset, size) < 0)
		return 0;

	while (savedBytes < size) {
		pageAddr = offset + savedBytes;
		sector_id = pageAddr / context->properties.sector_size;

		/* If sector_id has changed, data from previous sector have to be saved and new sector is read. */
		if (sector_id != context->id) {
			flash_sync(context);

			if (flash_readData(context, context->properties.sector_size * sector_id, context->buff, context->properties.sector_size) <= 0)
				return savedBytes;

			if (flexspi_nor_flash_erase(context->instance, &context->config, context->properties.sector_size * sector_id, context->properties.sector_size) != 0)
				return savedBytes;

			context->id = sector_id;
			context->counter = offset - context->properties.sector_size * context->id;
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


void flash_sync(flash_context_t* context)
{
	int i;
	uint32_t dstAddr;
	const uint32_t* src;

	if (context->counter == 0)
		return;

	for (i = 0; i < context->properties.sector_size / context->properties.page_size; ++i) {
		dstAddr = context->id * context->properties.sector_size + i * context->properties.page_size;
		src = (const uint32_t* )(context->buff + i * context->properties.page_size);

		flexspi_nor_flash_page_program(context->instance, &context->config, dstAddr, src);
	}

	context->counter = 0;
	context->id = -1;
}


int flash_init(flash_context_t* context)
{
	if (flash_getProperties(context) < 0)
		return -ENODEV;

	if (flexspi_nor_get_config(context->instance, &context->config, &context->option) != 0)
		return -ENXIO;

	if (flexspi_nor_flash_init(context->instance, &context->config) != 0)
		return -ENXIO;

	context->id = -1;
	context->counter = 0;

	return EOK;
}


void flash_contextDestroy(flash_context_t* context)
{
	flash_sync(context);
	free(context->buff);
}
