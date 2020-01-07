/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash Configurator
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <errno.h>

#include "flash_config.h"

#define WINDBOND_W25Q32JV_IQ 0x4016
#define ISSI_DEV_IS25WP064A 0x7017
#define MICRON_MT25QL512ABB 0xba20

#define GET_MANUFACTURE_ID(flashID) (flashID & 0xff)
#define GET_DEVICE_ID(flashID) (((flashID >> 16) & 0xff) | (flashID & (0xff << 8)))


enum {	flash_windbond = 0xef, flash_issi = 0x9d, flash_micron = 0x20 };


static int flash_getWindbondConfig(flash_context_t *context)
{
	switch (GET_DEVICE_ID(context->flashID)) {
		case WINDBOND_W25Q32JV_IQ :
			context->properties.size = 0x400000;
			context->properties.page_size = 0x100;
			context->properties.sector_size = 0x1000;
			context->buff = malloc(context->properties.sector_size);

			if (context->buff == NULL)
				return -ENOMEM;
			break;

		default :
			return -ENODEV;
	}

	return EOK;
}


static int flash_getIssiConfig(flash_context_t *context)
{
	switch (GET_DEVICE_ID(context->flashID)) {
		case ISSI_DEV_IS25WP064A :
			context->properties.size = 0x800000;
			context->properties.page_size = 0x100;
			context->properties.sector_size = 0x1000;
			context->buff = malloc(context->properties.sector_size);

			if (context->buff == NULL)
				return -ENOMEM;
			break;

		default :
			return -ENODEV;
	}

	return EOK;
}


static int flash_getMicronConfig(flash_context_t *context)
{
	switch (GET_DEVICE_ID(context->flashID)) {
		case MICRON_MT25QL512ABB :
			context->properties.size = 0x400000;
			context->properties.page_size = 0x100;
			context->properties.sector_size = 0x1000;
			context->buff = malloc(context->properties.sector_size);

			if (context->buff == NULL)
				return -ENOMEM;
			break;

		default :
			return -ENODEV;
	}

	return EOK;
}


int flash_getConfig(flash_context_t *context)
{
	switch (GET_MANUFACTURE_ID(context->flashID)) {
		case flash_windbond :
			if (flash_getWindbondConfig(context) < 0)
				return -ENODEV;
			break;

		case flash_issi :
			if (flash_getIssiConfig(context) < 0)
				return -ENODEV;
			break;

		case flash_micron :
			if (flash_getMicronConfig(context) < 0)
				return -ENODEV;
			break;

		default:
			return -ENODEV;
	}

	return EOK;
}
