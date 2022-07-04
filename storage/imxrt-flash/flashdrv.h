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


#ifndef _IMXRT_FLASH_H_
#define _IMXRT_FLASH_H_

#include <stdio.h>

#define FLASH_EXT_DATA_ADDRESS      (FLEXSPI1_AHB_ADDR)
#define FLASH_INTERNAL_DATA_ADDRESS (FLEXSPI2_AHB_ADDR)


typedef struct {
	uint32_t size;
	uint32_t page_size;
	uint32_t sector_size;
	const char *pVendor;
} flash_properties_t;


typedef struct {
	flash_properties_t properties;
	flexspi_t fspi;
	uint8_t port;

	uint32_t address;
	uint32_t instance;
	uint32_t flashID;

	int sectorID;
	int counter;

	char *buff;
} flash_context_t;


ssize_t flash_readData(flash_context_t *ctx, uint32_t offset, void *buff, size_t size);


ssize_t flash_directBytesWrite(flash_context_t *ctx, uint32_t offset, const void *buff, size_t size);


ssize_t flash_bufferedPagesWrite(flash_context_t *ctx, uint32_t offset, const void *buff, size_t size);


void flash_sync(flash_context_t *ctx);


int flash_chipErase(flash_context_t *ctx);


int flash_sectorErase(flash_context_t *ctx, uint32_t offset);


int flash_init(flash_context_t *ctx);


void flash_contextDestroy(flash_context_t *ctx);


#endif
