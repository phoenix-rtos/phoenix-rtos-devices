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


#ifndef _IMXRT_FLASH_H_
#define _IMXRT_FLASH_H_

#include <stdio.h>

#include "rom_api.h"

#define FLEXSPI_DATA_ADDRESS 0x60000000
#define FLEXSPI2_DATA_ADDRESS 0x70000000


typedef struct _flash_properties_t {
	uint32_t size;
	uint32_t page_size;
	uint32_t sector_size;
} __attribute__((packed)) flash_properties_t;


typedef struct _flash_context_t {
	int id;
	int counter;

	char* buff;

	uint32_t address;
	uint32_t instance;

	flash_properties_t properties;
	serial_nor_config_option_t option;
	flexspi_nor_config_t config;
} __attribute__((packed)) flash_context_t;


size_t flash_readData(flash_context_t* context, uint32_t offset, char *buff, size_t size);


size_t flash_writeDataPage(flash_context_t* context, uint32_t offset, const char *buff, size_t size);


void flash_sync(flash_context_t* context);


int flash_init(flash_context_t* context);


void flash_contextDestroy(flash_context_t* context);


#endif
