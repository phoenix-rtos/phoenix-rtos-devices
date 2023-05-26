/*
 * Phoenix-RTOS
 *
 * GR716 Flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GR716_FLASH_H_
#define _GR716_FLASH_H_

#include <stdint.h>
#include <sys/types.h>

#include "spimctrl.h"
#include "nor/nor.h"


typedef struct {
	const char *vendor;
	const struct nor_info *properties;
	spimctrl_t spimctrl;
	uint8_t port;

	uint32_t address;
	uint32_t instance;

	addr_t sectorBufAddr;
	uint8_t sectorBufDirty;
	uint8_t *sectorBuf;
} flash_context_t;


ssize_t flash_readData(flash_context_t *ctx, addr_t offs, void *buff, size_t len);


ssize_t flash_directWrite(flash_context_t *ctx, addr_t offs, const void *buff, size_t len);


ssize_t flash_bufferedWrite(flash_context_t *ctx, addr_t offs, const void *buff, size_t len);


int flash_sync(flash_context_t *ctx);


int flash_sectorErase(flash_context_t *ctx, addr_t offs);


int flash_chipErase(flash_context_t *ctx);


int flash_init(flash_context_t *ctx, int instance);


void flash_contextDestroy(flash_context_t *ctx);


#endif
