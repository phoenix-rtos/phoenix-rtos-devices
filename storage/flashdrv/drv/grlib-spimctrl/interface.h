/*
 * Phoenix-RTOS
 *
 * Interface wrapping around CFI and SFDP compatible flash devices.
 * 
 * TBD: if more standards are supported, a virtual table shall be implemented.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../cfi-flash/cfiFlash.h"
#include "../sfdp-flash/sfdpFlash.h"


typedef enum flashTimeout {
    pageProgram = 0,
    eraseChip,
    eraseSector,
    maxTimeoutType
} flashTimeout_t;


typedef enum segmSize {
    bufSize,
    sectSize,
    maxSegmSize
} segmSize_t;


void flash_destroy(struct _storage_devCtx_t *ctx);


int flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase);


time_t flash_timeout(const struct _storage_devCtx_t *ctx, flashTimeout_t timeoutWhat);


size_t flash_segmSize(const struct _storage_devCtx_t *ctx, segmSize_t sizeWhat);


const char* flash_name(const struct _storage_devCtx_t *ctx);


int flash_pageProgram(const struct _storage_devCtx_t *ctx, addr_t addr, const void *src, size_t len, time_t timeout);


int flash_chipErase(const struct _storage_devCtx_t *ctx, time_t timeout);


int flash_sectorErase(const struct _storage_devCtx_t *ctx, addr_t addr, time_t timeout);


ssize_t flash_readData(const struct _storage_devCtx_t *ctx, addr_t addr, void *buff, size_t len);


size_t flash_size(const struct _storage_devCtx_t *ctx);


void flash_printInfo(const struct _storage_devCtx_t *ctx);