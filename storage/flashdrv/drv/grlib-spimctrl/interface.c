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

#include "interface.h"
#include "../grlib-spimctrl/flashdrv.h"
#include <errno.h>


void flash_destroy(struct _storage_devCtx_t *ctx)
{
    if (ctx->isCfi) {
        spimctrl_flash_destroy(ctx);
    }
    else {
        nor_destroy(ctx);
    }
}


int flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase)
{
    int res = 0;

    if (ctx->isCfi) {
        res = spimctrl_flash_init(ctx, flashBase);
    }
    else {
        res = nor_flash_init(ctx, flashBase);
    }

    return res;
}


time_t flash_timeout(const struct _storage_devCtx_t *ctx, flashTimeout_t timeoutWhat)
{
    time_t timeout = 0;

    if (ctx->isCfi) {
        switch (timeoutWhat)
        {
            case pageProgram:
                timeout = CFI_TIMEOUT_MAX_PROGRAM(ctx->flash_data.cfi.toutTypical.bufWrite, ctx->flash_data.cfi.toutMax.bufWrite);
                break;

            case eraseChip:
                timeout = CFI_TIMEOUT_MAX_ERASE(ctx->flash_data.cfi.toutTypical.chipErase, ctx->flash_data.cfi.toutMax.chipErase);
                break;

            case eraseSector:
                timeout = CFI_TIMEOUT_MAX_ERASE(ctx->flash_data.cfi.toutTypical.blkErase, ctx->flash_data.cfi.toutMax.blkErase);
                break;
            
            default:
                break;
        }
    }
    else {
        switch (timeoutWhat)
        {
            case pageProgram:
                timeout = ctx->flash_data.sfdp->tPP;
                break;

            case eraseChip:
                timeout = ((ctx->flash_data.sfdp->tCE) * (ctx->flash_data.sfdp->stacked));
                break;

            case eraseSector:
                timeout = ctx->flash_data.sfdp->tSE;
                break;
            
            default:
                break;
        }
    }

    return timeout;
}


size_t flash_segmSize(const struct _storage_devCtx_t *ctx, segmSize_t sizeWhat)
{
    size_t segmSize = 0;

    if (ctx->isCfi) {
        switch (sizeWhat)
        {
            case bufSize:
                segmSize = CFI_SIZE(ctx->flash_data.cfi.bufSz);
                break;

            case sectSize:
                segmSize = ctx->sectorsz;
                break;
            
            default:
                break;
        }
    }
    else {
        switch (sizeWhat)
        {
            case bufSize:
                segmSize = ctx->flash_data.sfdp->pageSz;
                break;

            case sectSize:
                segmSize = ctx->flash_data.sfdp->sectorSz;
                LOG_ERROR("sectorsize: %zu\n", segmSize);
                break;
            
            default:
                break;
        }
    }

    return segmSize;
}


const char* flash_name(const struct _storage_devCtx_t *ctx)
{
    if (ctx->isCfi) {
        return ctx->dev->name;
    }
    else {
        return ctx->flash_data.sfdp->name;
    }
}


int flash_pageProgram(const struct _storage_devCtx_t *ctx, addr_t addr, const void *src, size_t len, time_t timeout)
{
    if (ctx->isCfi) {
        return spimctrl_flash_pageProgram(ctx, addr, src, len, timeout);
    }
    else {
        return nor_pageProgram(ctx->spimctrl, addr, src, len, timeout);
    }
}


int flash_chipErase(const struct _storage_devCtx_t *ctx, time_t timeout)
{
    if (ctx->isCfi) {
        return spimctrl_flash_chipErase(ctx, timeout);
    }
    else {
        return nor_eraseChip(ctx->spimctrl, timeout);
    }
}


int flash_sectorErase(const struct _storage_devCtx_t *ctx, addr_t addr, time_t timeout)
{
    if (ctx->isCfi) {
        return spimctrl_flash_sectorErase(ctx, addr, timeout);
    }
    else {
        return nor_eraseSector(ctx->spimctrl, addr, timeout);
    }
}


ssize_t flash_readData(const struct _storage_devCtx_t *ctx, addr_t addr, void *data, size_t size)
{

    if (ctx->isCfi) {
        return spimctrl_flash_readData(ctx, addr, data, size);
    }
    else {
        return nor_readData(ctx->spimctrl, addr, data, size);
    }
}


size_t flash_size(const struct _storage_devCtx_t *ctx)
{
    size_t fsize = 0;

    if (ctx->isCfi) {
        fsize = CFI_SIZE(ctx->flash_data.cfi.chipSz);
    }
    else {
        fsize = ((ctx->flash_data.sfdp->totalSz) * (ctx->flash_data.sfdp->stacked));
    }  

    return fsize;
}


void flash_printInfo(const struct _storage_devCtx_t *ctx)
{
    if (ctx->isCfi) {
        spimctrl_flash_printInfo(ctx);
    }
    else {
        nor_printInfo(ctx);
    } 
}

