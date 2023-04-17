/*
 * Phoenix-RTOS
 *
 * Flash emulator
 *
 * Copyright 2021 Phoenix Systems
 * Author: Tomasz Korniluk
 *
 * %LICENSE%
 */

#ifndef HOST_FLASH_H
#define HOST_FLASH_H

#include <sys/types.h>
#include <meterfs.h>


ssize_t hostflash_read(struct _meterfs_devCtx_t *devCtx, off_t offs, void *buff, size_t bufflen);


ssize_t hostflash_write(struct _meterfs_devCtx_t *devCtx, off_t offs, const void *buff, size_t bufflen);


int hostflash_sectorErase(struct _meterfs_devCtx_t *devCtx, off_t offs);


void hostflash_powerCtrl(struct _meterfs_devCtx_t *devCtx, int state);


struct _meterfs_devCtx_t *hostflash_devCtx(void);


int hostflash_init(size_t *flashsz, size_t *sectorsz, const char *fileName);

#endif
