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

ssize_t hostflash_read(unsigned int addr, void *buff, size_t bufflen);


ssize_t hostflash_write(unsigned int addr, void *buff, size_t bufflen);


int hostflash_sectorErase(unsigned int addr);


int hostflash_chipErase(void);


int hostflash_init(size_t *flashsz, size_t *sectorsz, const char *fileName);

#endif
