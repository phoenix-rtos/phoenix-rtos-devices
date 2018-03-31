/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * FLASH memory MTD driver
 *
 * Copyright 2014 Phoenix Systems
 * Author: Katarzyna Baranowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _MTD_LOCAL_IF_H_
#define _MTD_LOCAL_IF_H_

#include <dev/storage/flash/flash_cfi.h>


enum {
	MTD_TO_ERASE = 0,
	MTD_TO_WRITE,
	MTD_THE_SAME,
};


extern int mtd_getCount(void);
extern int _mtd_init(void);
extern int _mtd_getCfi(unsigned int flNo, flash_cfi_t *);
extern int _mtd_compare(unsigned int flNo, offs_t offs, char *buff, unsigned int len, char **first_diff, char **last_diff);
extern int _mtd_getStatus(unsigned int flNo);
extern int _mtd_erase(unsigned int flNo, offs_t offs);
extern int _mtd_eraseUniform(unsigned int flNo, offs_t offs);
extern int _mtd_eraseAll(unsigned int flNo);

extern int _mtd_read(unsigned int flNo, offs_t offs, char *buff, unsigned int len, unsigned int *readedLen);
extern int _mtd_programPage(unsigned int flNo, offs_t offs, const char *buff, unsigned int len, unsigned int *programmedLen);


/* Dangerous functions. No synchronization is done. Do not use in Phoenix-RTOS */
extern int _mtd_checkBusy(unsigned int flNo);


/* Assumptions: buff, offs and len are 4-aligned, _mtd_checkBusy() returned 0 */
extern int _mtd_readQuick(unsigned int flNo, u32* buff, offs_t offs, unsigned int len);


/* Assumptions: buff, offs and len are 4-aligned, _mtd_checkBusy() returned 0 */
extern int _mtd_programPageQuick(unsigned int flNo, const u32* buff, offs_t offs, unsigned int len);


#endif
