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

#ifndef _MTD_IF_H_
#define _MTD_IF_H_

#include <dev/storage/flash/mtd_local_if.h>


/* Function brings up FLASH memory MTD driver */
extern int _mtd_devInit(void);

extern void mtd_lock(unsigned int mtdNo);
extern void mtd_unlock(unsigned int mtdNo);

extern int _mtd_programPageWithWait(unsigned int mtdNo, offs_t offs, char *buff, unsigned int len, unsigned int *programmedLen);
extern int _mtd_programWithWait(unsigned int mtdNo, offs_t offs, char *buff, unsigned int len, unsigned int *programmedLen);
extern int _mtd_eraseWithWait(unsigned int mtdNo, offs_t offs);
extern int _mtd_eraseUniformWithWait(unsigned int mtdNo, offs_t offs);
extern int _mtd_eraseAllWithWait(unsigned int mtdNo);
extern int _mtd_programWithWait(unsigned int mtdNo, offs_t offs, char *buff, unsigned int len, unsigned int *programmedLen);


#endif
