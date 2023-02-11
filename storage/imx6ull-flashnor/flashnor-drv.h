/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI NOR flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHNOR_DRV_H_
#define _FLASHNOR_DRV_H_

#include <unistd.h>
#include <storage/storage.h>


typedef struct {
	ssize_t (*read)(int ndev, off_t offs, void *buff, size_t bufflen);
	ssize_t (*write)(int ndev, off_t offs, const void *buff, size_t bufflen);
	int (*erase)(int ndev, off_t offs, size_t size);
} flashnor_ops_t;


/* information about NOR flash configuration */
typedef struct {
	const char *name;
	size_t size;        /* total NOR size in bytes */
	size_t writeBuffsz; /* write page size in bytes */
	size_t erasesz;     /* erase block size in bytes (multiple of writesize) */
} flashnor_devInfo_t;


typedef struct {
	const flashnor_devInfo_t *devInfo;
	const flashnor_ops_t *ops;
	int ndev;
} flashnor_info_t;


extern void flashnor_drvDone(storage_t *storage);


extern int flashnor_drvInit(const flashnor_info_t *info, storage_t *storage);


#endif
