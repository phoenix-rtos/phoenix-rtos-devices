/*
 * Phoenix-RTOS
 *
 * SD Card libstorage-based driver header file
 *
 * Copyright 2023 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SDSTORAGE_DEV_H_
#define _SDSTORAGE_DEV_H_

#define DEVTYPE_POS   (29)
#define DEVTYPE_MASK  (3 << DEVTYPE_POS)
#define DEVTYPE_MTD   (1 << DEVTYPE_POS)
#define DEVTYPE_BLOCK (2 << DEVTYPE_POS)

#define GET_STORAGE_ID(id)     ((id) & ~DEVTYPE_MASK)
#define IS_MTD_DEVICE_ID(id)   (((id)&DEVTYPE_MTD) != 0)
#define IS_BLOCK_DEVICE_ID(id) (((id)&DEVTYPE_BLOCK) != 0)


int sdstorage_handleInsertion(unsigned int slot);


int sdstorage_handleRemoval(unsigned int slot);


int sdstorage_initHost(unsigned int slot);


int sdstorage_runPresenceDetection(void);


void sdstorage_setDefaultCachePolicy(int cachePolicy);

#endif /* _SDSTORAGE_DEV_H_ */
