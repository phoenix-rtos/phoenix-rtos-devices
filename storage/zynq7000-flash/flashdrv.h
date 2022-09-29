/*
 * Phoenix-RTOS
 *
 * Zynq-7000 nor flash driver
 *
 * Copyright 2021, 2022 Phoenix Systems
 * Author: Hubert Buczynski, Malgorzata Wrobel
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASH_DRIVER_H_
#define _FLASH_DRIVER_H_

#include <stdint.h>
#include <storage/storage.h>
#include <mtd/mtd.h>

/* Storage device context definition */
typedef struct _storage_devCtx_t {
	unsigned int id; /* flash device memory id */
} storage_devCtx_t;


/* Clean up flash memory driver, returns 0 on success <0 on error */
extern int flashdrv_done(storage_t *strg);


/* Initialize only a one physicall flash memory via qspi interface, returns:
 * - >0 - number of remaining logic devices for initialization (in case where there are multiple regions)
 * - <0 - on error
 * - 0  - all logic devices are initialized
 */
extern int flashdrv_init(storage_t *strg);


#endif
