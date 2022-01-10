/*
 * Phoenix-RTOS
 *
 * Zynq-7000 nor flash driver
 *
 * Copyright 2021, 20222 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASH_DRIVER_H_
#define _FLASH_DRIVER_H_

#include "flashcfg.h"
#include <stdint.h>


/* Get information about flash memory properties based on CFI (Common Flash Interface) */
const flash_info_t *flashdrv_flashInfo(void);


/* Force writing data keeping in cache buffer to flash memory, returns 0 on success <0 on error */
int flashdrv_sync(void);


/* Erase single sector. 'offs' argument has to be align to sector size, returns 0 on success <0 on error */
int flashdrv_sectorErase(addr_t offs);


/* Erase the whole flash memory, returns 0 on success <0 on error */
int flashdrv_chipErase(void);


/* Below read/write operations return:
 *   >0 read/written data length
 *  -EINVAL - wrong args,
 *  -ETIMEDOUT - timed out waiting for the transfer to succeed */

/* Read data from the given address on flash memory */
ssize_t flashdrv_read(addr_t offs, void *buff, size_t len, time_t timeout);


/* Write data to the given address on flash memory */
ssize_t flashdrv_write(addr_t offs, const void *buff, size_t len);


/* Clean up flash memory driver, returns 0 on success <0 on error */
int flashdrv_done(void);


/* Initialize flash memory, returns 0 on success <0 on error */
int flashdrv_init(void);

#endif
