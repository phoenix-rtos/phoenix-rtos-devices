/*
 * Phoenix-RTOS
 *
 * STM32 external Flash driver API
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _EXT_FLASH_H_
#define _EXT_FLASH_H_

#include "stm32l4-multi.h"
#include "common.h"


int extFlash_fn_init(void);


ssize_t extFlash_fn_read(unsigned int xspiDevice, addr_t offs, void *buff, size_t len);


ssize_t extFlash_fn_write(unsigned int xspiDevice, addr_t offs, const void *buff, size_t len);


ssize_t extFlash_fn_erase(unsigned int xspiDevice, addr_t offs, size_t len);


int extFlash_fn_sync(unsigned int xspiDevice);


int extFlash_fn_getInfo(unsigned int xspiDevice, extFlashDef_t *info);


#endif /* _EXT_FLASH_H_ */
