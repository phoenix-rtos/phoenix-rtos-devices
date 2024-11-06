/*
 * Phoenix-RTOS
 *
 * Flash driver interface
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_FLASH_INTERFACE_H_
#define _FLASHDRV_FLASH_INTERFACE_H_


#include <storage/storage.h>


struct flash_driver {
	const char *name;
	storage_t *(*init)(addr_t mctrlBase, addr_t flashBase);
	void (*destroy)(storage_t *strg);
};


#endif
