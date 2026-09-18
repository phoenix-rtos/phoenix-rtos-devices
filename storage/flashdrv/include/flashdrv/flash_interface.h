/*
 * Phoenix-RTOS
 *
 * Flash driver interface
 *
 * Copyright 2025 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_FLASH_INTERFACE_H_
#define _FLASHDRV_FLASH_INTERFACE_H_


#include <flashdrv/devctl_params.h>
#include <storage/storage.h>

#define USE_CACHE 0


typedef struct {
	int type;

	union {
		/* eraseSector */
		struct {
			EraseType_t eraseType;
			size_t size;
			uint32_t addr;
		} erase;

		/* SPI mode */
		struct {
			SPIMode_t mode;
			int speed;
		} spi;
	};
} __attribute__((packed)) flash_i_devctl_t;


/* Flash driver ops vtable */
struct flash_driver {
	const char *name;
	storage_t *(*init)(addr_t mctrlBase, addr_t flashBase);

	void (*destroy)(storage_t *strg);

	int (*devCtl)(storage_t *strg, flash_i_devctl_t *devctl);
};


#endif
