/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32 flash driver.
 *
 * Copyright 2017 Phoenix Systems
 * Author: Jakub Sejdak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_PRIV_H_
#define _FLASHDRV_PRIV_H_

#include HAL


typedef enum { FLASH_EVENT_GET, FLASH_EVENT_SET, FLASH_LOG_GET, FLASH_LOG_SET,
			FLASH_EEPROM_INFO_GET, FLASH_BANK_GET, FLASH_BANK_BREAK, FLASH_ATOM_COPY } devctltype_t ;


typedef struct {
	devctltype_t type;
	int idx;
	union {
		flashevent_t event;
		flashlog_t log;
		flashatomcopy_t atomcpy;
	};
} __attribute__((packed)) flashdevctl_t;


extern int eeprom_eraseByte(u32 addr);


extern size_t flash_readData(u32 offset, char *buff, size_t size);


extern size_t flash_writeData(u32 offset, char *buff, size_t size);


#endif
