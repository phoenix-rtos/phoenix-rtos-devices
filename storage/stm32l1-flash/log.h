/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32 flash driver.
 *
 * Copyright 2017 Phoenix Systems
 * Author: Jakub Sejdak, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_LOG_H_
#define _FLASHDRV_LOG_H_

#include ARCH
#include "flashdrv.h"
#include "flashdrv_priv.h"


#define FLASH_EVENT_COUNT   100
#define FLASH_LOG_COUNT     24


extern void event_read(flashdevctl_t *devctl, flashevent_t *event);


extern void event_write(flashdevctl_t *devctl);


extern void log_read(flashlog_t *log);


extern void log_write(flashdevctl_t *devctl);


extern u32 eeprom_freeAddr(void);


extern size_t eeprom_freeSize(void);


extern void eeprom_init(void);


#endif
