/*
 * Phoenix-RTOS
 *
 * STM32L1 flash driver.
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Jakub Sejdak, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_H_
#define _FLASHDRV_H_

#include ARCH
#include "rtc.h"
#include "common.h"

#define FLASH_PAGE_SIZE         256
#define FLASH_PROGRAM_1_ADDR    0x08000000
#define FLASH_PROGRAM_2_ADDR    0x08030000
#define FLASH_PROGRAM_SIZE      (192 * 1024)
#define FLASH_EEPROM_1_ADDR     0x08080000
#define FLASH_EEPROM_2_ADDR     0x08081800
#define FLASH_EEPROM_SIZE       (6 * 1024)
#define FLASH_OB_1_ADDR         0x1ff80000
#define FLASH_OB_2_ADDR         0x1ff80080
#define FLASH_OB_SIZE           32


typedef enum { FLASH_EVENT_GET, FLASH_EVENT_SET, FLASH_LOG_GET, FLASH_LOG_SET,
			FLASH_EEPROM_INFO_GET, FLASH_BANK_GET, FLASH_BANK_BREAK, FLASH_ATOM_COPY } devctltype_t ;


typedef struct {
	u64 id;
	u32 events;
	rtctimestamp_t timestamp;
} __attribute__((packed)) flashevent_t;


typedef struct {
	u64 id;
	u64 volume;
	u64 volumeCorr;
	u64 volumeBase;
	rtctimestamp_t timestamp;
} __attribute__((packed)) flashlog_t;


typedef struct {
	u32 addr;
	size_t size;
} __attribute__((packed)) flasheeprominfo_t;


typedef struct {
	u32 dest;
	u32 src;
	size_t len;
} __attribute__((packed)) flashatomcopy_t;


typedef struct {
	devctltype_t type;
	int idx;
	union {
		flashevent_t event;
		flashlog_t log;
		flashatomcopy_t atomcpy;
	};
} __attribute__((packed)) flashdevctl_t;


static inline int flash_activeBank(void)
{
	u32 pc = getPC();

	if (pc < FLASH_PROGRAM_1_ADDR)
		pc += FLASH_PROGRAM_1_ADDR;

	return !(pc < FLASH_PROGRAM_2_ADDR);
}


extern int flash_eventRead(int idx, flashevent_t *event);


extern int flash_eventWrite(flashevent_t *event);


extern int flash_logRead(flashlog_t *log);


extern int flash_logWrite(flashlog_t *log);


extern int flash_read(void *buff, size_t size, u32 addr);


extern int flash_write(void *buff, size_t size, u32 addr);


extern int flash_eepromInfo(unsigned int *addr, size_t *size);


extern int flash_getActiveBank(int *bank);


extern int flash_breakActiveBank(void);


extern int flash_atomCopy(u32 dest, u32 src, size_t len);


extern int eeprom_eraseByte(u32 addr);


extern size_t flash_readData(u32 offset, char *buff, size_t size);


extern size_t flash_writeData(u32 offset, char *buff, size_t size);


extern void flash_init(void);


#endif
