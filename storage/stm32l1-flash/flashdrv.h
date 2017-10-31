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

#ifndef _FLASHDRV_H_
#define _FLASHDRV_H_

#include HAL
#include "../rtcdrv.h"

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


static inline int flash_activeBank(void)
{
	u32 pc = hal_cpuGetPC();

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


extern void flash_init(void);


#endif
