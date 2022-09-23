/*
 * Phoenix-RTOS
 *
 * STM32L4 flash driver.
 *
 * Copyright 2017, 2018, 2022 Phoenix Systems
 * Author: Jakub Sejdak, Aleksander Kaminski, Tomasz Korniluk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>
#include <stdlib.h> /* For stddef.h */

#include "stm32l4-multi.h"

#define FLASH_PAGE_SIZE         2048
#define FLASH_OB_1_ADDR         0x1fff7800
#define FLASH_OB_2_ADDR         0x1ffff800
#define FLASH_OB_SIZE           16

#ifndef FLASH_PROGRAM_1_ADDR
#define FLASH_PROGRAM_1_ADDR 0x08000000
#endif

#ifndef FLASH_PROGRAM_2_ADDR
#define FLASH_PROGRAM_2_ADDR 0x08080000
#endif

#ifndef FLASH_PROGRAM_BANK_SIZE
#define FLASH_PROGRAM_BANK_SIZE (512 * 1024)
#endif

#define OTP_ADDR 0x1fff7000
#define OTP_SIZE (128 * 8)

static inline uint32_t getPC(void)
{
	uint32_t ret;

	__asm__ volatile ("mov %0, pc" : "=r" (ret));

	return ret;
}


static inline int flash_activeBank(void)
{
	uint32_t pc = getPC();

	if (pc < FLASH_PROGRAM_1_ADDR)
		pc += FLASH_PROGRAM_1_ADDR;

	return !(pc < FLASH_PROGRAM_2_ADDR);
}


extern size_t flash_readData(uint32_t offset, char *buff, size_t size);


extern size_t flash_writeData(uint32_t offset, const char *buff, size_t size);


extern ssize_t flash_readOtp(uint32_t offset, char *buff, size_t size);


extern ssize_t flash_writeOtp(uint32_t offset, const char *buff, size_t size);


extern int flash_switchBanks(void);


extern void flash_getInfo(flashinfo_t *info);


extern ssize_t flash_writeRaw(uint32_t offset, const char *buff, size_t size);


extern ssize_t flash_erasePage(uint32_t offset);


extern int flash_init(void);


#endif
