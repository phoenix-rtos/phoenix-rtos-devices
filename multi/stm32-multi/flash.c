/*
 * Phoenix-RTOS
 *
 * STM32L1 internal flash driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski, Jakub Sejdak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include ARCH
#include <sys/threads.h>
#include <sys/pwman.h>
#include <unistd.h>
#include <errno.h>

#include "flash.h"
#include "common.h"

#ifndef NDEBUG
static const char drvname[] = "flash";
#endif


enum { flash_pecr = 1, flash_pekeyr = 3, flash_prgkeyr, flash_oprkeyr, flash_sr, flash_obr };


enum { EVENTS = 0, LOGS };


typedef struct {
	u32 addr;
	int currIdx;
	u64 currId;
	int entrySize;
	int entryCount;
} __attribute__((packed)) logarea_t;


struct {
	volatile unsigned int *flash;
	unsigned int port;
	handle_t lock;

	char buff[64];
	char page[256];
} flash_common;


static int _flash_wait(void)
{
	int i;

	for (i = 0; i < 10; ++i) {
		/* Check if flash is busy. */
		if ((*(flash_common.flash + flash_sr) & 0x1) == 0x1) {
			keepidle(1);
			usleep(10000);
			keepidle(0);
			continue;
		}

		/* Check if flash is write protected. */
		if (*(flash_common.flash + flash_sr) & (1 << 8))
			return 1;

		return (*(flash_common.flash + flash_sr) & 0x1e00);
	}

	return 1;
}


static inline void _flash_clearFlags(void)
{
	*(flash_common.flash + flash_sr) |= 0x3f02;
}


static inline int eeprom_isValidAdress(u32 addr)
{
	return (addr >= FLASH_EEPROM_1_ADDR && addr <= (FLASH_EEPROM_2_ADDR + FLASH_EEPROM_SIZE));
}


static inline void _eeprom_lock(void)
{
	*(flash_common.flash + flash_pecr) |= (1 << 0);
	dataBarier();
}


static void _eeprom_unlock(void)
{
	while (*(flash_common.flash + flash_pecr) & (1 << 0)) {
		*(flash_common.flash + flash_pekeyr) = 0x89abcdef;
		dataBarier();
		*(flash_common.flash + flash_pekeyr) = 0x02030405;
		dataBarier();
	}
}


static int _eeprom_eraseByte(u32 addr)
{
	int err;

	_eeprom_unlock();
	_flash_clearFlags();

	if ((err = _flash_wait()) == 0)
		*(volatile u8 *) addr = 0x0;

	_eeprom_lock();

	return err;
}


static int _eeprom_writeByte(u32 addr, char value)
{
	int err;

	_eeprom_unlock();
	_flash_clearFlags();

	if ((err = _flash_wait()) == 0) {
		*(volatile u8 *) addr = value;
		err = _flash_wait();
	}

	_eeprom_lock();

	return err;
}


static size_t eeprom_readData(u32 offset, char *buff, size_t size)
{
	unsigned int i;

	mutexLock(flash_common.lock);
	for (i = 0; i < size; ++i)
		buff[i] = *((volatile u8 *) offset + i);
	mutexUnlock(flash_common.lock);

	return i;
}


static size_t eeprom_writeData(u32 offset, const char *buff, size_t size)
{
	unsigned int i;

	for (i = 0; i < size; ++i) {
		mutexLock(flash_common.lock);
		_eeprom_eraseByte(offset + i);
		_eeprom_writeByte(offset + i, buff[i]);
		mutexUnlock(flash_common.lock);
	}

	return i;
}


static inline int ob_isValidAdress(u32 addr)
{
	/* Address must be even. */
	if (addr & 1)
		return 0;

	if (!flash_activeBank()) {
		if (addr >= FLASH_OB_1_ADDR && addr <= (FLASH_OB_1_ADDR + FLASH_OB_SIZE))
			return 1;
	}
	else {
		if (addr >= FLASH_OB_2_ADDR && addr <= (FLASH_OB_2_ADDR + FLASH_OB_SIZE))
			return 1;
	}

	return 0;
}


static inline void _ob_lock(void)
{
	*(flash_common.flash + flash_pecr) |= (1 << 2);
}


static void _ob_unlock(void)
{
	while (*(flash_common.flash + flash_pecr) & (1 << 2)) {
		_eeprom_unlock();

		*(flash_common.flash + flash_oprkeyr) = 0xfbead9c8;
		dataBarier();
		*(flash_common.flash + flash_oprkeyr) = 0x24252627;
		dataBarier();
	}
}


static int ob_writeByte(u32 addr, char value)
{
	int err;
	volatile u32 word;
	volatile u32 *wordAddr = (u32 *) (addr & ~((u32) 0x3));

	mutexLock(flash_common.lock);
	_ob_unlock();
	_flash_clearFlags();

	if ((err = _flash_wait()) == 0) {
		word = *wordAddr;
		if (addr & (1 << 0)) {
			word &= 0x00ff00ff;
			word |= ((u32) value) << 8;
			word |= ((u32) ~value) << 24;
		}
		else {
			word &= 0xff00ff00;
			word |= ((u32) value);
			word |= ((u32) ~value) << 16;
		}


		*wordAddr = word;
		err = _flash_wait();
	}

	_ob_lock();
	mutexUnlock(flash_common.lock);

	return err;
}


static size_t ob_readData(u32 offset, char *buff, size_t size)
{
	unsigned int i;

	mutexLock(flash_common.lock);

	for (i = 0; i < size; ++i)
		buff[i] = *((volatile u8 *) offset + i);

	mutexUnlock(flash_common.lock);

	return i;
}


static size_t ob_writeData(u32 offset, const char *buff, size_t size)
{
	if (size != 1)
		return 0;

	ob_writeByte(offset, buff[0]);

	return 1;
}


static inline int program_isValidAddress(u32 addr)
{
	if (addr >= FLASH_PROGRAM_1_ADDR && addr <= (FLASH_PROGRAM_1_ADDR + FLASH_PROGRAM_SIZE))
		return 1;

	if (addr >= FLASH_PROGRAM_2_ADDR && addr <= (FLASH_PROGRAM_2_ADDR + FLASH_PROGRAM_SIZE))
		return 1;

	return 0;
}


static inline void _program_lock(void)
{
	*(flash_common.flash + flash_pecr) |= (1 << 1);
	dataBarier();
}


static void _program_unlock(void)
{

	while (*(flash_common.flash + flash_pecr) & (1 << 1)) {
		_eeprom_unlock();

		*(flash_common.flash + flash_prgkeyr) = 0x8c9daebf;
		dataBarier();
		*(flash_common.flash + flash_prgkeyr) = 0x13141516;
		dataBarier();
	}
}


static int _program_erasePage(u32 addr)
{
	int err;

	_program_unlock();
	_flash_clearFlags();

	if ((err = _flash_wait()) == 0) {
		/* Set the ERASE and PROG bits. */
		*(flash_common.flash + flash_pecr) |= ((1 << 9) | (1 << 3));

		/* Erase page. */
		*(volatile u32 *) addr = 0x0;
		err = _flash_wait();

		/* Disable the ERASE and PROG bits. */
		*(flash_common.flash + flash_pecr) &= ~((1 << 9) | (1 << 3));
	}

	_program_lock();

	return err;
}


static int _program_writeWord(u32 addr, u32 value)
{
	int err;
	volatile u32 *current = (volatile u32 *) addr;

	_program_unlock();
	_flash_clearFlags();

	if ((err = _flash_wait()) == 0)  {
		if (*current == value) {
			_program_lock();
			return 0;
		}

		*current = value;
		err = _flash_wait();
	}

	_program_lock();

	return err;
}


static size_t _program_readData(u32 offset, char *buff, size_t size)
{
	unsigned int i, j, n = 0;
	unsigned int prefixBytes = min(4 - (offset & 0x3), size);
	unsigned int suffixBytes = (size - prefixBytes) % 4;
	unsigned int middleBytes = size - prefixBytes - suffixBytes;
	volatile u32 *addr = (u32 *) (offset & ~((u32) 0x3));
	volatile u32 value;

	/* Read prefix word. */
	value = *addr++;
	for (i = 0; i < prefixBytes; ++i, ++n)
		buff[n] = value >> (i * 8);

	/* Read middle section. */
	for (i = 0; i < middleBytes; i += 4, n += 4) {
		value = *addr++;
		for (j = 0; j < 4; ++j)
			buff[n + j] = value >> (j * 8);
	}

	/* Read suffix word. */
	if (suffixBytes) {
		value = *addr;
		for (i = 0; i < suffixBytes; ++i, ++n)
			buff[n] = value >> (i * 8);
	}

	return n;
}


static size_t program_readData(u32 offset, char *buff, size_t size)
{
	size_t ret;

	mutexLock(flash_common.lock);
	ret = _program_readData(offset, buff, size);
	mutexUnlock(flash_common.lock);

	return ret;
}


static size_t program_writeData(u32 offset, const char *buff, size_t size)
{
	u32 word, pageAddr, addr = offset;
	size_t n = 0;
	int i, j, toSkip;

	mutexLock(flash_common.lock);

	while (n < size) {
		/* Read page into buffer and erase it. */
		pageAddr = addr & ~((u32) (FLASH_PAGE_SIZE - 1));
		_program_readData(pageAddr, flash_common.page, FLASH_PAGE_SIZE);
		_program_erasePage(pageAddr);

		/* Modify data in buffer. */
		toSkip = addr - pageAddr;
		for (i = toSkip; (i < FLASH_PAGE_SIZE) && n < size; ++i, ++n)
			flash_common.page[i] = buff[n];

		/* Write back page. */
		for (i = 0; i < FLASH_PAGE_SIZE; i += 4) {
			for (j = 0, word = 0; j < 4; ++j)
				word |= flash_common.page[i + j] << 8 * j;

			_program_writeWord(pageAddr + i, word);
		}

		addr += FLASH_PAGE_SIZE;
	}

	mutexUnlock(flash_common.lock);

	return n;
}


size_t flash_readData(u32 offset, char *buff, size_t size)
{
	if (program_isValidAddress(offset))
		return program_readData(offset, buff, size);

	if (eeprom_isValidAdress(offset))
		return eeprom_readData(offset, buff, size);

	if (ob_isValidAdress(offset))
		return ob_readData(offset, buff, size);

	return 0;
}


size_t flash_writeData(u32 offset, const char *buff, size_t size)
{
	if (program_isValidAddress(offset))
		return program_writeData(offset, buff, size);

	if (eeprom_isValidAdress(offset))
		return eeprom_writeData(offset, buff, size);

	if (ob_isValidAdress(offset))
		return ob_writeData(offset, buff, size);

	return 0;
}

#if 0
void flash_bankBreak(void)
{
	spinlock_t spinlock;
	char buff[4] = { 0x11, 0x11, 0x11, 0x11 };
	u32 currentBankAddress = flash_activeBank() ? FLASH_PROGRAM_2_ADDR : FLASH_PROGRAM_1_ADDR;

	/* Set option bytes so boot bank will be choosen on startup based on bank first word */
	ob_writeByte(0x1ff80004, 0x78);

	hal_spinlockCreate(&spinlock, "bankBreak");
	hal_spinlockSet(&spinlock);

	flash_writeData(currentBankAddress, buff, sizeof(buff));

	/* force reload of option bytes */
	ob_unlock();
	*(flash_common.flash + flash_pecr) |= (1 << 18);
	ob_lock();

	hal_cpuRestart();

	hal_spinlockClear(&spinlock);
	hal_spinlockDestroy(&spinlock);
}


static int _flash_atomCopy(u32 dest, u32 src, size_t len)
{
	hal_cpuDisableInterrupts();

	flash_writeData(dest, (void *)src, len);

	hal_cpuRestart();

	/* Should never reach here */
	return 0;
}
#endif

int flash_init(void)
{
	flash_common.flash = (void *) 0x40023c00;

	if (mutexCreate(&flash_common.lock) != EOK) {
		DEBUG("Failed to create mutex\n");
		return -ENOMEM;
	}

	_flash_clearFlags();

	return EOK;
}
