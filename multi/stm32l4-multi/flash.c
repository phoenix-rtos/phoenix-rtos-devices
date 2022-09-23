/*
 * Phoenix-RTOS
 *
 * STM32L4 internal flash driver
 *
 * Copyright 2017, 2018, 2022 Phoenix Systems
 * Author: Aleksander Kaminski, Jakub Sejdak, Tomasz Korniluk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/threads.h>
#include <sys/pwman.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/interrupt.h>

#include "flash.h"
#include "common.h"


enum { flash_acr = 0, flash_pdkeyr, flash_keyr, flash_optkeyr, flash_sr, flash_cr, flash_eccr,
	flash_optr = flash_eccr + 2, flash_pcrop1sr, flash_pcrop1er, flash_wrp1ar, flash_wrp1br,
	flash_pcrop2sr = flash_wrp1br + 5, flash_pcrop2er, flash_wrp2ar, flash_wrp2br };


struct {
	volatile unsigned int *flash;
	volatile int operr;
	unsigned int bankflip;
	unsigned int activebank;
	handle_t lock;
	handle_t irqcond;
	handle_t irqlock;
	handle_t irqh;

	char page[FLASH_PAGE_SIZE];
} flash_common;


static int _flash_intHandler(unsigned int n, void *arg)
{
	if (!(*(flash_common.flash + flash_sr) & 3))
		return -1;

	if (*(flash_common.flash + flash_sr) & 1)
		flash_common.operr = 0;
	else if (*(flash_common.flash + flash_sr) & 2)
		flash_common.operr = -1;

	*(flash_common.flash + flash_sr) |= 0x3;

	return 1;
}


static int _flash_wait(void)
{

	mutexLock(flash_common.irqlock);
	while (*(flash_common.flash + flash_sr) & (1 << 16))
		condWait(flash_common.irqcond, flash_common.irqlock, 0);
	mutexUnlock(flash_common.irqlock);

	return flash_common.operr;
}


static inline void _flash_clearFlags(void)
{
	*(flash_common.flash + flash_sr) |= 0xc3fb;
	dataBarier();
	flash_common.operr = 0;
}


static inline void _program_lock(void)
{
	*(flash_common.flash + flash_cr) |= 1ul << 31;
	dataBarier();
	keepidle(0);
}


static void _program_unlock(void)
{
	/* We'll get hardfault if we do this wrong... */

	keepidle(1);
	*(flash_common.flash + flash_keyr) = 0x45670123;
	dataBarier();
	*(flash_common.flash + flash_keyr) = 0xcdef89ab;
	dataBarier();
}


static inline int program_isValidAddress(uint32_t addr, size_t size)
{
	if (addr >= FLASH_PROGRAM_1_ADDR && addr + size <= (FLASH_PROGRAM_1_ADDR + FLASH_PROGRAM_BANK_SIZE))
		return 1;

	if (addr >= FLASH_PROGRAM_2_ADDR && addr + size <= (FLASH_PROGRAM_2_ADDR + FLASH_PROGRAM_BANK_SIZE))
		return 1;

	return 0;
}


static inline int otp_isValidAddress(uint32_t addr, size_t size)
{
	if ((addr >= OTP_ADDR) && (addr + size < (OTP_ADDR + OTP_SIZE)) && (addr % (2 * sizeof(uint32_t)) == 0) &&
			size % (2 * sizeof(uint32_t)) == 0) {
		return 1;
	}

	return 0;
}


static size_t _program_readData(uint32_t offset, char *buff, size_t size)
{
	memcpy(buff, (void *)offset, size);

	return size;
}


size_t flash_readData(uint32_t offset, char *buff, size_t size)
{
	size_t ret;

	if (!program_isValidAddress(offset, size))
		return 0;

	mutexLock(flash_common.lock);
	ret = _program_readData(offset, buff, size);
	mutexUnlock(flash_common.lock);

	return ret;
}


ssize_t flash_readOtp(uint32_t offset, char *buff, size_t size)
{
	size_t ret;

	if (otp_isValidAddress(offset + OTP_ADDR, size) == 0) {
		return -EINVAL;
	}

	mutexLock(flash_common.lock);
	ret = _program_readData(offset + OTP_ADDR, buff, size);
	mutexUnlock(flash_common.lock);

	return ret;
}


static int _program_erasePage(uint32_t addr)
{
	int err;
	unsigned int taddr, t, bank, page;

	bank = (addr < FLASH_PROGRAM_2_ADDR) ? 0 : 1;
	taddr = addr - ((bank == 0) ? FLASH_PROGRAM_1_ADDR : FLASH_PROGRAM_2_ADDR);
	page = taddr / FLASH_PAGE_SIZE;

	if ((err = _flash_wait()) < 0)
		return err;

	_flash_clearFlags();

	t = *(flash_common.flash + flash_cr) & ~((1 << 11) | (0xff << 3) | 1);
	*(flash_common.flash + flash_cr) = t | ((bank ^ flash_common.bankflip) << 11) | (page << 3) | (1 << 1);
	dataBarier();
	*(flash_common.flash + flash_cr) |= 1 << 16;

	err = _flash_wait();
	_flash_clearFlags();

	*(flash_common.flash + flash_cr) &= ~(1 << 1);

	return err;
}


static int _program_dblWords(volatile uint32_t *addr, const char *buff, size_t size)
{
	int pos = 0;
	uint32_t value[2];

	for (pos = 0; pos < size; pos += 2 * sizeof(uint32_t)) {
		*(flash_common.flash + flash_cr) |= 1;
		memcpy(value, buff + pos, 2 * sizeof(uint32_t));
		*(addr++) = value[0];
		dataBarier();
		*(addr++) = value[1];
		dataBarier();

		if (_flash_wait() != 0) {
			break;
		}

		_flash_clearFlags();
	}

	*(flash_common.flash + flash_cr) &= ~1;

	_flash_clearFlags();

	return pos >= size ? size : pos;
}


static int _program_writePage(uint32_t offset)
{
	int ret;

	if (offset & (FLASH_PAGE_SIZE - 1)) {
		return -1;
	}

	if (_flash_wait() != 0) {
		return -1;
	}

	_flash_clearFlags();

	ret = _program_dblWords((void *)offset, flash_common.page, FLASH_PAGE_SIZE);
	if (ret < FLASH_PAGE_SIZE) {
		return -1;
	}

	return 0;
}


size_t flash_writeData(uint32_t offset, const char *buff, size_t size)
{
	size_t towrite = size, chunk;
	uint32_t coffset = offset, cpage, missalign;

	if (!program_isValidAddress(offset, size))
		return 0;

	mutexLock(flash_common.lock);
	_program_unlock();
	_flash_clearFlags();

	while (towrite) {
		cpage = coffset & ~(FLASH_PAGE_SIZE - 1);
		missalign = coffset - cpage;
		chunk = (towrite > FLASH_PAGE_SIZE - missalign) ? FLASH_PAGE_SIZE - missalign : towrite;

		if (chunk != FLASH_PAGE_SIZE) {
			if (_program_readData(cpage, flash_common.page, FLASH_PAGE_SIZE) != FLASH_PAGE_SIZE)
				break;
		}
		if (_program_erasePage(cpage) < 0)
			break;
		memcpy(flash_common.page + missalign, buff + size - towrite, chunk);
		if (_program_writePage(cpage) < 0)
			break;
		towrite -= chunk;
		coffset += chunk;
	}

	_program_lock();
	mutexUnlock(flash_common.lock);

	return size - towrite;
}


ssize_t flash_writeOtp(uint32_t offset, const char *buff, size_t size)
{
	int ret;
	uint32_t *ptr = (void *)(offset + OTP_ADDR);

	if (otp_isValidAddress((uint32_t)ptr, size) == 0) {
		return -EINVAL;
	}

	mutexLock(flash_common.lock);
	_program_unlock();
	_flash_clearFlags();

	ret = _flash_wait();
	if (ret == 0) {
		ret = _program_dblWords(ptr, buff, size);
	}

	_program_lock();
	mutexUnlock(flash_common.lock);

	return ret;
}


static unsigned int _flash_getOptions(void)
{
	return *(flash_common.flash + flash_optr);
}


void flash_getInfo(flashinfo_t *info)
{
	info->dualbank = !!(_flash_getOptions() & (1 << 21));
	info->dualboot = !!(_flash_getOptions() & (1 << 20));
	info->remap = flash_common.bankflip;
	info->activebank = flash_common.activebank;
}


ssize_t flash_writeRaw(uint32_t offset, const char *buff, size_t size)
{
	int ret;

	if (program_isValidAddress(offset, size) == 0)
		return 0;

	mutexLock(flash_common.lock);
	_program_unlock();

	ret = _flash_wait();
	if (ret == 0) {
		_flash_clearFlags();
		ret = _program_dblWords((void *)offset, buff, size);
	}

	_program_lock();
	mutexUnlock(flash_common.lock);

	return ret;
}


ssize_t flash_erasePage(uint32_t offset)
{
	int ret;

	if (program_isValidAddress(offset, FLASH_PAGE_SIZE) == 0)
		return 0;

	mutexLock(flash_common.lock);
	_program_unlock();

	ret = _flash_wait();
	if (ret == 0) {
		_flash_clearFlags();
		ret = _program_erasePage(offset);
	}

	_program_lock();
	mutexUnlock(flash_common.lock);

	return ret;
}


int flash_init(void)
{
	volatile unsigned int *syscfg = (void *)0x40010000;

	flash_common.flash = (void *)0x40022000;

	/* Check what flash bank is mapped at FLASH_PROGRAM_1_ADDR */
	flash_common.bankflip = ((*syscfg & (1 << 8)) != 0) ? 1 : 0;
	flash_common.activebank = flash_activeBank() ^ flash_common.bankflip;

	mutexCreate(&flash_common.lock);
	mutexCreate(&flash_common.irqlock);
	condCreate(&flash_common.irqcond);
	interrupt(flash_irq, _flash_intHandler, NULL, flash_common.irqcond, &flash_common.irqh);

	_flash_clearFlags();

	/* Enable EOP and error interrupts */
	_program_unlock();
	*(flash_common.flash + flash_cr) |= (1 << 25) | (1 << 24);
	_program_lock();

	return EOK;
}
