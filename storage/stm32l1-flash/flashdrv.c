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

#include <errno.h>
#include <unistd.h>

#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/pwman.h>

#include "log.h"
#include "flashdrv.h"
#include "flashdrv_priv.h"


#define BUFFER_SIZE         128
#define FLASH_MAX_READ      64


struct {
	volatile unsigned int *flash;
	volatile unsigned int *iwdg;
	unsigned int port;
	handle_t lock;

	char buff[FLASH_MAX_READ];
	char page[FLASH_PAGE_SIZE];
} flash_common;


enum { flash_pecr = 1, flash_pekeyr = 3, flash_prgkeyr, flash_oprkeyr, flash_sr, flash_obr };


static int flash_wait(void)
{
	int i;

	for (i = 0; i < 10; ++i) {
		/* Check if flash is busy. */
		if ((*(flash_common.flash + flash_sr) & 0x1) == 0x1) {
			keepidle(1);
			usleep(10 * 1000);
			keepidle(0);
			*flash_common.iwdg = 0xaaaa;
			continue;
		}

		/* Check if flash is write protected. */
		if (*(flash_common.flash + flash_sr) & (1 << 8))
			return 1;

		return (*(flash_common.flash + flash_sr) & 0x1e00);
	}

	return 1;
}


static inline void flash_clearFlags(void)
{
	*(flash_common.flash + flash_sr) |= 0x3f02;
}


static inline int eeprom_isValidAdress(u32 addr)
{
	return (addr >= FLASH_EEPROM_1_ADDR && addr <= (FLASH_EEPROM_2_ADDR + FLASH_EEPROM_SIZE));
}


static inline void eeprom_lock(void)
{
	*(flash_common.flash + flash_pecr) |= (1 << 0);
	__asm__ volatile ("dmb");
}


static void eeprom_unlock(void)
{
	while (*(flash_common.flash + flash_pecr) & (1 << 0)) {
		*(flash_common.flash + flash_pekeyr) = 0x89abcdef;
		__asm__ volatile ("dmb");
		*(flash_common.flash + flash_pekeyr) = 0x02030405;
		__asm__ volatile ("dmb");
	}
}


int eeprom_eraseByte(u32 addr)
{
	int err;

	mutexLock(flash_common.lock);
	eeprom_unlock();
	flash_clearFlags();

	if ((err = flash_wait()) == 0)
		*(volatile u8 *) addr = 0x0;

	eeprom_lock();
	mutexUnlock(flash_common.lock);
	return err;
}


static int eeprom_writeByte(u32 addr, char value)
{
	int err;

	mutexLock(flash_common.lock);
	eeprom_unlock();
	flash_clearFlags();

	if ((err = flash_wait()) == 0) {
		*(volatile u8 *) addr = value;
		err = flash_wait();
	}

	eeprom_lock();
	mutexUnlock(flash_common.lock);
	return err;
}


static size_t eeprom_readData(u32 offset, char *buff, size_t size)
{
	unsigned int i;

	for (i = 0; i < size; ++i)
		buff[i] = *((volatile u8 *) offset + i);

	return i;
}


static size_t eeprom_writeData(u32 offset, char *buff, size_t size)
{
	unsigned int i;

	for (i = 0; i < size; ++i) {
		eeprom_eraseByte(offset + i);
		eeprom_writeByte(offset + i, buff[i]);
	}
	return i;
}


static inline int ob_isValidAdress(u32 addr)
{
	/* Address must be even. */
	if (addr & (1 << 1))
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


static inline void ob_lock(void)
{
	*(flash_common.flash + flash_pecr) |= (1 << 2);
}


static void ob_unlock(void)
{
	while (*(flash_common.flash + flash_pecr) & (1 << 2)) {
		eeprom_unlock();

		*(flash_common.flash + flash_oprkeyr) = 0xfbead9c8;
		__asm__ volatile ("dmb");
		*(flash_common.flash + flash_oprkeyr) = 0x24252627;
		__asm__ volatile ("dmb");
	}
}


static int ob_writeByte(u32 addr, char value)
{
	int err;
	volatile u32 word;
	volatile u32 *wordAddr = (u32 *) (addr & ~((u32) 0x3));

	mutexLock(flash_common.lock);
	ob_unlock();
	flash_clearFlags();

	if ((err = flash_wait()) == 0) {
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
		err = flash_wait();
	}

	ob_lock();
	mutexUnlock(flash_common.lock);
	return err;
}


static size_t ob_readData(u32 offset, char *buff, size_t size)
{
	unsigned int i;

	for (i = 0; i < size; ++i)
		buff[i] = *((volatile u8 *) offset + i);

	return i;
}


static size_t ob_writeData(u32 offset, char *buff, size_t size)
{
	/* Writing OB is to compliaceted to do in chunks greater than a byte. */
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


static inline void program_lock(void)
{
	*(flash_common.flash + flash_pecr) |= (1 << 1);
	__asm__ volatile ("dmb");
}


static void program_unlock(void)
{

	while (*(flash_common.flash + flash_pecr) & (1 << 1)) {
		eeprom_unlock();

		*(flash_common.flash + flash_prgkeyr) = 0x8c9daebf;
		__asm__ volatile ("dmb");
		*(flash_common.flash + flash_prgkeyr) = 0x13141516;
		__asm__ volatile ("dmb");
	}
}


static int program_erasePage(u32 addr)
{
	int err;

	mutexLock(flash_common.lock);
	program_unlock();
	flash_clearFlags();

	if ((err = flash_wait()) == 0) {
		/* Set the ERASE and PROG bits. */
		*(flash_common.flash + flash_pecr) |= ((1 << 9) | (1 << 3));

		/* Erase page. */
		*(volatile u32 *) addr = 0x0;
		err = flash_wait();

		/* Disable the ERASE and PROG bits. */
		*(flash_common.flash + flash_pecr) &= ~((1 << 9) | (1 << 3));
	}

	program_lock();
	mutexUnlock(flash_common.lock);
	return err;
}


static int program_writeWord(u32 addr, u32 value)
{
	int err;
	volatile u32 *current = (volatile u32 *) addr;

	mutexLock(flash_common.lock);
	program_unlock();
	flash_clearFlags();

	if ((err = flash_wait()) == 0)  {
		if (*current == value) {
			program_lock();
			mutexUnlock(flash_common.lock);
			return 0;
		}

		*current = value;
		err = flash_wait();
	}

	program_lock();
	mutexUnlock(flash_common.lock);
	return err;
}


static size_t program_readData(u32 offset, char *buff, size_t size)
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


static size_t program_writeData(u32 offset, char *buff, size_t size)
{
	u32 word, pageAddr, addr = offset;
	size_t n = 0;
	int i, j, toSkip;

	while (n < size) {
		/* Read page into buffer and erase it. */
		pageAddr = addr & ~((u32) (FLASH_PAGE_SIZE - 1));
		program_readData(pageAddr, flash_common.page, FLASH_PAGE_SIZE);
		program_erasePage(pageAddr);

		/* Modify data in buffer. */
		toSkip = addr - pageAddr;
		for (i = toSkip; (i < FLASH_PAGE_SIZE) && n < size; ++i, ++n)
			flash_common.page[i] = buff[n];

		/* Write back page. */
		for (i = 0; i < FLASH_PAGE_SIZE; i += 4) {
			for (j = 0, word = 0; j < 4; ++j)
				word |= flash_common.page[i + j] << 8 * j;

			program_writeWord(pageAddr + i, word);
		}

		addr += FLASH_PAGE_SIZE;
	}

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


size_t flash_writeData(u32 offset, char *buff, size_t size)
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
	char *buff;

	if ((buff = vm_kmalloc(FLASH_MAX_READ)) == NULL)
		return -ENOMEM;

	hal_cpuDisableInterrupts();

	flash_writeData(dest, (void *)src, len);

	hal_cpuRestart();

	/* Should never reach here */
	return 0;
}
#endif

static void flash_thread(void *arg)
{
	char msg[BUFFER_SIZE];
	msghdr_t hdr;
	msgdata_t *data;
	flashdevctl_t *devctl;
	flashevent_t event;
	flashlog_t log;
	flasheeprominfo_t info;
	int count, bank;

	eeprom_init();

	for (;;) {
		int size = recv(flash_common.port, msg, sizeof(msg), &hdr);
		if (hdr.type == NOTIFY)
			continue;

		switch (hdr.op) {
		case READ:
			if (size != sizeof(msgdata_t)) {
				respond(flash_common.port, EINVAL, NULL, 0);
				break;
			}

			data = (msgdata_t *) msg;
			size = min(FLASH_MAX_READ, data->size);

			count = flash_readData(data->offset, flash_common.buff, size);
			respond(flash_common.port, EOK, flash_common.buff, count);
			break;

		case WRITE:
			if (size != sizeof(msgdata_t)) {
				respond(flash_common.port, EINVAL, NULL, 0);
				break;
			}

			data = (msgdata_t *) msg;
			flash_writeData(data->offset, data->buff, data->size);
			respond(flash_common.port, EOK, NULL, 0);
			break;

		case DEVCTL:
			if (size != sizeof(flashdevctl_t)) {
				respond(flash_common.port, EINVAL, NULL, 0);
				break;
			}

			devctl = (flashdevctl_t *) msg;
			switch (devctl->type) {
			case FLASH_EVENT_GET:
				event_read(devctl, &event);
				respond(flash_common.port, EOK, &event, sizeof(event));
				break;

			case FLASH_EVENT_SET:
				event_write(devctl);
				respond(flash_common.port, EOK, NULL, 0);
				break;

			case FLASH_LOG_GET:
				log_read(&log);
				respond(flash_common.port, EOK, &log, sizeof(log));
				break;

			case FLASH_LOG_SET:
				log_write(devctl);
				respond(flash_common.port, EOK, NULL, 0);
				break;

			case FLASH_EEPROM_INFO_GET:
				info.addr = eeprom_freeAddr();
				info.size = eeprom_freeSize();
				respond(flash_common.port, EOK, &info, sizeof(info));
				break;

			case FLASH_BANK_GET:
				bank = flash_activeBank();
				respond(flash_common.port, EOK, &bank, sizeof(bank));
				break;
			case FLASH_BANK_BREAK:
				/* flash_bankBreak(); */
				respond(flash_common.port, EINVAL, NULL, 0);
				break;

			case FLASH_ATOM_COPY:
				/* res = _flash_atomCopy(devctl->atomcpy.dest, devctl->atomcpy.src, devctl->atomcpy.len); */
				respond(flash_common.port, EINVAL, NULL, 0);
				break;
			}

			break;

		default:
			respond(flash_common.port, EINVAL, NULL, 0);
			break;
		}
	}
}


void main(void)
{
	flash_common.flash = (void *) 0x40023c00;
	flash_common.iwdg = (void *)0x40003000;

	mutexCreate(&flash_common.lock);

	flash_clearFlags();

	portCreate(&flash_common.port);
	portRegister(flash_common.port, "/flashdrv");

	flash_thread(NULL);
}
