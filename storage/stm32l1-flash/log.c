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

#include <string.h>
#include <stdio.h>

#include "log.h"


#define FLASH_AREA_EVENTS   0
#define FLASH_AREA_LOGS     1


enum { EVENTS = 0, LOGS };


typedef struct {
	u32 addr;
	int currIdx;
	u64 currId;
	int entrySize;
	int entryCount;
} __attribute__((packed)) logarea_t;


struct {
	logarea_t area[2];
} log_common;


void event_read(flashdevctl_t *devctl, flashevent_t *event)
{
	int currIdx = log_common.area[EVENTS].currIdx;
	int diffIdx = (currIdx >= devctl->idx) ? (currIdx - devctl->idx) : (log_common.area[EVENTS].entryCount - (devctl->idx - currIdx));

	u32 addr = log_common.area[EVENTS].addr + diffIdx * log_common.area[EVENTS].entrySize;

	flash_readData(addr, (char *)event, log_common.area[EVENTS].entrySize);
}


void event_write(flashdevctl_t *devctl)
{
	u32 addr;

	log_common.area[EVENTS].currIdx += 1;
	log_common.area[EVENTS].currIdx %= log_common.area[EVENTS].entryCount;
	addr = log_common.area[EVENTS].addr + log_common.area[EVENTS].currIdx * log_common.area[EVENTS].entrySize;
	devctl->event.id = ++log_common.area[EVENTS].currId;

	flash_writeData(addr, (char *)(&devctl->event), log_common.area[EVENTS].entrySize);
}


void log_read(flashlog_t *log)
{
	int currIdx = log_common.area[LOGS].currIdx;
	u32 addr = log_common.area[LOGS].addr + currIdx * log_common.area[LOGS].entrySize;

	flash_readData(addr, (char *)log, log_common.area[LOGS].entrySize);
}


void log_write(flashdevctl_t *devctl)
{
	u32 addr;

	log_common.area[LOGS].currIdx += 1;
	log_common.area[LOGS].currIdx %= log_common.area[LOGS].entryCount;
	addr = log_common.area[LOGS].addr + log_common.area[LOGS].currIdx * log_common.area[LOGS].entrySize;
	devctl->event.id = ++log_common.area[LOGS].currId;

	flash_writeData(addr, (char *)(&devctl->event), log_common.area[LOGS].entrySize);
}


u32 eeprom_freeAddr(void)
{
	if (log_common.area[0].addr > log_common.area[1].addr)
		return log_common.area[0].addr + sizeof(u32) + log_common.area[0].entrySize * log_common.area[0].entryCount;
	else
		return log_common.area[1].addr + sizeof(u32) + log_common.area[1].entrySize * log_common.area[1].entryCount;
}


size_t eeprom_freeSize(void)
{
	size_t size = FLASH_EEPROM_SIZE;

	size -= 2 * sizeof(u32);
	size -= log_common.area[0].entryCount * log_common.area[0].entrySize;
	size -= log_common.area[1].entryCount * log_common.area[1].entrySize;

	return size;
}


void eeprom_init(void)
{
	u32 base;
	char buff[sizeof(u32)];
	char magic[] = { 0x12, 0x34, 0xab, 0xcd };
	int i, j;
	u64 id;

	/* Use eeprom from diffrent bank than flash we're executing from */
	if (!flash_activeBank())
		base = FLASH_EEPROM_2_ADDR;
	else
		base = FLASH_EEPROM_1_ADDR;

	log_common.area[EVENTS].addr = base;
	log_common.area[EVENTS].entrySize = sizeof(flashevent_t);
	log_common.area[EVENTS].entryCount = FLASH_EVENT_COUNT;
	log_common.area[LOGS].addr = base + sizeof(magic) + log_common.area[EVENTS].entrySize * log_common.area[EVENTS].entryCount;
	log_common.area[LOGS].entrySize = sizeof(flashlog_t);
	log_common.area[LOGS].entryCount = FLASH_LOG_COUNT;

	for (i = 0; i < 2; ++i) {
		/* Clear area if uninitialized. */
		flash_readData(log_common.area[i].addr + (log_common.area[i].entryCount * log_common.area[i].entrySize), buff, sizeof(buff));

		if (memcmp(buff, magic, sizeof(magic)) != 0) {
			printf("flash: Log area magic mismatch [%d], erasing...\n", i);

			for (j = 0; j < (log_common.area[i].entryCount * log_common.area[i].entrySize) + sizeof(magic); ++j)
				eeprom_eraseByte(log_common.area[i].addr + j);

			flash_writeData(log_common.area[i].addr + (log_common.area[i].entryCount * log_common.area[i].entrySize), magic, sizeof(magic));

			log_common.area[i].currId = 0;
			log_common.area[i].currIdx = 0;
			continue;
		}

		/* Find last log. */
		log_common.area[i].currId = 0;
		for (j = log_common.area[i].entryCount - 1; j >= 0; --j) {
			id = *((u64 *) (log_common.area[i].addr + (j * log_common.area[i].entrySize)));
			if (id >= log_common.area[i].currId) {
				log_common.area[i].currId = id;
				log_common.area[i].currIdx = j;
			}
		}
	}
}
