/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * CFI
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _CFI_H_
#define _CFI_H_


#include <sys/types.h>

/* Timeouts in us */
#define CFI_TIMEOUT_MAX_PROGRAM(typical, maximum) ((1u << typical) * (1u << maximum) * 2)
#define CFI_TIMEOUT_MAX_ERASE(typical, maximum)   ((1u << typical) * (1u << maximum) * 1024u * 2)

#define CFI_SIZE(size) (1u << ((uint32_t)size))


typedef struct {
	uint8_t wordProgram;
	uint8_t bufWrite;
	uint8_t blkErase;
	uint8_t chipErase;
} __attribute__((packed)) cfi_timeout_t;


typedef struct {
	uint8_t vendorData[0x10];
	uint8_t qry[3];
	uint16_t cmdSet1;
	uint16_t addrExt1;
	uint16_t cmdSet2;
	uint16_t addrExt2;
	struct {
		uint8_t vccMin;
		uint8_t vccMax;
		uint8_t vppMin;
		uint8_t vppMax;
	} __attribute__((packed)) voltages;
	cfi_timeout_t toutTypical;
	cfi_timeout_t toutMax;
	uint8_t chipSz;
	uint16_t fdiDesc;
	uint16_t bufSz;
	uint8_t regionCnt;
	struct {
		uint16_t count;
		uint16_t size;
	} __attribute__((packed)) regions[4];
} __attribute__((packed)) cfi_info_t;


#endif
