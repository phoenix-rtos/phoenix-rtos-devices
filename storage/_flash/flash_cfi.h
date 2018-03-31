/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * FLASH memory driver
 *
 * Copyright 2012-2013 Phoenix Systems
 * Copyright 2014 Phoenix Systems
 *
 * Author: Jacek Popko, Katarzyna Baranowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASH_CFI_H_
#define _FLASH_CFI_H_

#include <main/if.h>

typedef struct {
	u8 byteWrite;
	u8 bufferWrite;
	u8 sectorErase;
	u8 chipErase;
} __attribute__((packed)) flash_cfi_timeout_t;


typedef struct {
	u8  vendorSpecific[0x10];
	u8  qry[3];
	u16 cmdSet1;
	u16 addrExt1;
	u16 cmdSet2;
	u16 addrExt2;
	struct {
		u8 vccMin;
		u8 vccMax;
		u8 vppMin;
		u8 vppMax;
	} __attribute__((packed)) voltages;
	flash_cfi_timeout_t timeoutTypical;
	flash_cfi_timeout_t timeoutMax;
	u8  chipSize;
	u16 fdiDesc;
	u16 bufferSize;
	u8  regionsCount;
	struct {
		u16 blockCount;
		u16 blockSize;
	} __attribute__((packed)) region[4];
} __attribute__((packed)) flash_cfi_t;

#endif