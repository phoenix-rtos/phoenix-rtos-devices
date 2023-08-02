/*
 * Phoenix-RTOS
 *
 * Common Flash Interface for flash driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHCFG_H_
#define _FLASHCFG_H_

#include <stdint.h>

/* Return timeouts in ms */
#define CFI_TIMEOUT_MAX_PROGRAM(typical, max) (((1 << typical) * (1 << max)) / 1000)
#define CFI_TIMEOUT_MAX_ERASE(typical, max)   ((1 << typical) * (1 << max))

/* Return size in bytes */
#define CFI_SIZE_FLASH(val)                (1 << val)
#define CFI_SIZE_SECTION(val)              (val * 0x100)
#define CFI_SIZE_PAGE(val)                 (1 << val)
#define CFI_SIZE_REGION(regSize, regCount) (CFI_SIZE_SECTION(regSize) * (regCount + 1))

#define CFI_DUMMY_CYCLES_NOT_SET 0xff

/* Order in command's table */
enum {
	flash_cmd_rdid = 0,
	flash_cmd_rdsr1,
	flash_cmd_wrdi,
	flash_cmd_wren,
	flash_cmd_read,
	flash_cmd_4read,
	flash_cmd_fast_read,
	flash_cmd_4fast_read,
	flash_cmd_dor,
	flash_cmd_4dor,
	flash_cmd_qor,
	flash_cmd_4qor,
	flash_cmd_dior,
	flash_cmd_4dior,
	flash_cmd_qior,
	flash_cmd_4qior,
	flash_cmd_pp,
	flash_cmd_4pp,
	flash_cmd_qpp,
	flash_cmd_4qpp,
	flash_cmd_p4e,
	flash_cmd_4p4e,
	flash_cmd_p64e,
	flash_cmd_4p64e,
	flash_cmd_be,
	flash_cmd_end
};


typedef struct {
	uint8_t byteWrite;
	uint8_t pageWrite;
	uint8_t sectorErase;
	uint8_t chipErase;
} __attribute__((packed)) flash_cfi_timeout_t;


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
	flash_cfi_timeout_t timeoutTypical;
	flash_cfi_timeout_t timeoutMax;
	uint8_t chipSize;
	uint16_t fdiDesc;
	uint16_t pageSize;
	uint8_t regsCount;
	struct {
		uint16_t count;
		uint16_t size;
	} __attribute__((packed)) regs[4];
} __attribute__((packed)) flash_cfi_t;


typedef struct {
	uint8_t opCode;
	uint8_t size;
	uint8_t dummyCyc;
	uint8_t dataLines;
} flash_cmd_t;


typedef struct flash_info {
	flash_cfi_t cfi;
	flash_cmd_t cmds[flash_cmd_end];

	int readCmd; /* Default read command define for specific flash memory */
	int ppCmd;   /* Default page program command define for specific flash memory */

	const char *name;
	/* clang-format off */
	enum { flash_3byteAddr, flash_4byteAddr } addrMode;
	/* clang-format on */

	int (*init)(const struct flash_info *info);
} flash_info_t;


extern void flashcfg_jedecIDGet(flash_cmd_t *cmd);


extern int flashcfg_infoResolve(flash_info_t *info);


#endif
