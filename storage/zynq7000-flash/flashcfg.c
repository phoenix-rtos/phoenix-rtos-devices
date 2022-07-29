/*
 * Phoenix-RTOS
 *
 * Common Flash Interface for flash drive
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "flashcfg.h"

#include <errno.h>
#include <string.h>


/* Generic flash commands */

#define FLASH_CMD_RDID       0x9f /* Read JEDEC ID */
#define FLASH_CMD_RDSR1      0x05 /* Read Status Register - 1 */
#define FLASH_CMD_WRDI       0x04 /* Write enable */
#define FLASH_CMD_WREN       0x06 /* Write disable */
#define FLASH_CMD_READ       0x03 /* Read data */
#define FLASH_CMD_4READ      0x13 /* 4-byte Read data */
#define FLASH_CMD_FAST_READ  0x0b /* Fast read data */
#define FLASH_CMD_4FAST_READ 0x0c /* 4-byte Fast read data */
#define FLASH_CMD_DOR        0x3b /* Dual output fast read data */
#define FLASH_CMD_4DOR       0x3c /* 4-byte Dual output fast read data */
#define FLASH_CMD_QOR        0x6b /* Quad output read data */
#define FLASH_CMD_4QOR       0x6c /* 4-byte Quad output read data */
#define FLASH_CMD_DIOR       0xbb /* Dual input/output fast read data */
#define FLASH_CMD_4DIOR      0xbc /* 4-byte Dual input/output fast read data */
#define FLASH_CMD_QIOR       0xeb /* Quad input/output fast read data */
#define FLASH_CMD_4QIOR      0xec /* 4-byte Quad input/output fast read data */
#define FLASH_CMD_PP         0x02 /* Page program */
#define FLASH_CMD_4PP        0x12 /* 4-byte Page program */
#define FLASH_CMD_QPP        0x32 /* Quad input fast program */
#define FLASH_CMD_4QPP       0x34 /* 4-byte Quad input fast program */
#define FLASH_CMD_P4E        0x20 /* 4KB sector erase */
#define FLASH_CMD_4P4E       0x21 /* 4-byte 4KB sector erase */
#define FLASH_CMD_SE         0xd8 /* 64KB sector erase*/
#define FLASH_CMD_4SE        0xdc /* 4-byte 64KB sector erase*/
#define FLASH_CMD_BE         0x60 /* Chip erase */


static const flash_cmd_t defCmds[flash_cmd_end] = {
	{ FLASH_CMD_RDID, 1, 24 },
	{ FLASH_CMD_RDSR1, 1, 8 },
	{ FLASH_CMD_WRDI, 1, 0 },
	{ FLASH_CMD_WREN, 1, 0 },
	{ FLASH_CMD_READ, 4, 0 },
	{ FLASH_CMD_4READ, 5, 0 },
	{ FLASH_CMD_FAST_READ, 4, 8 },
	{ FLASH_CMD_4FAST_READ, 5, 8 },
	{ FLASH_CMD_DOR, 4, 8 },
	{ FLASH_CMD_4DOR, 5, 16 },
	{ FLASH_CMD_QOR, 4, 8 },
	{ FLASH_CMD_4QOR, 5, 8 },
	{ FLASH_CMD_DIOR, 4, 8 },
	{ FLASH_CMD_4DIOR, 5, 8 },
	{ FLASH_CMD_QIOR, 4, 24 },
	{ FLASH_CMD_4QIOR, 5, 24 },
	{ FLASH_CMD_PP, 4, 0 },
	{ FLASH_CMD_4PP, 5, 0 },
	{ FLASH_CMD_QPP, 4, 0 },
	{ FLASH_CMD_4QPP, 5, 0 },
	{ FLASH_CMD_P4E, 4, 0 },
	{ FLASH_CMD_4P4E, 5, 0 },
	{ FLASH_CMD_SE, 4, 0 },
	{ FLASH_CMD_4SE, 5, 0 },
	{ FLASH_CMD_BE, 1, 0 }
};


static void flashcfg_spansion(flash_info_t *info)
{
	info->name = "Spansion s25fl256s1";
	memcpy(info->cmds, defCmds, sizeof(defCmds));

	/* FIXME: s25fl256s1 should be able to operate in quad and dual mode for 4byte transactions.
	          In the current implementation errors occur for the dual and the quad. */
	info->readCmd = flash_cmd_4fast_read;
	info->ppCmd = flash_cmd_4pp;
}


static void flashcfg_micron(flash_info_t *info)
{
	info->name = "Micron n25q128";

	/* Lack of CFI support, filled based on specification */
	info->cfi.timeoutTypical.byteWrite = 0x6;
	info->cfi.timeoutTypical.pageWrite = 0x9;
	info->cfi.timeoutTypical.sectorErase = 0x8;
	info->cfi.timeoutTypical.chipErase = 0xf;
	info->cfi.timeoutMax.byteWrite = 0x2;
	info->cfi.timeoutMax.pageWrite = 0x2;
	info->cfi.timeoutMax.sectorErase = 0x3;
	info->cfi.timeoutMax.chipErase = 0x3;
	info->cfi.chipSize = 0x18;
	info->cfi.fdiDesc = 0x0102;
	info->cfi.pageSize = 0x08;
	info->cfi.regsCount = 1;
	info->cfi.regs[0].count = 0xff;
	info->cfi.regs[0].size = 0x100;

	memcpy(info->cmds, defCmds, sizeof(defCmds));

	/* Specific configuration */
	info->cmds[flash_cmd_dior].dummyCyc = 16;
	info->cmds[flash_cmd_qior].dummyCyc = 32;

	/* Default read and page program commands */
	info->readCmd = flash_cmd_qior;
	info->ppCmd = flash_cmd_pp;
}


static void flashcfg_winbond(flash_info_t *info)
{
	info->name = "Winbond W25Q128JV";

	/* Lack of CFI support, filled based on specification */
	info->cfi.timeoutTypical.byteWrite = 0x6;
	info->cfi.timeoutTypical.pageWrite = 0x9;
	info->cfi.timeoutTypical.sectorErase = 0x8;
	info->cfi.timeoutTypical.chipErase = 0xf;
	info->cfi.timeoutMax.byteWrite = 0x2;
	info->cfi.timeoutMax.pageWrite = 0x2;
	info->cfi.timeoutMax.sectorErase = 0x3;
	info->cfi.timeoutMax.chipErase = 0x3;
	info->cfi.chipSize = 0x18;
	info->cfi.fdiDesc = 0x0102;
	info->cfi.pageSize = 0x08;
	info->cfi.regsCount = 1;
	info->cfi.regs[0].count = 0xff;
	info->cfi.regs[0].size = 0x100;

	memcpy(info->cmds, defCmds, sizeof(defCmds));

	info->readCmd = flash_cmd_qior;
	info->ppCmd = flash_cmd_pp;
}


void flashcfg_jedecIDGet(flash_cmd_t *cmd)
{
	memcpy(cmd, &defCmds[flash_cmd_rdid], sizeof(flash_cmd_t));
}


int flashcfg_infoResolve(flash_info_t *info)
{
	int res = EOK;

	/* Spansion s25fl256s1 */
	if (info->cfi.vendorData[0] == 0x1 && info->cfi.vendorData[2] == 0x19) {
		flashcfg_spansion(info);
	}
	/* Micron n25q128 */
	else if (info->cfi.vendorData[0] == 0x20 && info->cfi.vendorData[1] == 0xbb && info->cfi.vendorData[2] == 0x18) {
		flashcfg_micron(info);
	}
	/* Winbond W25Q128JV - IM/JM */
	else if (info->cfi.vendorData[0] == 0xef && info->cfi.vendorData[1] == 0x40 && info->cfi.vendorData[2] == 0x18) {
		flashcfg_winbond(info);
	}
	else {
		info->name = "Unknown";
		res = -EINVAL;
	}

	info->addrMode = (info->cfi.chipSize > 24) ? flash_4byteAddr : flash_3byteAddr;

	return res;
}
