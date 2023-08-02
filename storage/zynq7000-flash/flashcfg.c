/*
 * Phoenix-RTOS
 *
 * Common Flash Interface for flash drive
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Hubert Buczynski, Malgorzata Wrobel
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "flashcfg.h"
#include "qspi.h"

#include <errno.h>
#include <string.h>
#include <sys/time.h>


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

#define FLASH_TIMEOUT_CMD_MS 1000
#define FLASH_TIMEOUT_WIP_MS 1000


static const flash_cmd_t flash_defCmds[flash_cmd_end] = {
	{ FLASH_CMD_RDID, 1, 24, 1 },
	{ FLASH_CMD_RDSR1, 1, 8, 1 },
	{ FLASH_CMD_WRDI, 1, 0, 1 },
	{ FLASH_CMD_WREN, 1, 0, 1 },
	{ FLASH_CMD_READ, 4, 0, 1 },
	{ FLASH_CMD_4READ, 5, 0, 1 },
	{ FLASH_CMD_FAST_READ, 4, CFI_DUMMY_CYCLES_NOT_SET, 1 },
	{ FLASH_CMD_4FAST_READ, 5, CFI_DUMMY_CYCLES_NOT_SET, 1 },
	{ FLASH_CMD_DOR, 4, CFI_DUMMY_CYCLES_NOT_SET, 2 },
	{ FLASH_CMD_4DOR, 5, CFI_DUMMY_CYCLES_NOT_SET, 2 },
	{ FLASH_CMD_QOR, 4, CFI_DUMMY_CYCLES_NOT_SET, 4 },
	{ FLASH_CMD_4QOR, 5, CFI_DUMMY_CYCLES_NOT_SET, 4 },
	{ FLASH_CMD_DIOR, 4, CFI_DUMMY_CYCLES_NOT_SET, 2 },
	{ FLASH_CMD_4DIOR, 5, CFI_DUMMY_CYCLES_NOT_SET, 2 },
	{ FLASH_CMD_QIOR, 4, CFI_DUMMY_CYCLES_NOT_SET, 4 },
	{ FLASH_CMD_4QIOR, 5, CFI_DUMMY_CYCLES_NOT_SET, 4 },
	{ FLASH_CMD_PP, 4, 0, 1 },
	{ FLASH_CMD_4PP, 5, 0, 1 },
	{ FLASH_CMD_QPP, 4, 0, 4 },
	{ FLASH_CMD_4QPP, 5, 0, 4 },
	{ FLASH_CMD_P4E, 4, 0, 1 },
	{ FLASH_CMD_4P4E, 5, 0, 1 },
	{ FLASH_CMD_SE, 4, 0, 1 },
	{ FLASH_CMD_4SE, 5, 0, 1 },
	{ FLASH_CMD_BE, 1, 0, 1 }
};


static int flashcfg_wren(const flash_info_t *info)
{
	int res;
	uint8_t cmdOpCode = info->cmds[flash_cmd_wren].opCode;

	qspi_start();
	res = qspi_transfer(&cmdOpCode, NULL, 1, FLASH_TIMEOUT_CMD_MS);
	qspi_stop();

	return res;
}


static int flashcfg_statusRegGet(const flash_info_t *info, uint8_t *val)
{
	int res;
	uint8_t rx[2] = { 0, 0 };
	uint8_t tx[2] = { 0, 0 };

	tx[0] = info->cmds[flash_cmd_rdsr1].opCode;

	qspi_start();
	res = qspi_transfer(tx, rx, 2, FLASH_TIMEOUT_CMD_MS);
	qspi_stop();

	*val = rx[1];

	return res;
}


static inline time_t flashcfg_timeMsGet(void)
{
	time_t now;
	gettime(&now, NULL);

	return now / 1000;
}


static int flashcfg_wipCheck(const flash_info_t *info, time_t timeout)
{
	int res;
	uint8_t st;
	const time_t stop = flashcfg_timeMsGet() + timeout;

	do {
		res = flashcfg_statusRegGet(info, &st);
		if (res < 0) {
			return res;
		}

		if (flashcfg_timeMsGet() >= stop) {
			return -ETIME;
		}
	} while ((st & 0x1) != 0);

	return EOK;
}


static int flashcfg_spansionInit(const flash_info_t *info)
{
	int res;
	uint8_t tx[3];

	res = flashcfg_wren(info);
	if (res < 0) {
		return res;
	}

	/* Write registers */

	/* Write registers cmd code */
	tx[0] = 0x1;

	/* SR */
	/* Leave SR in default state */
	tx[1] = 0;

	/* CR */
	/* Set latency code to 10b to support 100 MHz */
	/* Other values are set as in default config (0) */
	tx[2] = (2 << 6);
	/* Enable quad mode if needed. */
	if ((info->cmds[info->readCmd].dataLines == 4) || (info->cmds[info->ppCmd].dataLines == 4)) {
		tx[2] = (1 << 1);
	}

	qspi_start();
	res = qspi_transfer(tx, NULL, 3, FLASH_TIMEOUT_CMD_MS);
	qspi_stop();

	if (res < 0) {
		return res;
	}

	return flashcfg_wipCheck(info, FLASH_TIMEOUT_WIP_MS);
}


static void flashcfg_spansion(flash_info_t *info)
{
	info->name = "Spansion s25fl256s1";
	memcpy(info->cmds, flash_defCmds, sizeof(flash_defCmds));

	/* Specific configuration */
	/* Dummy cycles chosen for 100MHz clock speed. Latency Code 10b. */
	/* Dummy cycles * data lines must be divisible by 8. */

	/* Single line */
	info->cmds[flash_cmd_fast_read].dummyCyc = 8;
	info->cmds[flash_cmd_4fast_read].dummyCyc = 8;
	/* Two lines */
	info->cmds[flash_cmd_dor].dummyCyc = 8;
	info->cmds[flash_cmd_4dor].dummyCyc = 8;
	info->cmds[flash_cmd_dior].dummyCyc = 6;
	info->cmds[flash_cmd_4dior].dummyCyc = 6;
	/* Four lines */
	info->cmds[flash_cmd_qor].dummyCyc = 8;
	info->cmds[flash_cmd_4qor].dummyCyc = 8;
	info->cmds[flash_cmd_qior].dummyCyc = 7; /* 1 mode byte + 5 dummy cycles */
	info->cmds[flash_cmd_4qior].dummyCyc = 7;

	/* FIXME:  s25fl256s1 should be able to operate in quad and dual mode for 4byte transactions.
	 * In the current implementation errors occur for the dual and the quad mode.
	 * Zedboard's QSPI controller doesn't officially support 4byte addressing.
	 * Due to this fact quad / dual mode with 4 byte addressing maybe impossible. */
	info->readCmd = flash_cmd_4fast_read;
	info->ppCmd = flash_cmd_4pp;

	info->init = flashcfg_spansionInit;
}


static int flashcfg_micronVolatileCRWREN(const flash_info_t *info, int enable)
{
	int res;
	uint8_t tx[3];

	res = flashcfg_wren(info);
	if (res < 0) {
		return res;
	}

	/* Enable non-volatile CR write */
	memset(tx, 0, sizeof(tx));
	/* non-volatile CR write cmd code */
	tx[0] = 0xb1;

	/* Output driver strength 30 Ohms, TODO optimize */
	/* Enable hold, disable Quad I/O and Dual I/O protocols */
	/* Enable write on non-volatile register */
	tx[1] = 0xfe;
	if (enable == 0) {
		tx[1] |= 1;
	}

	/* Set default value for number of dummy clock cycles */
	/* Disable XIP */
	tx[2] = 0xff;

	qspi_start();
	res = qspi_transfer(tx, NULL, 3, FLASH_TIMEOUT_CMD_MS);
	qspi_stop();

	if (res < 0) {
		return res;
	}

	return flashcfg_wipCheck(info, FLASH_TIMEOUT_WIP_MS);
}


static int flashcfg_micronInit(const flash_info_t *info)
{
	int res;
	uint8_t tx[2];

	/* Ensure number of dummy cycles is supported by this flash. */
	if ((info->cmds[info->readCmd].dummyCyc - 1) > 0xf) {
		return -EINVAL;
	}

	res = flashcfg_micronVolatileCRWREN(info, 1);
	if (res < 0) {
		return res;
	}

	/* Volatile CR write cmd code */
	tx[0] = 0x81;

	/* Set number of dummy cycles */
	tx[1] = ((info->cmds[info->readCmd].dummyCyc - 1) << 4);
	/* Disable XIP, continuous read */
	tx[1] |= 0xb;

	qspi_start();
	res = qspi_transfer(tx, NULL, 2, FLASH_TIMEOUT_CMD_MS);
	qspi_stop();

	if (res < 0) {
		return res;
	}
	res = flashcfg_wipCheck(info, FLASH_TIMEOUT_WIP_MS);
	if (res < 0) {
		return res;
	}

	res = flashcfg_micronVolatileCRWREN(info, 0);
	if (res < 0) {
		return res;
	}

	return 0;
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

	memcpy(info->cmds, flash_defCmds, sizeof(flash_defCmds));

	/* Specific configuration */
	/* Dummy cycles chosen for 100MHz clock speed. */
	/* Dummy cycles * data lines must be divisible by 8. */

	/* Single line */
	info->cmds[flash_cmd_fast_read].dummyCyc = 8;
	info->cmds[flash_cmd_4fast_read].dummyCyc = 8;
	/* Two lines */
	info->cmds[flash_cmd_dor].dummyCyc = 4;
	info->cmds[flash_cmd_4dor].dummyCyc = 4;
	info->cmds[flash_cmd_dior].dummyCyc = 8;
	info->cmds[flash_cmd_4dior].dummyCyc = 8;
	/* Four lines */
	info->cmds[flash_cmd_qor].dummyCyc = 6;
	info->cmds[flash_cmd_4qor].dummyCyc = 6;
	info->cmds[flash_cmd_qior].dummyCyc = 10;
	info->cmds[flash_cmd_4qior].dummyCyc = 10;

	/* Default read and page program commands */
	info->readCmd = flash_cmd_qior;
	info->ppCmd = flash_cmd_qpp;

	info->init = flashcfg_micronInit;
}


static int flashcfg_winbondInit(const flash_info_t *info)
{
	int res;
	uint8_t tx[4];

	/* Write enable SR and set the SR write mode to volatile. */
	tx[0] = 0x50;
	qspi_start();
	res = qspi_transfer(tx, NULL, 2, FLASH_TIMEOUT_CMD_MS);
	qspi_stop();
	if (res < 0) {
		return res;
	}

	/* Write volatile status registers */

	/* Write status registers cmd code */
	tx[0] = 0x1;

	/* SR */
	/* Ensure SR is protected by software */
	/* Do not enable any protection. */
	tx[1] = 0;
	tx[2] = 0;
	/* Enable quad mode if needed. */
	if ((info->cmds[info->readCmd].dataLines == 4) || (info->cmds[info->ppCmd].dataLines == 4)) {
		tx[2] = (1 << 1);
	}
	/* Set output the driver strength to 25% (default value in the docs). */
	/* TODO: optimize */
	tx[3] = (3 << 5);

	qspi_start();
	res = qspi_transfer(tx, NULL, 4, FLASH_TIMEOUT_CMD_MS);
	qspi_stop();

	if (res < 0) {
		return res;
	}

	return flashcfg_wipCheck(info, FLASH_TIMEOUT_WIP_MS);
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

	memcpy(info->cmds, flash_defCmds, sizeof(flash_defCmds));

	/* 4 byte address commands are not supported. */

	/* Single line */
	info->cmds[flash_cmd_fast_read].dummyCyc = 8;
	/* Two lines */
	info->cmds[flash_cmd_dor].dummyCyc = 8;
	info->cmds[flash_cmd_dior].dummyCyc = 4; /* One mode byte */
	/* Four lines */
	info->cmds[flash_cmd_qor].dummyCyc = 8;
	info->cmds[flash_cmd_qior].dummyCyc = 6; /* One mode byte + 4 dummy cycles */

	info->readCmd = flash_cmd_qior;
	info->ppCmd = flash_cmd_qpp;

	info->init = flashcfg_winbondInit;
}


void flashcfg_jedecIDGet(flash_cmd_t *cmd)
{
	*cmd = flash_defCmds[flash_cmd_rdid];
}


int flashcfg_infoResolve(flash_info_t *info)
{
	int res = EOK;

	/* Spansion s25fl256s1 */
	if ((info->cfi.vendorData[0] == 0x1) && (info->cfi.vendorData[2] == 0x19)) {
		flashcfg_spansion(info);
	}
	/* Micron n25q128 */
	else if ((info->cfi.vendorData[0] == 0x20) && (info->cfi.vendorData[1] == 0xbb) && (info->cfi.vendorData[2] == 0x18)) {
		flashcfg_micron(info);
	}
	/* Winbond W25Q128JV - IM/JM */
	else if ((info->cfi.vendorData[0] == 0xef) && (info->cfi.vendorData[1] == 0x40) && (info->cfi.vendorData[2] == 0x18)) {
		flashcfg_winbond(info);
	}
	else {
		info->name = "Unknown";
		res = -EINVAL;
	}

	info->addrMode = (info->cfi.chipSize > 0x18) ? flash_4byteAddr : flash_3byteAddr;

	return res;
}
