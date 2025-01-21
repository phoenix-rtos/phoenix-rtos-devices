/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * AMD command set flash interface
 *
 * Copyright 2025 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "cmds.h"
#include "flash.h"

#include <errno.h>


/* Valid offset on flash - for executing commands */
#define FLASH_VALID_OFFS 0x0
#define STS_FULL_CHECK   ((1 << 5) | (1 << 4) | (1 << 3) | (1 << 1))

/* TODO: adapt also to x16 interface */

static void amd_unlockSequence(volatile uint8_t *base)
{
	*(base + 0x0aaa) = 0xaa;
	*(base + 0x0555) = 0x55;
}


static void amd_issueReset(volatile uint8_t *base)
{
	*(base + FLASH_VALID_OFFS) = AMD_CMD_RESET;
}


static uint8_t amd_statusRead(volatile uint8_t *base)
{
	*(base + 0x0aaa) = CMD_RD_STATUS;

	return *(base + FLASH_VALID_OFFS);
}


static void amd_statusClear(volatile uint8_t *base)
{
	*(base + 0x0aaa) = AMD_CMD_CLR_STATUS;
}


static int amd_statusCheck(volatile uint8_t *base, const char *msg)
{
	int ret = 0;
	uint8_t status = amd_statusRead(base);

	if ((status & STS_FULL_CHECK) != 0) {
		LOG_ERROR("%s error: status 0x%x", msg, status);
		ret = -EIO;
	}

	return ret;
}


static void amd_issueWriteBuffer(volatile uint8_t *base, off_t sectorOffs, off_t programOffs, size_t len)
{
	(void)programOffs;

	amd_unlockSequence(base);

	*(base + sectorOffs) = AMD_CMD_WR_BUF;
	*(base + sectorOffs) = (len - 1) & 0xff;
}


static void amd_issueWriteConfirm(volatile uint8_t *base, off_t sectorOffs)
{
	*(base + sectorOffs) = AMD_CMD_WR_CONFIRM;
}


static void amd_issueSectorErase(volatile uint8_t *base, off_t sectorOffs)
{
	amd_unlockSequence(base);
	*(base + 0x0aaa) = AMD_CMD_BE_CYC1;

	amd_unlockSequence(base);
	*(base + sectorOffs) = AMD_CMD_BE_CYC2;
}


static void amd_issueChipErase(volatile uint8_t *base)
{
	amd_unlockSequence(base);
	*(base + 0x0aaa) = AMD_CMD_CE_CYC1;

	amd_unlockSequence(base);
	*(base + 0x0aaa) = AMD_CMD_CE_CYC2;
}


static void amd_enterQuery(volatile uint8_t *base, off_t sectorOffs)
{
	*(base + sectorOffs + 0xaa) = CMD_RD_QUERY;
}


static void amd_exitQuery(volatile uint8_t *base)
{
	*base = AMD_CMD_EXIT_QUERY;
}


void ftmctrl_amd_register(void)
{
	static const flash_ops_t ops = {
		.statusRead = amd_statusRead,
		.statusCheck = amd_statusCheck,
		.statusClear = amd_statusClear,
		.issueReset = amd_issueReset,
		.issueWriteBuffer = amd_issueWriteBuffer,
		.issueWriteConfirm = amd_issueWriteConfirm,
		.issueSectorErase = amd_issueSectorErase,
		.issueChipErase = amd_issueChipErase,
		.enterQuery = amd_enterQuery,
		.exitQuery = amd_exitQuery
	};

	static const flash_dev_t amd_devices[] = {
		{
			.name = "Infineon S29GL01/512T",
			.vendor = 0x01,
			.device = 0x227e,
			.chipWidth = 16,
			.statusRdyMask = (1 << 7),
			.usePolling = 1,
			.ops = &ops,
		},
	};

	for (size_t i = 0; i < (sizeof(amd_devices) / sizeof(amd_devices[0])); ++i) {
		ftmctrl_flash_register(&amd_devices[i]);
	}
}
