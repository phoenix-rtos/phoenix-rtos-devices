/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Intel command set flash interface
 *
 * Copyright 2024 Phoenix Systems
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
/* Flash status register */
#define STS_FULL_CHECK ((1 << 5) | (1 << 4) | (1 << 3) | (1 << 1))
#define XSR_WRBUF_RDY  (1 << 7) /* Write buffer ready */


static uint8_t intel_statusRead(volatile uint8_t *base)
{
	*(base + FLASH_VALID_OFFS) = CMD_RD_STATUS;

	return *base;
}


static void intel_statusClear(volatile uint8_t *base)
{
	*(base + FLASH_VALID_OFFS) = INTEL_CMD_CLR_STATUS;
}


static int intel_statusCheck(volatile uint8_t *base, const char *msg)
{
	int ret = 0;
	uint8_t status = intel_statusRead(base);

	if ((status & STS_FULL_CHECK) != 0) {
		LOG_ERROR("dev/flash: %s error: status 0x%x\n", msg, status);
		ret = -EIO;
	}

	intel_statusClear(base);

	return ret;
}


static void intel_issueReset(volatile uint8_t *base)
{
	*(base + FLASH_VALID_OFFS) = INTEL_CMD_RESET;
}

static void intel_issueWriteBuffer(volatile uint8_t *base, off_t sectorOffs, off_t programOffs, size_t len)
{
	uint8_t xsr;
	do {
		*(base + programOffs) = INTEL_CMD_WR_BUF;
		xsr = *base;
	} while ((xsr & XSR_WRBUF_RDY) == 0);

	*(base + programOffs) = (len - 1) & 0xff;
}


static void intel_issueWriteConfirm(volatile uint8_t *base, off_t sectorOffs)
{
	*(base + sectorOffs) = INTEL_CMD_WR_CONFIRM;
}


static void intel_issueSectorErase(volatile uint8_t *base, off_t sectorOffs)
{
	*(base + sectorOffs) = INTEL_CMD_BE_CYC1;
	*(base + sectorOffs) = INTEL_CMD_WR_CONFIRM;
}


static void intel_enterQuery(volatile uint8_t *base, off_t sectorOffs)
{
	*(base + sectorOffs) = CMD_RD_QUERY;
}


static void intel_exitQuery(volatile uint8_t *base)
{
	(void)base;
}


void intel_register(void)
{
	static const flash_ops_t ops = {
		.statusRead = intel_statusRead,
		.statusCheck = intel_statusCheck,
		.statusClear = intel_statusClear,
		.issueReset = intel_issueReset,
		.issueWriteBuffer = intel_issueWriteBuffer,
		.issueWriteConfirm = intel_issueWriteConfirm,
		.issueSectorErase = intel_issueSectorErase,
		.issueChipErase = NULL, /* Not supported */
		.enterQuery = intel_enterQuery,
		.exitQuery = intel_exitQuery
	};

	static const flash_dev_t intel_devices[] = {
		{
			.name = "Intel JS28F640J3",
			.vendor = 0x89u,
			.device = 0x0017u,
			.chipWidth = 8,
			.statusRdyMask = (1 << 7),
			.usePolling = 0,
			.ops = &ops,
		},
	};

	for (size_t i = 0; i < sizeof(intel_devices) / sizeof(intel_devices[0]); i++) {
		flash_register(&intel_devices[i]);
	}
}
