/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI NOR flash driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <sys/threads.h>
#include <sys/types.h>

#include <ecspi.h>
#include <meterfs.h>

#include "flashnor-ecspi.h"


enum {
	cmd_wren = 0x06,       /* Write Enable */
	cmd_wrdi = 0x04,       /* Write Disable */
	cmd_rdsr1 = 0x05,      /* Read Status Register1 */
	cmd_rdsr2 = 0x35,      /* Read Status Register2 */
	cmd_wrsr = 0x01,       /* Write Status Register */
	cmd_write = 0x02,      /* Page Write */
	cmd_qwrite = 0x32,     /* Quad Page Write */
	cmd_erase64 = 0xd8,    /* Block Erase (64 KB) */
	cmd_erase32 = 0x52,    /* Block Erase (32 KB) */
	cmd_erase4 = 0x20,     /* Sector Erase (4 KB) */
	cmd_erase_chip = 0x60, /* Chip Erase */
	cmd_suspend = 0x75,    /* Suspend Erase */
	cmd_resume = 0x7a,     /* Resume Erase */
	cmd_pwrdown = 0xb9,    /* Power Down */
	cmd_hpm = 0xa3,        /* High Performance Mode */
	cmd_rst = 0xff,        /* Mode Bit Reset */
	cmd_release = 0xab,    /* Release Power Down or HPM */
	cmd_id = 0x90,         /* Manufacturer / Device ID */
	cmd_uniq = 0x4b,       /* Read Unique ID */
	cmd_jedec = 0x9f,      /* JEDEC ID */
	cmd_read = 0x03,       /* Read Data */
	cmd_fread = 0x0b,      /* Fast Read */
	cmd_dfread_out = 0x3b, /* Dual Fast Read Output */
	cmd_dfread_io = 0xbb,  /* Dual Fast Read I/O */
	cmd_qfread_out = 0x6b, /* Quad Fast Read Output */
	cmd_qfread_io = 0xeb   /* Quad Fast Read I/O */
};


typedef struct {
	const char *name;       /* Chip name */
	unsigned char jedec[3]; /* Chip JEDEC ID */
	unsigned int sectorsz;  /* Chip sector size in bytes */
	unsigned int flashsz;   /* Chip storage size in bytes */
} chip_t;


/* Supported NOR flash chips */
static const chip_t chips[] = {
	{ "Winbond 25Q16JV", { 0xef, 0x40, 0x15 }, 4 * 1024, 2 * 1024 * 1024 },
	{ "Adesto AT25FF321A", { 0x1f, 0x47, 0x08 }, 4 * 1024, 4 * 1024 * 1024 },
	{ "Adesto AT25SF321A", { 0x1f, 0x87, 0x01 }, 4 * 1024, 4 * 1024 * 1024 },
};


static struct {
	unsigned int ndev;
	meterfs_ctx_t ctx;
	handle_t lock;
} flashnor_common;


static int _flashnor_ecspiWriteEnable(void)
{
	unsigned char cmd = cmd_wren;
	int err;

	if ((err = ecspi_exchangeBusy(flashnor_common.ndev, &cmd, &cmd, 1)) < 0)
		return err;

	return EOK;
}


static int _flashnor_ecspiWaitBusy(void)
{
	unsigned char cmd[2] = { cmd_rdsr1, 0xff };
	unsigned int sleep = 1000;
	int err;

	if ((err = ecspi_exchangeBusy(flashnor_common.ndev, cmd, cmd, sizeof(cmd))) < 0)
		return err;

	while (cmd[1] & 1) {
		usleep(sleep);
		if (sleep < 100000)
			sleep <<= 1;

		cmd[0] = cmd_rdsr1;
		cmd[1] = 0xff;

		if ((err = ecspi_exchangeBusy(flashnor_common.ndev, cmd, cmd, sizeof(cmd))) < 0)
			return err;
	}

	return EOK;
}


ssize_t flashnor_ecspiRead(unsigned int addr, void *buff, size_t bufflen)
{
	unsigned char cmd[256];
	size_t size, len = 0;
	int err;

	mutexLock(flashnor_common.lock);

	while (len < bufflen) {
		if ((size = bufflen - len) > sizeof(cmd) - 4)
			size = sizeof(cmd) - 4;

		cmd[0] = cmd_read;
		cmd[1] = addr >> 16;
		cmd[2] = addr >> 8;
		cmd[3] = addr;
		memset(cmd + 4, 0xff, size);

		if ((err = ecspi_exchange(flashnor_common.ndev, cmd, cmd, size + 4)) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}

		memcpy((char *)buff + len, cmd + 4, size);
		addr += size;
		len += size;
	}

	mutexUnlock(flashnor_common.lock);

	return bufflen;
}


ssize_t flashnor_ecspiWrite(unsigned int addr, const void *buff, size_t bufflen)
{
	unsigned char cmd[256];
	size_t size, len = 0;
	int err;

	mutexLock(flashnor_common.lock);

	while (len < bufflen) {
		if ((err = _flashnor_ecspiWriteEnable()) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}

		/* Limit write size to the buffer size - 4 bytes for the command */
		if ((size = bufflen - len) > sizeof(cmd) - 4)
			size = sizeof(cmd) - 4;

		/* Limit write size to the page aligned address (don't wrap around at the page boundary) */
		if (size > 0x100 - (addr & 0xff))
			size = 0x100 - (addr & 0xff);

		cmd[0] = cmd_write;
		cmd[1] = addr >> 16;
		cmd[2] = addr >> 8;
		cmd[3] = addr;
		memcpy(cmd + 4, (char *)buff + len, size);

		if ((err = ecspi_exchange(flashnor_common.ndev, cmd, cmd, size + 4)) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}

		if ((err = _flashnor_ecspiWaitBusy()) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}

		addr += size;
		len += size;
	}

	mutexUnlock(flashnor_common.lock);

	return bufflen;
}


int flashnor_ecspiEraseSector(unsigned int addr)
{
	unsigned char cmd[4] = { cmd_erase4, addr >> 16, addr >> 8, addr };
	int err;

	mutexLock(flashnor_common.lock);

	if ((err = _flashnor_ecspiWriteEnable()) < 0) {
		mutexUnlock(flashnor_common.lock);
		return err;
	}

	if ((err = ecspi_exchangeBusy(flashnor_common.ndev, cmd, cmd, sizeof(cmd))) < 0) {
		mutexUnlock(flashnor_common.lock);
		return err;
	}

	if ((err = _flashnor_ecspiWaitBusy()) < 0) {
		mutexUnlock(flashnor_common.lock);
		return err;
	}

	mutexUnlock(flashnor_common.lock);

	return EOK;
}


int flashnor_ecspiInit(unsigned int ndev, storage_t *dev)
{
	unsigned char *jedec, data[4] = { cmd_jedec, 0xff, 0xff, 0xff };
	int i, err;

	/* Enable instance channel 0 */
	if ((err = ecspi_init(ndev, 1)) < 0)
		return err;

	/* Operate in mode 0 (POL = 0, PHA = 0) */
	if ((err = ecspi_setMode(ndev, 0, 0)) < 0)
		return err;

	/* Set SCLK to 30 MHz */
	if ((err = ecspi_setClockDiv(ndev, 1, 0)) < 0)
		return err;

	/* No CS-SCLK delay */
	if ((err = ecspi_setCSDelay(ndev, 0)) < 0)
		return err;

	/* No SS-SS delay */
	if ((err = ecspi_setSSDelay(ndev, 0)) < 0)
		return err;

	/* Detect NOR flash chip */
	if ((err = ecspi_exchangeBusy(ndev, data, data, sizeof(data))) < 0)
		return err;
	jedec = data + 1;
	printf("imx6ull-flashnor: JEDEC ID: %#02x %#02x %#02x\n", jedec[0], jedec[1], jedec[2]);

	for (i = 0; i < sizeof(chips) / sizeof(chips[0]); i++) {
		if (!memcmp(jedec, chips[i].jedec, sizeof(chips[i].jedec))) {
			printf("imx6ull-flashnor: %s %uMbit NOR\n", chips[i].name, 8 * chips[i].flashsz >> 20);

			if ((err = mutexCreate(&flashnor_common.lock)) < 0)
				return err;

			flashnor_common.ndev = ndev;
			flashnor_common.ctx.sz = chips[i].flashsz;
			flashnor_common.ctx.sectorsz = chips[i].sectorsz;
			flashnor_common.ctx.offset = 0;
			flashnor_common.ctx.read = flashnor_ecspiRead;
			flashnor_common.ctx.write = flashnor_ecspiWrite;
			flashnor_common.ctx.eraseSector = flashnor_ecspiEraseSector;
			flashnor_common.ctx.powerCtrl = NULL;

			if ((err = meterfs_init(&flashnor_common.ctx)) < 0) {
				resourceDestroy(flashnor_common.lock);
				return err;
			}

			dev->ctx = &flashnor_common.ctx;
			dev->parent = NULL;
			dev->start = 0;
			dev->size = flashnor_common.ctx.sz / flashnor_common.ctx.sectorsz;

			return EOK;
		}
	}

	return -ENODEV;
}
