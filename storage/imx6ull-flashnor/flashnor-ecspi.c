/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI NOR flash driver
 *
 * Copyright 2021, 2023 Phoenix Systems
 * Author: Lukasz Kosinski, Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include <sys/types.h>

#include <imx6ull-ecspi.h>

#include "imx6ull-flashnor-ecspi.h"


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


/* Supported NOR flash chips */
static const struct {
	unsigned char jedec[3];        /* Chip JEDEC ID */
	const flashnor_devInfo_t info; /* Chip info. */
} chips[] = {
	{ .jedec = { 0xef, 0x40, 0x15 }, .info = { .name = "Winbond 25Q16JV", .erasesz = 4 * 1024, .size = 2 * 1024 * 1024, .writeBuffsz = 256 } },
	{ .jedec = { 0x1f, 0x47, 0x08 }, .info = { .name = "Adesto AT25FF321A", .erasesz = 4 * 1024, .size = 4 * 1024 * 1024, .writeBuffsz = 256 } },
	{ .jedec = { 0x1f, 0x87, 0x01 }, .info = { .name = "Adesto AT25SF321A", .erasesz = 4 * 1024, .size = 4 * 1024 * 1024, .writeBuffsz = 256 } },
};


static struct {
	handle_t locks[4];
} flashnor_common;


static int _flashnor_ecspiWriteEnable(int ndev)
{
	unsigned char cmd = cmd_wren;
	int err;

	if ((err = ecspi_exchangeBusy(ndev, &cmd, &cmd, 1)) < 0) {
		return err;
	}

	return EOK;
}


static int _flashnor_ecspiWaitBusy(int ndev)
{
	unsigned char cmd[2] = { cmd_rdsr1, 0xff };
	unsigned int sleep = 1000;
	int err;

	err = ecspi_exchangeBusy(ndev, cmd, cmd, sizeof(cmd));
	if (err < 0) {
		return err;
	}

	while ((cmd[1] & 1u) != 0u) {
		usleep(sleep);
		if (sleep < 100000u) {
			sleep <<= 1;
		}

		cmd[0] = cmd_rdsr1;
		cmd[1] = 0xff;

		err = ecspi_exchangeBusy(ndev, cmd, cmd, sizeof(cmd));
		if (err < 0) {
			return err;
		}
	}

	return EOK;
}


static ssize_t flashnor_ecspiRead(int ndev, off_t offs, void *buff, size_t bufflen)
{
	unsigned char cmd[256];
	size_t size, len = 0;
	int err;


	(void)mutexLock(flashnor_common.locks[ndev - 1]);
	while (len < bufflen) {
		size = bufflen - len;
		if (size > sizeof(cmd) - 4u) {
			size = sizeof(cmd) - 4u;
		}

		cmd[0] = cmd_read;
		cmd[1] = offs >> 16;
		cmd[2] = offs >> 8;
		cmd[3] = offs;
		(void)memset(cmd + 4, 0xff, size);

		err = ecspi_exchange(ndev, cmd, cmd, size + 4u);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
			return err;
		}

		(void)memcpy((char *)buff + len, cmd + 4, size);
		offs += size;
		len += size;
	}

	(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
	return bufflen;
}


static ssize_t flashnor_ecspiWrite(int ndev, off_t offs, const void *buff, size_t bufflen)
{
	unsigned char cmd[256];
	size_t size, len = 0;
	int err;

	(void)mutexLock(flashnor_common.locks[ndev - 1]);
	while (len < bufflen) {
		err = _flashnor_ecspiWriteEnable(ndev);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
			return err;
		}

		/* Limit write size to the buffer size - 4 bytes for the command */
		size = bufflen - len;
		if (size > sizeof(cmd) - 4u) {
			size = sizeof(cmd) - 4u;
		}

		/* Limit write size to the page aligned offsess (don't wrap around at the page boundary) */
		if (size > (0x100u - (offs & 0xffu))) {
			size = 0x100u - (offs & 0xffu);
		}

		cmd[0] = cmd_write;
		cmd[1] = offs >> 16;
		cmd[2] = offs >> 8;
		cmd[3] = offs;
		(void)memcpy(cmd + 4, (char *)buff + len, size);

		err = ecspi_exchange(ndev, cmd, cmd, size + 4u);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
			return err;
		}
		err = _flashnor_ecspiWaitBusy(ndev);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
			return err;
		}

		offs += size;
		len += size;
	}

	(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
	return bufflen;
}


static int flashnor_ecspiErase(int ndev, off_t offs, size_t size)
{
	unsigned char cmd[4] = { cmd_erase4, offs >> 16, offs >> 8, offs };
	int err;
	size_t len;

	(void)mutexLock(flashnor_common.locks[ndev - 1]);
	for (len = 0; len < size; len += 4u * 1024u) {
		err = _flashnor_ecspiWriteEnable(ndev);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
			return err;
		}
		err = ecspi_exchangeBusy(ndev, cmd, cmd, sizeof(cmd));
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
			return err;
		}
		err = _flashnor_ecspiWaitBusy(ndev);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
			return err;
		}
	}

	(void)mutexUnlock(flashnor_common.locks[ndev - 1]);
	return EOK;
}


static const flashnor_ops_t flashnor_ecspiOps = {
	.erase = flashnor_ecspiErase,
	.write = flashnor_ecspiWrite,
	.read = flashnor_ecspiRead,
};


int flashnor_ecspiInit(int ndev, flashnor_info_t *info)
{
	unsigned char *jedec, data[4] = { cmd_jedec, 0xff, 0xff, 0xff };
	int err;
	unsigned i;

	/* Enable instance channel 0 */
	err = ecspi_init(ndev, 1);
	if (err < 0) {
		return err;
	}

	/* Operate in mode 0 (POL = 0, PHA = 0) */
	err = ecspi_setMode(ndev, 0, 0);
	if (err < 0) {
		return err;
	}

	/* Set SCLK to 30 MHz */
	err = ecspi_setClockDiv(ndev, 1, 0);
	if (err < 0) {
		return err;
	}

	/* No CS-SCLK delay */
	err = ecspi_setCSDelay(ndev, 0);
	if (err < 0) {
		return err;
	}

	/* No SS-SS delay */
	err = ecspi_setSSDelay(ndev, 0);
	if (err < 0) {
		return err;
	}

	/* Detect NOR flash chip */
	err = ecspi_exchangeBusy(ndev, data, data, sizeof(data));
	if (err < 0) {
		return err;
	}

	jedec = data + 1;
	(void)printf("imx6ull-flashnor: JEDEC ID: %#02x %#02x %#02x\n", jedec[0], jedec[1], jedec[2]);

	for (i = 0; i < sizeof(chips) / sizeof(chips[0]); i++) {
		if (memcmp(jedec, chips[i].jedec, sizeof(chips[i].jedec)) == 0) {
			(void)printf("imx6ull-flashnor: %s %zuMbit NOR\n", chips[i].info.name, (8u * chips[i].info.size) >> 20u);

			err = mutexCreate(&flashnor_common.locks[ndev - 1]);
			if (err < 0) {
				return err;
			}

			info->devInfo = &chips[i].info;
			info->ndev = ndev;
			info->ops = &flashnor_ecspiOps;

			return EOK;
		}
	}

	return -ENODEV;
}
