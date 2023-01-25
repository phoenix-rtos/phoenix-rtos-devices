/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL QSPI NOR flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/threads.h>
#include <sys/types.h>
#include <qspi.h>
#include "flashnor-qspi.h"

enum {
	cmd_wren = 0x06,      /* Write Enable */
	cmd_rdsr = 0x05,      /* Read Status Register */
	cmd_4qwrite = 0x34,   /* 4-byte Quad I/O Write */
	cmd_qenable = 0x35,   /* Enable Quad I/O */
	cmd_4erase4 = 0x21,   /*  4-byte  Sector Erase (4 KB) */
	cmd_jedec = 0x9f,     /* JEDEC ID */
	cmd_4qfread_io = 0xec /*  4-byte Quad Fast Read I/O */
};

enum {
	lut_seq_jedec,
	lut_seq_write_enable,
	lut_seq_write,
	lut_seq_erase,
	lut_seq_read,
	lut_seq_read_status,
	lut_seq_quad_io,
};

typedef struct {
	const char *name;       /* Chip name */
	unsigned char jedec[3]; /* Chip JEDEC ID */
	unsigned int sectorsz;  /* Chip sector size in bytes */
	unsigned int flashsz;   /* Chip storage size in bytes */
	size_t pagesz;          /* Chip storage size in bytes */
} chip_t;

/* Supported NOR flash chips */
static const chip_t chips[] = {
	{ "Micron MT25QL256ABA(256Mb, 3V)", { 0x20, 0xBA, 0x19 }, 4 * 1024, 256 * 1024 * 1024, 256 },
	/* Not tested.

	{ "Micron MT25QL256ABA(1Gb, 3V)", { 0x20, 0xBA, 0x21 }, 64 * 1024, 1024 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(512Mb, 3V)", { 0x20, 0xBA, 0x20 }, 64 * 1024, 512 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(128Mb, 3V)", { 0x20, 0xBA, 0x18 }, 64 * 1024, 128 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(16Mb, 3V)", { 0x20, 0xBA, 0x17 }, 64 * 1024, 16 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(1Gb, 1.8V)", { 0x20, 0xBB, 0x21 }, 64 * 1024, 1024 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(512Mb, 1.8V)", { 0x20, 0xBB, 0x20 }, 64 * 1024, 512 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(256Mb, 1.8V)", { 0x20, 0xBB, 0x19 }, 64 * 1024, 256 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(128Mb, 1.8V)", { 0x20, 0xBB, 0x18 }, 64 * 1024, 128 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(16Mb, 1.8V)", { 0x20, 0xBB, 0x17 }, 64 * 1024, 16 * 1024 * 1024 },
	*/
};

struct {
	handle_t lock;
	bool init;
	size_t pagesz[2];
} flashnor_common;


static int _flashnor_qspiWriteEnable(qspi_dev_t dev)
{
	return _qspi_read(dev, lut_seq_write_enable, 0, NULL, 0);
}


static int _flashnor_qspiWaitBusy(qspi_dev_t dev)
{
	uint8_t status;
	int err;
	do {
		if ((err = _qspi_read(dev, lut_seq_read_status, 0, &status, 1)) < 0) {
			return err;
		}
		printf("STATUS READ %d\n", status);
		usleep(5e5);
	} while (status & 1);
	return EOK;
}


ssize_t flashnor_qspiRead(qspi_dev_t dev, unsigned int addr, void *buff, size_t bufflen)
{
	size_t size, len;
	int err;

	if ((err = mutexLock(flashnor_common.lock)) < 0) {
		return err;
	}

	for (len = 0; len < bufflen; len += size) {
		if ((size = bufflen - len) > MAX_READ_LEN)
			size = MAX_READ_LEN;

		/* TODO XIP */
		if ((err = _qspi_read(dev, lut_seq_read, addr + len, ((char *)buff) + len, size)) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}
	}

	if ((err = mutexUnlock(flashnor_common.lock)) < 0) {
		return err;
	}

	return bufflen;
}


ssize_t flashnor_qspiWrite(qspi_dev_t dev, unsigned int addr, const void *buff, size_t bufflen)
{
	size_t size, len;
	int err;

	if ((err = mutexLock(flashnor_common.lock)) < 0) {
		return err;
	}

	for (len = 0; len < bufflen; len += size) {
		if ((err = _flashnor_qspiWriteEnable(dev)) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}
		if ((size = bufflen - len) > MAX_WRITE_LEN)
			size = MAX_WRITE_LEN;

		/* Limit write size to the page aligned address (don't wrap around at the page boundary) */
		if (size > flashnor_common.pagesz[dev] - ((addr + len) % flashnor_common.pagesz[dev]))
			size = flashnor_common.pagesz[dev] - ((addr + len) % flashnor_common.pagesz[dev]);
		printf("SIZE %d\n", size);
		if (size == 0) {
			printf("BUFFLEN %d\n", bufflen);
			printf("LEN %d\n", len);
			printf("PAGE SIZE %d\n", flashnor_common.pagesz[dev]);
			printf("ADDR %d\n", addr + len);
			return -1;
		}
		/* TODO XIP */
		if ((err = _qspi_write(dev, lut_seq_write, addr + len, ((const char *)buff) + len, size)) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}

		if ((err = _flashnor_qspiWaitBusy(dev)) < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}
		usleep(5e5);
	}

	if ((err = mutexUnlock(flashnor_common.lock)) < 0) {
		return err;
	}

	return bufflen;
}


int flashnor_qspiEraseSector(qspi_dev_t dev, unsigned int addr)
{
	int err;
	if ((err = mutexLock(flashnor_common.lock)) < 0) {
		return err;
	}
	if ((err = _flashnor_qspiWriteEnable(dev)) < 0) {
		return err;
	}
	if ((err = _qspi_write(dev, lut_seq_erase, addr, NULL, 0)) < 0) {
		return err;
	}
	if ((err = _flashnor_qspiWaitBusy(dev)) < 0) {
		return err;
	}
	return mutexUnlock(flashnor_common.lock);
}


static int get_jedec_id(qspi_dev_t dev, uint8_t data[3])
{
	int err;
	lut_seq_t seq = { .instrs = { LUT_INSTR(lut_cmd, 0, cmd_jedec), LUT_INSTR(lut_read, 0, 3), 0 } };
	if ((err = qspi_setLutSeq(&seq, lut_seq_jedec)) < 0) {
		return err;
	}
	return _qspi_read(dev, lut_seq_jedec, 0, data, 3);
}

static int enable_quad_io(qspi_dev_t dev)
{
	int err;
	lut_seq_t seq = { .instrs = { LUT_INSTR(lut_cmd, 0, cmd_qenable), 0 } };
	if ((err = qspi_setLutSeq(&seq, lut_seq_quad_io) < 0)) {
		return err;
	}
	return _qspi_read(dev, lut_seq_quad_io, 0, NULL, 0);
}


static int populate_lut()
{
	int err, i;
	/* clang-format off */
	struct {
		unsigned int num;
		lut_seq_t lut_seq;
	} seqs[5] = {
		{ lut_seq_write_enable, { { LUT_INSTR(lut_cmd, 2, cmd_wren), 0} } },
		{ lut_seq_read,         { { LUT_INSTR(lut_cmd, 2, cmd_4qfread_io), LUT_INSTR(lut_addr, 2, 32), LUT_INSTR(lut_dummy, 2, 10), LUT_INSTR(lut_read, 2, 0), 0} } },
		{ lut_seq_write,        { { LUT_INSTR(lut_cmd, 2, cmd_4qwrite), LUT_INSTR(lut_addr, 2, 32), LUT_INSTR(lut_write, 2, 0), 0} } },
		{ lut_seq_erase,        { { LUT_INSTR(lut_cmd, 2, cmd_4erase4), LUT_INSTR(lut_addr, 2, 32), 0 } } },
		{ lut_seq_read_status,  { { LUT_INSTR(lut_cmd, 2, cmd_rdsr), LUT_INSTR(lut_read, 2, 1), 0 } }  },
	};
	/* clang-format on */
	for (i = 0; i < 5; i++) {
		if ((err = qspi_setLutSeq(&seqs[i].lut_seq, seqs[i].num)) < 0) {
			return err;
		}
	}
	return EOK;
}


int _flashnor_qspiInit(qspi_dev_t dev, storage_t *storage_dev)
{
	int i, err;
	uint8_t jedec[3];

	if ((err = _qspi_init(dev)) < 0) {
		return err;
	}
	qspi_setTCSH(0);
	qspi_setTCSS(0);

	if ((err = get_jedec_id(dev, jedec)) < 0) {
		return err;
	}

	printf("imx6ull-flashnor: JEDEC ID: %#02x %#02x %#02x\n", jedec[0], jedec[1], jedec[2]);

	for (i = 0; i < sizeof(chips) / sizeof(chips[0]); i++) {
		if (!memcmp(jedec, chips[i].jedec, sizeof(chips[i].jedec))) {
			printf("imx6ull-flashnor: %s %uMbit NOR\n", chips[i].name, 8 * chips[i].flashsz >> 20);
			flashnor_common.pagesz[dev] = chips[i].pagesz;
			if (!flashnor_common.init) {
				if ((err = mutexCreate(&flashnor_common.lock)) < 0) {
					return err;
				}
				if ((err = populate_lut()) < 0) {
					return err;
				}
				if ((err = enable_quad_io(dev))) {
					return err;
				}
			}
			flashnor_common.init = true;
			/* TODO init storage device */
			return EOK;
		}
	}

	return -ENODEV;
}
