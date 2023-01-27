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
} chip_t;

/* Supported NOR flash chips */
static const chip_t chips[] = {
	{ "Micron MT25QL256ABA(256Mb, 3V)", { 0x20, 0xba, 0x19 }, 4 * 1024, 32 * 1024 * 1024 },
	/* Not tested.

	{ "Micron MT25QL256ABA(1Gb, 3V)", { 0x20, 0xba, 0x21 }, 64 * 1024, 1024 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(512Mb, 3V)", { 0x20, 0xba, 0x20 }, 64 * 1024, 512 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(128Mb, 3V)", { 0x20, 0xba, 0x18 }, 64 * 1024, 128 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(16Mb, 3V)", { 0x20, 0xba, 0x17 }, 64 * 1024, 16 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(1Gb, 1.8V)", { 0x20, 0xbb, 0x21 }, 64 * 1024, 1024 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(512Mb, 1.8V)", { 0x20, 0xbb, 0x20 }, 64 * 1024, 512 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(256Mb, 1.8V)", { 0x20, 0xbb, 0x19 }, 64 * 1024, 256 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(128Mb, 1.8V)", { 0x20, 0xbb, 0x18 }, 64 * 1024, 128 * 1024 * 1024 },
	{ "Micron MT25QL256ABA(16Mb, 1.8V)", { 0x20, 0xbb, 0x17 }, 64 * 1024, 16 * 1024 * 1024 },
	*/
};

struct {
	handle_t lock;
	int init;
} flashnor_common;


static int _flashnor_qspiWriteEnable(qspi_dev_t dev)
{
	return _qspi_readBusy(dev, lut_seq_write_enable, 0, NULL, 0);
}


static int _flashnor_qspiWaitBusy(qspi_dev_t dev)
{
	uint8_t status;
	int err;
	unsigned int sleep = 1000;

	err = _qspi_readBusy(dev, lut_seq_read_status, 0, &status, 1);
	if (err < 0) {
		return err;
	}

	while (status & 1) {
		usleep(sleep);
		if (sleep < 100000) {
			sleep <<= 1;
		}
		err = _qspi_readBusy(dev, lut_seq_read_status, 0, &status, 1);
		if (err < 0) {
			return err;
		}
	}

	return EOK;
}


ssize_t flashnor_qspiRead(qspi_dev_t dev, unsigned int addr, void *buff, size_t bufflen)
{
	size_t size, len;
	int err;

	err = mutexLock(flashnor_common.lock);
	if (err < 0) {
		return err;
	}

	for (len = 0; len < bufflen; len += size) {
		if ((size = bufflen - len) > MAX_READ_LEN)
			size = MAX_READ_LEN;

		/* TODO XIP */
		err = _qspi_readBusy(dev, lut_seq_read, addr + len, ((char *)buff) + len, size);
		if (err < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}
	}

	err = mutexUnlock(flashnor_common.lock);
	if (err < 0) {
		return err;
	}

	return bufflen;
}


ssize_t flashnor_qspiWrite(qspi_dev_t dev, unsigned int addr, const void *buff, size_t bufflen)
{
	size_t size, len;
	int err;

	err = mutexLock(flashnor_common.lock);
	if (err < 0) {
		return err;
	}

	for (len = 0; len < bufflen; len += size) {
		err = _flashnor_qspiWriteEnable(dev);
		if (err < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}
		if ((size = bufflen - len) > MAX_WRITE_LEN)
			size = MAX_WRITE_LEN;

		/* Limit write size to the page aligned address (don't wrap around at the page boundary) */
		if (size > 0x100 - ((addr + len) & 0xff))
			size = 0x100 - ((addr + len) & 0xff);
		/* TODO XIP */
		err = _qspi_write(dev, lut_seq_write, addr + len, ((const char *)buff) + len, size);
		if (err < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}
		err = _flashnor_qspiWaitBusy(dev);
		if (err < 0) {
			mutexUnlock(flashnor_common.lock);
			return err;
		}
	}

	err = mutexUnlock(flashnor_common.lock);
	if (err < 0) {
		return err;
	}

	return bufflen;
}


int flashnor_qspiEraseSector(qspi_dev_t dev, unsigned int addr)
{
	int err;

	err = mutexLock(flashnor_common.lock);
	if (err < 0) {
		return err;
	}
	err = _flashnor_qspiWriteEnable(dev);
	if (err < 0) {
		mutexUnlock(flashnor_common.lock);
		return err;
	}
	err = _qspi_write(dev, lut_seq_erase, addr, NULL, 0);
	if (err < 0) {
		mutexUnlock(flashnor_common.lock);
		return err;
	}
	err = _flashnor_qspiWaitBusy(dev);
	if (err < 0) {
		mutexUnlock(flashnor_common.lock);
		return err;
	}
	return mutexUnlock(flashnor_common.lock);
}


static int get_jedec_id(qspi_dev_t dev, uint8_t data[3])
{
	int err;
	lut_seq_t seq = { .instrs = { LUT_INSTR(lut_cmd, lut_pad1, cmd_jedec), LUT_INSTR(lut_read, lut_pad1, 3), 0 } };

	err = qspi_setLutSeq(&seq, lut_seq_jedec);
	if (err < 0) {
		return err;
	}
	return _qspi_readBusy(dev, lut_seq_jedec, 0, data, 3);
}

static int enable_quad_io(qspi_dev_t dev)
{
	int err;
	lut_seq_t seq = { .instrs = { LUT_INSTR(lut_cmd, lut_pad1, cmd_qenable), 0 } };
	err = qspi_setLutSeq(&seq, lut_seq_quad_io);
	if (err < 0) {
		return err;
	}
	return _qspi_readBusy(dev, lut_seq_quad_io, 0, NULL, 0);
}


static int populate_lut(void)
{
	int err, i;
	/* clang-format off */
	struct {
		unsigned int num;
		lut_seq_t lut_seq;
	} seqs[5] = {
		{ lut_seq_write_enable, { { LUT_INSTR(lut_cmd, lut_pad4, cmd_wren), 0} } },
		{ lut_seq_read,         { { LUT_INSTR(lut_cmd, lut_pad4, cmd_4qfread_io), LUT_INSTR(lut_addr, lut_pad4, 32), LUT_INSTR(lut_dummy, lut_pad4, 10), LUT_INSTR(lut_read, lut_pad4, 0), 0} } },
		{ lut_seq_write,        { { LUT_INSTR(lut_cmd, lut_pad4, cmd_4qwrite), LUT_INSTR(lut_addr, lut_pad4, 32), LUT_INSTR(lut_write, lut_pad4, 0), 0} } },
		{ lut_seq_erase,        { { LUT_INSTR(lut_cmd, lut_pad4, cmd_4erase4), LUT_INSTR(lut_addr, lut_pad4, 32), 0 } } },
		{ lut_seq_read_status,  { { LUT_INSTR(lut_cmd, lut_pad4, cmd_rdsr), LUT_INSTR(lut_read, lut_pad4, 1), 0 } }  },
	};
	/* clang-format on */
	for (i = 0; i < sizeof(seqs) / sizeof(*seqs); i++) {
		err = qspi_setLutSeq(&seqs[i].lut_seq, seqs[i].num);
		if (err < 0) {
			return err;
		}
	}
	return EOK;
}


int _flashnor_qspiInit(qspi_dev_t dev, storage_t *storage_dev)
{
	int i, err;
	uint8_t jedec[3];

	err = _qspi_init(dev);
	if (err < 0) {
		return err;
	}
	qspi_setTCSH(0);
	qspi_setTCSS(0);

	err = get_jedec_id(dev, jedec);
	if (err < 0) {
		return err;
	}

	printf("imx6ull-flashnor: JEDEC ID: %#02x %#02x %#02x\n", jedec[0], jedec[1], jedec[2]);

	for (i = 0; i < sizeof(chips) / sizeof(chips[0]); i++) {
		if (!memcmp(jedec, chips[i].jedec, sizeof(chips[i].jedec))) {
			printf("imx6ull-flashnor: %s %uMbit NOR\n", chips[i].name, 8 * chips[i].flashsz >> 20);
			if (!flashnor_common.init) {
				err = mutexCreate(&flashnor_common.lock);
				if (err < 0) {
					return err;
				}
				err = populate_lut();
				if (err < 0) {
					return err;
				}
				(err = enable_quad_io(dev));
				if (err < 0) {
					return err;
				}
			}
			flashnor_common.init = 1;
			/* TODO init storage device */
			return EOK;
		}
	}

	return -ENODEV;
}
