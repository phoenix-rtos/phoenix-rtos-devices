/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * i.MX RT NOR flash device driver
 *
 * Copyright 2021-2023 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include "fspi.h"
#include "nor.h"


#define FLASH_ID(vid, pid) ((vid & 0xff) | (pid & 0xff00) | ((pid & 0xff) << 16))
#define LUT_SEQIDX(code)   (code)
#define LUT_SEQNUM(code)   (0)


static const char *nor_vendors[] = {
	"\xef Winbond",
	"\x20 Micron",
	"\x9d ISSI",
	"\xc2 Macronix",
	NULL
};


static const struct nor_info flashInfo[] = {
	/* Winbond */
	{ FLASH_ID(0xef, 0x4015), "W25Q16", 2 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0xef, 0x4016), "W25Q32", 4 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0xef, 0x4017), "W25Q64", 8 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0xef, 0x4018), "W25Q128", 16 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0xef, 0x4019), "W25Q256JVEIQ", 32 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0xef, 0x6019), "W25Q256JW-Q", 32 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0xef, 0x8019), "W25Q256JW-M", 32 * 1024 * 1024, 0x100, 0x1000 },

	/* ISSI */
	{ FLASH_ID(0x9d, 0x7016), "IS25WP032", 4 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0x9d, 0x7017), "IS25WP064", 8 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0x9d, 0x7018), "IS25WP128", 16 * 1024 * 1024, 0x100, 0x1000 },

	/* Micron */
	{ FLASH_ID(0x20, 0xba19), "MT25QL256", 32 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0x20, 0xba20), "MT25QL512", 64 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0x20, 0xba21), "MT25QL01G", 128 * 1024 * 1024, 0x100, 0x1000 },
	{ FLASH_ID(0x20, 0xba22), "MT25QL02G", 256 * 1024 * 1024, 0x100, 0x1000 },

	/* Macronix (MXIX) */
	{ FLASH_ID(0xc2, 0x2016), "MX25L3233", 4 * 1024 * 1024, 0x100, 0x1000 },
};


static int nor_readID(flexspi_t *fspi, uint8_t port, uint32_t *retValue, time_t timeout)
{
	struct xferOp xfer;

	xfer.op = xfer_opRead;
	xfer.port = port;
	xfer.timeout = timeout;
	xfer.addr = 0;
	xfer.seqIdx = LUT_SEQIDX(fspi_readID);
	xfer.seqNum = LUT_SEQNUM(fspi_readID);
	xfer.data.read.ptr = retValue;
	xfer.data.read.sz = 3;

	return flexspi_xferExec(fspi, &xfer);
}


int nor_readStatus(flexspi_t *fspi, uint8_t port, uint8_t *statusByte, time_t timeout)
{
	struct xferOp xfer;

	xfer.op = xfer_opRead;
	xfer.port = port;
	xfer.timeout = timeout;
	xfer.addr = 0;
	xfer.seqIdx = LUT_SEQIDX(fspi_readStatus);
	xfer.seqNum = LUT_SEQNUM(fspi_readStatus);
	xfer.data.read.ptr = statusByte;
	xfer.data.read.sz = 1;

	return flexspi_xferExec(fspi, &xfer);
}


int nor_waitBusy(flexspi_t *fspi, uint8_t port, time_t timeout)
{
	int res;
	uint8_t status = 0;

	do {
		res = nor_readStatus(fspi, port, &status, timeout);
		if (res < EOK) {
			return res;
		}
	} while ((status & 1) != 0);

	return EOK;
}


int nor_writeEnable(flexspi_t *fspi, uint8_t port, int enable, time_t timeout)
{
	struct xferOp xfer;
	int res, seqCode;
	uint8_t status;

	res = nor_waitBusy(fspi, port, timeout);
	if (res < EOK) {
		return res;
	}

	seqCode = (enable ? fspi_writeEnable : fspi_writeDisable);

	xfer.op = xfer_opCommand;
	xfer.port = port;
	xfer.timeout = timeout;
	xfer.addr = 0;
	xfer.seqIdx = LUT_SEQIDX(seqCode);
	xfer.seqNum = LUT_SEQNUM(seqCode);

	res = flexspi_xferExec(fspi, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_readStatus(fspi, port, &status, timeout);
	if (res < EOK) {
		return res;
	}

	if (((status >> 1) & 1) != !!enable) {
		return -EACCES;
	}

	return EOK;
}


int nor_eraseSector(flexspi_t *fspi, uint8_t port, addr_t addr, time_t timeout)
{
	struct xferOp xfer;

	int res = nor_writeEnable(fspi, port, 1, timeout);
	if (res < EOK) {
		return res;
	}

	xfer.op = xfer_opCommand;
	xfer.port = port;
	xfer.timeout = timeout;
	xfer.addr = addr;
	xfer.seqIdx = LUT_SEQIDX(fspi_eraseSector);
	xfer.seqNum = LUT_SEQNUM(fspi_eraseSector);

	res = flexspi_xferExec(fspi, &xfer);
	if (res < EOK) {
		return res;
	}

	return nor_waitBusy(fspi, port, timeout);
}


int nor_eraseChip(flexspi_t *fspi, uint8_t port, time_t timeout)
{
	struct xferOp xfer;

	int res = nor_writeEnable(fspi, port, 1, timeout);
	if (res < EOK) {
		return res;
	}

	xfer.op = xfer_opCommand;
	xfer.port = port;
	xfer.timeout = timeout;
	xfer.addr = 0;
	xfer.seqIdx = LUT_SEQIDX(fspi_eraseChip);
	xfer.seqNum = LUT_SEQNUM(fspi_eraseChip);

	res = flexspi_xferExec(fspi, &xfer);
	if (res < EOK) {
		return res;
	}

	return nor_waitBusy(fspi, port, timeout);
}


int nor_pageProgram(flexspi_t *fspi, uint8_t port, addr_t dstAddr, const void *src, size_t pageSz, time_t timeout)
{
	struct xferOp xfer;

	int res = nor_writeEnable(fspi, port, 1, timeout);
	if (res < EOK) {
		return res;
	}

	xfer.op = xfer_opWrite;
	xfer.port = port;
	xfer.timeout = timeout;
	xfer.addr = dstAddr;
	xfer.seqIdx = LUT_SEQIDX(fspi_programQPP);
	xfer.seqNum = LUT_SEQNUM(fspi_programQPP);
	xfer.data.write.ptr = src;
	xfer.data.write.sz = pageSz;

	res = flexspi_xferExec(fspi, &xfer);

	if (res < EOK) {
		return res;
	}

	return nor_waitBusy(fspi, port, timeout);
}


ssize_t nor_readData(flexspi_t *fspi, uint8_t port, addr_t addr, void *data, size_t size, time_t timeout)
{
	struct xferOp xfer;

	xfer.op = xfer_opRead;
	xfer.port = port;
	xfer.timeout = timeout;
	xfer.addr = addr;
	xfer.seqIdx = LUT_SEQIDX(fspi_readData);
	xfer.seqNum = LUT_SEQNUM(fspi_readData);
	xfer.data.read.ptr = data;
	xfer.data.read.sz = size;

	return flexspi_xferExec(fspi, &xfer);
}


int nor_probe(flexspi_t *fspi, uint8_t port, const struct nor_info **pInfo, const char **pVendor)
{
	int res, i;
	uint32_t jedecId = 0;

	res = nor_readID(fspi, port, &jedecId, 1000);
	if (res < EOK) {
		return res;
	}

	for (res = -ENXIO, i = 0; i < sizeof(flashInfo) / sizeof(flashInfo[0]); ++i) {
		if (flashInfo[i].jedecId == jedecId) {
			res = EOK;

			if (pInfo != NULL) {
				*pInfo = &flashInfo[i];
			}

			break;
		}
	}

	if (res == EOK && pVendor != NULL) {
		*pVendor = "Unknown";

		for (i = 0; nor_vendors[i]; ++i) {
			if (*nor_vendors[i] == (jedecId & 0xff)) {
				*pVendor = &nor_vendors[i][2];
				break;
			}
		}
	}

	return res;
}
