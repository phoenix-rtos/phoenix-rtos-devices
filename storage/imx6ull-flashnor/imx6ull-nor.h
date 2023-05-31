/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL flash device driver
 *
 * Copyright 2021-2023 Phoenix Systems
 * Author: Gerard Swiderski, Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _QSPI_NOR_H_
#define _QSPI_NOR_H_

#include <imx6ull-qspi.h>

#define NOR_ERASED_STATE    0xff
#define NOR_DEFAULT_TIMEOUT 10000
#define NOR_SECTORSZ_MAX    0x1000
#define NOR_PAGESZ_MAX      0x100


struct nor_device {
	const struct nor_info *nor;
	qspi_t qspi;
	int port;
	int active;
	time_t timeout;

	addr_t sectorPrevAddr;
	addr_t sectorSyncAddr;
	uint8_t sectorBuf[NOR_SECTORSZ_MAX];
};


struct nor_info {
	uint32_t jedecId;
	const char *name;
	size_t totalSz;
	size_t pageSz;
	size_t sectorSz;
	size_t blockSz;
};


enum { qspi_readData,
	qspi_readStatus,
	qspi_writeStatus,
	qspi_writeEnable,
	qspi_writeDisable,
	qspi_eraseSector,
	qspi_eraseBlock,
	qspi_eraseChip,
	qspi_programQPP,
	qspi_readID,
};


extern void nor_deviceInit(struct nor_device *dev, int port, int active, time_t timeout);


extern int nor_readStatus(qspi_t *qspi, uint8_t port, uint8_t *statusByte, time_t timeout);


extern int nor_waitBusy(qspi_t *qspi, uint8_t port, time_t timeout);


extern int nor_writeEnable(qspi_t *qspi, uint8_t port, int enable, time_t timeout);


extern int nor_eraseSector(qspi_t *qspi, uint8_t port, addr_t addr, time_t timeout);


extern int nor_eraseBlock(qspi_t *qspi, uint8_t port, addr_t addr, time_t timeout);


extern int nor_eraseChip(qspi_t *qspi, uint8_t port, time_t timeout);


extern int nor_pageProgram(qspi_t *qspi, uint8_t port, addr_t dstAddr, const void *src, size_t pageSz, time_t timeout);


extern ssize_t nor_readData(qspi_t *qspi, uint8_t port, addr_t addr, void *data, size_t size, time_t timeout);


extern int nor_probe(qspi_t *qspi, uint8_t port, const struct nor_info **pInfo, const char **vendor);


#endif /* _QSPI_NOR_H_ */
