/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * i.MX RT NOR flash device driver
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLEXSPI_NOR_H_
#define _FLEXSPI_NOR_H_

#define NOR_ERASED_STATE    0xff
#define NOR_DEFAULT_TIMEOUT 10000
#define NOR_SECTORSZ_MAX    0x1000
#define NOR_PAGESZ_MAX      0x100


struct nor_device {
	const struct nor_info *nor;
	flexspi_t fspi;
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
};


enum { fspi_readData,
	fspi_readStatus,
	fspi_writeStatus,
	fspi_writeEnable,
	fspi_writeDisable,
	fspi_eraseSector,
	fspi_eraseBlock,
	fspi_eraseChip,
	fspi_programQPP,
	fspi_readID,
};


extern void nor_deviceInit(struct nor_device *dev, int port, int active, time_t timeout);


extern int nor_readStatus(flexspi_t *fspi, uint8_t port, uint8_t *statusByte, time_t timeout);


extern int nor_waitBusy(flexspi_t *fspi, uint8_t port, time_t timeout);


extern int nor_writeEnable(flexspi_t *fspi, uint8_t port, int enable, time_t timeout);


extern int nor_eraseSector(flexspi_t *fspi, uint8_t port, addr_t addr, time_t timeout);


extern int nor_eraseChip(flexspi_t *fspi, uint8_t port, time_t timeout);


extern int nor_pageProgram(flexspi_t *fspi, uint8_t port, addr_t dstAddr, const void *src, size_t pageSz, time_t timeout);


extern ssize_t nor_readData(flexspi_t *fspi, uint8_t port, addr_t addr, void *data, size_t size, time_t timeout);


extern int nor_probe(flexspi_t *fspi, uint8_t port, const struct nor_info **pInfo, const char **vendor);


#endif /* _FLEXSPI_NOR_H_ */
