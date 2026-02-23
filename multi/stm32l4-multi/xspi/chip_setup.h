/*
 * Phoenix-RTOS
 *
 * External Flash parameters data structure
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _CHIP_SETUP_H_
#define _CHIP_SETUP_H_

#ifndef FLASHCS_FORMAT
#define FLASHCS_FORMAT "_flashcs_p%u"
#endif


#define CHIP_SETUP_VER_1 1

struct xspi_commandRegs_v1 {
	u32 ccr;
	u32 tcr;
	u32 ir;
} __attribute__((packed));


struct xspi_chipSetup_v1 {
	u32 version;
	char name[32];
	u8 jedecID[6];
	u8 log_chipSize;
	u8 log_eraseSize;
	u8 log_pageSize;
	u32 eraseTimeoutMs;
	u32 chipEraseTimeoutMs;
	struct xspi_commandRegs_v1 read;
	struct xspi_commandRegs_v1 write;
	struct xspi_commandRegs_v1 erase;
	struct xspi_commandRegs_v1 chipErase;
	struct xspi_commandRegs_v1 writeEnable;
	struct xspi_commandRegs_v1 writeDisable;
	struct xspi_commandRegs_v1 readStatus;
	u32 readStatus_addr;
	u8 readStatus_dataLen;
} __attribute__((packed));

#endif /* _CHIP_SETUP_H_ */
