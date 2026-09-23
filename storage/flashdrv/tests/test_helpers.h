/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash server helpers
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TEST_HELPERS_H_
#define _TEST_HELPERS_H_

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>

#include <sys/msg.h>
#include <sys/file.h>
#include <posix/utils.h>
#include <ptable.h>
#include <storage/storage.h>
#include <flashdrv/flashsrv.h>
#include <board_config.h>
#include <time.h>

#include "tests.h"


// uint64_t get_time_us(void);


int sendOpenCloseMsg(oid_t oid, int type);


int writeToFlash(oid_t oid, off_t offs, const void *data, size_t size);


int eraseSector(oid_t oid, uint32_t offs, size_t size);


int erasePartition(oid_t oid);


int eraseChip(oid_t oid);


int setSPI(oid_t oid, SPIMode_t mode);


int readFromFlash(oid_t oid, off_t offs, void *data, size_t size);


int getAttrFlash(oid_t oid, int type, long long *val);


int erase_write_read_print(oid_t oid, const off_t testAddr, const size_t testSize, const size_t sectorSize, 
                        const uint8_t checkPattern, const char* part_path);

#endif /* _TEST_HELPERS_H_ */