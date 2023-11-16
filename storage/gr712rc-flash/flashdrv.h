/*
 * Phoenix-RTOS
 *
 * GR712RC Flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GR712RC_FLASH_H_
#define _GR712RC_FLASH_H_

#include <stdio.h>
#include <sys/types.h>
#include <storage/storage.h>

/* clang-format off */
#define LOG(fmt, ...) do { (void)fprintf(stdout, "gr712rc-flashsrv: " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...) do { (void)fprintf(stderr, "gr712rc-flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(fmt, ...) do { if (0) { (void)fprintf(stdout, "gr712rc-flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
/* clang-format on */

#define CFI_SIZE(size) (1u << (uint32_t)size)


typedef struct {
	uint8_t wordProgram;
	uint8_t bufWrite;
	uint8_t blkErase;
	uint8_t chipErase;
} __attribute__((packed)) flash_cfi_timeout_t;


typedef struct _flash_cfi_t {
	uint8_t vendorData[0x10];
	uint8_t qry[3];
	uint16_t cmdSet1;
	uint16_t addrExt1;
	uint16_t cmdSet2;
	uint16_t addrExt2;
	struct {
		uint8_t vccMin;
		uint8_t vccMax;
		uint8_t vppMin;
		uint8_t vppMax;
	} __attribute__((packed)) voltages;
	flash_cfi_timeout_t toutTypical;
	flash_cfi_timeout_t toutMax;
	uint8_t chipSz;
	uint16_t fdiDesc;
	uint16_t bufSz;
	uint8_t regionCnt;
	struct {
		uint16_t count;
		uint16_t size;
	} __attribute__((packed)) regions[4];
} __attribute__((packed)) flash_cfi_t;


struct _storage_devCtx_t {
	flash_cfi_t cfi;

	handle_t lock;
	void *ftmctrl;
	size_t blockSz;
};


const storage_mtdops_t *flashdrv_getMtdOps(void);


struct _storage_devCtx_t *flashdrv_contextInit(void);


void flashdrv_contextDestroy(struct _storage_devCtx_t *ctx);


#endif
