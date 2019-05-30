/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash server.
 *
 * Copyright 2018 - 2019 Phoenix Systems
 * Author: Jan Sikorski, Hubert Buczyński
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_FLASHSRV_H_
#define _IMX6ULL_FLASHSRV_H_

#define PAGES_PER_BLOCK 64
#define FLASH_PAGE_SIZE 0x1000
#define RAW_FLASH_PAGE_SIZE 4320
#define BLOCKS_CNT 4096

#define ERASE_BLOCK_SIZE (FLASH_PAGE_SIZE * PAGES_PER_BLOCK)
#define ROOT_ID -1

enum { flashsrv_devctl_erase = 0, flashsrv_devctl_chiperase, flashsrv_devctl_writeraw, flashsrv_devctl_writemeta,
	 flashsrv_devctl_readraw };

typedef struct {
	int type;

	union {
		struct {
			size_t size;
			size_t offset;
			oid_t oid;
		} erase;

		struct {
			size_t size;
			size_t offset;
		} chiperase;

		struct {
			oid_t oid;
			u32 address;
			size_t size;
		} write;

		struct {
			oid_t oid;
			u32 address;
			size_t size;
		} readraw;
	};
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	int err;
} __attribute__((packed)) flash_o_devctl_t;

#endif
