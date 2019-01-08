/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash driver.
 *
 * Copyright 2019 Phoenix Systems
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_FLASH_H_
#define _IMX6ULL_FLASH_H_

#define PAGES_PER_BLOCK 64
#define FLASH_PAGE_SIZE 0x1000

enum { nand_erase, nand_flash };

typedef struct {
	int type;

	union {
		struct {
			unsigned start;
			unsigned end;
		} erase;

		struct {
			unsigned offset;
			unsigned end;
		} flash;
	};
} nand_devctl_t;

#endif
