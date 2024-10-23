/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Common auxiliary functions
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _COMMON_H_
#define _COMMON_H_


#include <storage/storage.h>


static inline off_t common_getSectorOffset(const storage_t *strg, off_t offs)
{
	return offs & ~(strg->dev->mtd->erasesz - 1);
}


static int common_isValidAddress(size_t memsz, off_t offs, size_t len)
{
	if ((offs < memsz) && ((offs + len) <= memsz)) {
		return 1;
	}

	return 0;
}


#endif
