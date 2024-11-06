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

#include <flashdrv/common.h>


off_t common_getSectorOffset(size_t sectorsz, off_t offs)
{
	return offs & ~(sectorsz - 1);
}


bool common_isValidAddress(size_t memsz, off_t offs, size_t len)
{
	return ((offs < memsz) && ((offs + len) <= memsz));
}
