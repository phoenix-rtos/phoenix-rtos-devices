/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 basic string funtions
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "string.h"

void *memcpy(void *dst, void *src, size_t len)
{
	size_t i;

	for (i = 0; i < len; ++i)
		((char *)dst)[i] = ((char *)src)[i];

	return dst;
}


void *memset(void *dst, int val, size_t len)
{
	size_t i;

	for (i = 0; i < len; ++i)
		((char *)dst)[i] = (char)val;

	return dst;
}
