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

#ifndef M4STRING_H_
#define M4STRING_H_

#include <stddef.h>

void *memcpy(void *dst, void *src, size_t len);


void *memset(void *dst, int val, size_t len);


#endif
