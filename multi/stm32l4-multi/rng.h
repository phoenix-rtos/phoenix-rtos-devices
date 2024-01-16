/*
 * Phoenix-RTOS
 *
 * STM32L4 RNG driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef MULTI_RNG_H_
#define MULTI_RNG_H_

#include <stdint.h>
#include <sys/types.h>

ssize_t rng_read(uint8_t *val, size_t len);


int rng_init(void);

#endif
