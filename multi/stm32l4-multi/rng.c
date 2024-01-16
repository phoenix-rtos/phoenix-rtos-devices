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

#include <board_config.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <phoenix/arch/stm32l4.h>

#include "common.h"

#define RNG_CR 0
#define RNG_SR 1
#define RNG_DR 2


static struct {
	volatile uint32_t *base;
	handle_t lock;
} rng_common;


static int rng_readDR(uint32_t *val)
{
	int ret = -EAGAIN;
	uint32_t sr = *(rng_common.base + RNG_SR);

	if ((sr & 1u) != 0) {
		*val = *(rng_common.base + RNG_DR);

		/* Zero check is recommended in Reference Manual due to rare race condition */
		ret = (*val != 0) ? 0 : -EIO;
	}

	/* Has error been detected? */
	if ((sr & (3u << 5)) != 0) {
		/* Mark data as faulty */
		/* Request IP reinit only for SE */
		ret = (sr & (1 << 6)) ? -EIO : -EAGAIN;

		/* Clear flags */
		*(rng_common.base + RNG_SR) &= ~(sr & (3u << 5));
		dataBarier();
	}

	return ret;
}


static void rng_enable(void)
{
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_hsi48,
			.state = 1 }
	};

	/* Enable HSI48 */
	platformctl(&pctl);

	/* Enable RNG */
	*(rng_common.base + RNG_CR) |= 1u << 2;
}


static void rng_disable(void)
{
	/* Disable RNG */
	*(rng_common.base + RNG_CR) &= ~(1u << 2);

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_hsi48,
			.state = 0 }
	};

	/* Disable HSI48 */
	platformctl(&pctl);
}


ssize_t rng_read(uint8_t *val, size_t len)
{
	mutexLock(rng_common.lock);

	rng_enable();

	size_t pos = 0;
	int retry = 0;
	while (pos < len) {
		uint32_t t;
		int err = rng_readDR(&t);
		if (err < 0) {
			if (err != -EAGAIN) {
				/* Reinitialize IP */
				rng_disable();
				rng_enable();
			}

			++retry;
			if (retry < 1000) {
				/* Retry */
				continue;
			}

			rng_disable();
			mutexUnlock(rng_common.lock);
			return -EIO;
		}

		retry = 0;
		size_t chunk = ((len - pos) > sizeof(t)) ? sizeof(t) : len - pos;
		memcpy(val + pos, &t, chunk);
		pos += chunk;
	}

	rng_disable();

	mutexUnlock(rng_common.lock);

	return (ssize_t)len;
}


int rng_init(void)
{
	platformctl_t pctl = {
		.type = pctl_devclk,
		.action = pctl_set,
		.devclk = {
			.dev = pctl_rng,
			.state = 1 }
	};

	/* Enable clock */
	platformctl(&pctl);

	rng_common.base = (void *)0x50060800;

	if (mutexCreate(&rng_common.lock) < 0) {
		return -1;
	}

	/* Disable RNG */
	*(rng_common.base + RNG_CR) &= ~(1u << 2);
	dataBarier();

	/* Enable clock error detection */
	*(rng_common.base + RNG_CR) &= ~(1u << 5);

	/* Disable interrupt */
	*(rng_common.base + RNG_CR) &= ~(1u << 3);

	return 0;
}
