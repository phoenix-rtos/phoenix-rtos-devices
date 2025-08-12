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

#include "common.h"

#define RNG_CR   0
#define RNG_SR   1
#define RNG_DR   2
#define RNG_NSCR 3
#define RNG_HTCR 4


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
		/* Mark data as faulty, but check cause of interrupts */
		if ((sr & (3u << 1)) == 0) {
			/* Situation has been recovered from, try again */
			ret = -EAGAIN;
		}
		else {
			/* Request IP reinit only for SE */
			ret = (sr & (1 << 6)) ? -EIO : -EAGAIN;
		}

		/* Clear flags */
		*(rng_common.base + RNG_SR) &= ~(sr & (3u << 5));
		dataBarier();
	}

	return ret;
}


static void rng_enable(void)
{
#if defined(__CPU_STM32L4X6)
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_hsi48,
			.state = 1 }
	};

	/* Enable HSI48 */
	platformctl(&pctl);
#elif defined(__CPU_STM32N6)
	/* On STM32N6 RNG uses HSIS clock that is always enabled in Run mode */
#endif

	/* Enable RNG */
	*(rng_common.base + RNG_CR) |= 1u << 2;
}


static void rng_disable(void)
{
	/* Disable RNG */
	*(rng_common.base + RNG_CR) &= ~(1u << 2);

#if defined(__CPU_STM32L4X6)
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_hsi48,
			.state = 0 }
	};

	/* Disable HSI48 */
	platformctl(&pctl);
#elif defined(__CPU_STM32N6)
	/* On STM32N6 RNG uses HSIS clock that cannot be stopped by software */
#endif
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

	rng_common.base = RNG_BASE;

	if (mutexCreate(&rng_common.lock) < 0) {
		return -1;
	}

	/* Disable RNG */
	*(rng_common.base + RNG_CR) &= ~(1u << 2);
	dataBarier();

#if defined(__CPU_STM32L4X6)
	/* Enable clock error detection */
	*(rng_common.base + RNG_CR) &= ~(1u << 5);
#elif defined(__CPU_STM32N6)
	uint32_t v = *(rng_common.base + RNG_CR);
	v |= (1u << 30); /* Conditioning soft reset */
	v &= ~(1u << 7); /* Enable auto reset if seed error detected */
	v &= ~(1u << 5); /* Enable clock error detection */
	/* AN4230 5.1.3 NIST compliant RNG configuration */
	v &= ~0x03FFFF00;
	v |= 0x00F00D00;
	*(rng_common.base + RNG_CR) = v; /* Apply new configuration, hold in reset */
	/* AN4230 5.1.3 NIST compliant RNG configuration */
	*(rng_common.base + RNG_HTCR) = 0xAAC7;
	*(rng_common.base + RNG_NSCR) = 0x0003FFFF; /* Activate all noise sources */
	*(rng_common.base + RNG_CR) &= ~(1u << 30); /* Release from reset */
	while ((*(rng_common.base + RNG_CR) & (1u << 30)) != 0) {
		/* Wait for peripheral to become ready (2 AHB cycles + 2 RNG clock cycles) */
	}
#endif

	/* Disable interrupt */
	*(rng_common.base + RNG_CR) &= ~(1u << 3);

	return 0;
}
