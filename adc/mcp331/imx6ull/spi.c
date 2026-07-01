/*
 * Phoenix-RTOS
 *
 * SPI API for mcp331 adc on imx6ull platform
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <imx6ull-ecspi.h>

#include "../mcp331-low.h"
#include "../log.h"

/* TODO: Improve interfacing with peripherals on imx6ull platform */

#define READ_SIZE 2

typedef struct {
	ecspi_ctx_t ctx;
	uint8_t outBuff[READ_SIZE];
	uint8_t inBuff[READ_SIZE];
	spi_periodicCallback_t callback;
} periodicCtx_t;

periodicCtx_t periodicCtx = { 0 };


static int periodicHandler(const uint8_t *rx, size_t len, uint8_t *out)
{
	if (periodicCtx.callback != NULL) {
		if (periodicCtx.callback(rx, len) < 0) {
			return -1;
		}
	}
	else {
		return -1;
	}

	return len;
}


int spi_exchange(int devNo, int chan, uint8_t *buff, uint32_t len)
{
	int ret = ecspi_setChannel(devNo, chan);
	if (ret < 0) {
		if (ret == -1 || ret == -2) {
			return -ENODEV;
		}
		else if (ret == -3) {
			return -EPERM;
		}
		else {
			return -EINVAL;
		}
	}

	if (ecspi_exchangeBusy(devNo, buff, buff, len) < 0) {
		return -ENODEV;
	}

	return len;
}


int spi_init(int devNo, int chan)
{
	if (ecspi_init(devNo, 1 << chan) < 0) {
		log_error("Failed to initialize ecspi");
		return -ENODEV;
	}

	/* pre == 9 => divide input clock by 9, post == 0 => divide clock by 1.
	 * Resulting clock: 5 MHz */
	ecspi_setClockDiv(devNo, 9, 0);

	/* Set CS<=>CLK delay to 43 cycles == 8600 ns.
	 * SPI isolator enable signal is driven by inverted CS signal.
	 * ISO7741 requires up to 8.5 us from enable high to output low (tPZL).
	 * MCP331X1 requires additional 10 ns (negligible compared to isolator delay). */
	ecspi_setCSDelay(devNo, 43);

	/* Set CS deasserted <=> CS asserted delay to 7 cycles == 1400 ns.
	 * On MCP331X1-05 this is the maximum conversion time, so this is a safe value. */
	ecspi_setSSDelay(devNo, 7);

	return 0;
}


int spi_initPeriodic(int devNo, handle_t cond, spi_periodicCallback_t callback)
{
	periodicCtx.callback = callback;

	/* Initialize output buffer with zeros (dummy data for ADC read) */
	periodicCtx.outBuff[0] = 0;
	periodicCtx.outBuff[1] = 0;

	return ecspi_registerContext(devNo, &periodicCtx.ctx, cond);
}


int spi_startPeriodic(int devNo, int chan, uint16_t waitStates)
{
	int ret = 0;

	ret = ecspi_setChannel(devNo, chan);
	if (ret < 0) {
		if (ret == -1 || ret == -2) {
			return -ENODEV;
		}
		else if (ret == -3) {
			return -EPERM;
		}
		else {
			return -EINVAL;
		}
	}

	int retries = 10;
	do {
		ret = ecspi_exchangePeriodically(&periodicCtx.ctx, periodicCtx.outBuff,
				periodicCtx.inBuff, READ_SIZE, waitStates, periodicHandler);
		if (ret < 0) {
			return -EIO;
		}
		usleep(10000);
	} while (ret == 0 && --retries > 0);

	if (ret == 0) {
		return -EIO;
	}

	return 0;
}
