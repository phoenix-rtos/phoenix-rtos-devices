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
#include <errno.h>

#include <imx6ull-ecspi.h>

#include "../mcp331-low.h"
#include "../log.h"

/* TODO: Improve interfacing with peripherals on imx6ull platform */

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
