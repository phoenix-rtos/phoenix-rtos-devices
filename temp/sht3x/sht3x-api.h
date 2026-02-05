/*
 * Phoenix-RTOS
 *
 * SHT3x public API
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SHT3x_API_H
#define _SHT3x_API_H

#include <stdint.h>

enum {
	sht3x_freq0_5Hz = 0,
	sht3x_freq1Hz,
	sht3x_freq2Hz,
	sht3x_freq4Hz,
	sht3x_freq10Hz,

	sht3x_freqCnt
};


enum {
	sht3x_repeatabilityHigh = 0,
	sht3x_repeatabilityMed,
	sht3x_repeatabilityLow,

	sht3x_repeatabilityCnt
};


enum {
	sht3x_cmdART = 0x2b32,
	sht3x_cmdBreak = 0x3093,
	sht3x_cmdSoftReset = 0x30a2,
	sht3x_cmdHeaterOn = 0x306d,
	sht3x_cmdHeaterOff = 0x3066,
	sht3x_cmdClearStatus = 0x3041,

	sht3x_cmdGetStatus = 0xf32d,
	sht3x_cmdFetchData = 0xe000,
};


enum {
	sht3x_typeMeasConfig,
	sht3x_typeCmd,
};


typedef struct {
	int type;

	union {
		/* Periodic data acquisition configuration */
		struct {
			uint8_t freq;
			uint8_t repeatability;
		} measConfig;

		/* Direct command value */
		uint16_t cmd;
	};
} __attribute__((packed)) sht3x_i_devctl_t;


#endif
