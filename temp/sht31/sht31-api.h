/*
 * Phoenix-RTOS
 *
 * libphoenix
 *
 * bignum.h
 *
 * Copyright 2023 Phoenix Systems
 * Author: Andrzej Stalke
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _SHT31_H
#define _SHT31_H


enum {
	sht31_freq0_5Hz = 0,
	sht31_freq1Hz,
	sht31_freq2Hz,
	sht31_freq4Hz,
	sht31_freq10Hz
};


enum {
	sht31_repeatabilityHigh = 0,
	sht31_repeatabilityMed,
	sht31_repeatabilityLow
};


enum {
	sht31_cmdART = 0x2b32,
	sht31_cmdBreak = 0x3093,
	sht31_cmdSoftReset = 0x30a2,
	sht31_cmdHeaterOn = 0x306d,
	sht31_cmdHeaterOff = 0x3066,
	sht31_cmdClearStatus = 0x3041,

	sht31_cmdGetStatus = 0xf32d,
	sht31_cmdFetchData = 0xe000,
};

enum {
	sht31_typeMeasConfig,
	sht31_typeCmd,
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
} __attribute__((packed)) sht31_i_devctl_t;


#endif
