/*
 * Phoenix-RTOS
 *
 * Oled driver message API
 *
 * Copyright 2020 Phoenix Systems
 * Author: Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _OLED_API_H_
#define _OLED_API_H_

#include <stdint.h>

#define OLED_DRIVER "/dev/oled"
#define OLED_X_RES 128
#define OLED_Y_RES 64

typedef enum {
	oled_write__rect,
	oled_write__bitmap,
	oled_write__text_abs,
	oled_write__text_cont,
	oled_write__clear,
	oled_write__draw
} oled_write_type_t;

typedef struct {
	oled_write_type_t type;

	uint8_t x;
	uint8_t y;
	uint8_t w;
	uint8_t h;

	union {
		const char *text;
		uint8_t filled;
		const uint64_t *data;
	};
} oled_write_t;

#endif