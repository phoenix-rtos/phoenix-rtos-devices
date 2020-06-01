/*
 * Phoenix-RTOS
 *
 * Font header
 *
 * Copyright 2018 Phoenix Systems
 * Author: Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FONT_H_
#define _FONT_H_

enum {font_5x7};

typedef struct font_data_s {
	int char_width;
	int char_height;
	int char_num;
	char first_char;
	void* data;
} font_data_t;

extern font_data_t *font_common[];
#endif
