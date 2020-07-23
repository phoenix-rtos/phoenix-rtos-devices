/*
 * Phoenix-RTOS
 *
 * OLED graphic API
 *
 * Copyright 2018 Phoenix Systems
 * Author: Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _OLED_GRAPHIC_H_
#define _OLED_GRAPHIC_H_

#include <stdint.h>


void oledgraph_fillRect(int x, int y, int w, int h, int filled);


void oledgraph_fillBitmap(int x, int y, int w, int h, const uint64_t map[]);


void oledgraph_fillChar(int x, int y, int w, int h, int font, char c);


void oledgraph_drawStringAbs(int x, int y, int w, int h, int font, int size, const char* str);


void oledgraph_drawStringCont(int x, int y, int w, int h, int font, int size, const char* str);


void oledgraph_reset(int x, int y, int w, int h);


void oledgraph_drawBuffer(int x, int y, int w, int h, int force);

#endif
