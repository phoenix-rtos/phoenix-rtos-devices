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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include "oled-phy.h"
#include "oled-graphic.h"
#include "fonts/font.h"


#define BYTE_ELEM(buff, row, col) ((buff[col] >> (8 * row)) & 0xff)


#define CURSOR_COST 2

static void handle_control(int x, int y, int w, int h, int* cur_x, int* cur_y, int font, char c);


static void handle_character(int x, int y, int w, int h, int* cur_x, int* cur_y, int font, char c);


struct {
	uint64_t dev_buffer[128];
	uint64_t buffer[128];
	struct {
		int cur_x;
		int cur_y;
	} scroll;
} g_common;


void oledgraph_fillRect(int x, int y, int w, int h, int filled)
{
	int i;
	const uint64_t mask = (((uint64_t)-1) << y) &~ (((uint64_t)-1) << (y + h));

	if (filled) {
		for (i = x; i < x + w; ++i)
			g_common.buffer[i] |=  mask;
	}
	else {
		for (i = x; i < x + w; ++i)
			g_common.buffer[i] &= ~mask;
	}
}


void oledgraph_fillBitmap(int x, int y, int w, int h, const uint64_t map[w])
{
	int i;
	const uint64_t mask = (((uint64_t)-1) << y) &~ (((uint64_t)-1) << (y + h));
	for (i = 0; i < w; ++i) {
		g_common.buffer[i + x] &= ~mask;
		g_common.buffer[i + x] |= (map[i] << y);
	}
}


void oledgraph_fillChar(int x, int y, int w, int h, int font, char c)
{
	uint64_t* font_ptr = ((uint64_t(*)[5]) font_common[font]->data)[c - font_common[font]->first_char];
	oledgraph_fillBitmap(x, y, w, h, font_ptr);
}


static void handle_control(int x, int y, int w, int h, int* cur_x, int* cur_y, int font, char c)
{
	int i;
	switch(c) {
		case '\n':
			*cur_x = x;
			*cur_y += font_common[font]->char_height + 1;
			break;
		case '\t':
			for (i = 0; i < 4; ++i)
				handle_character(x, y, w, h, cur_x, cur_y, font, ' ');
			break;
	}
}

static void buffer_move_updown(int x, int y, int w, int h, int move)
{
	int i;
	uint64_t tmp;
	const uint64_t mask = (((uint64_t)-1) << y) &~ (((uint64_t)-1) << (y + h));
	for(i = x; i < x + w; ++i) {
		tmp = g_common.buffer[i];
		g_common.buffer[i] &= ~mask;
		g_common.buffer[i] |= (tmp >> move) & mask;
	}
}


static void handle_character(int x, int y, int w, int h, int* cur_x, int* cur_y, int font, char c)
{
	if (c - font_common[font]->first_char > font_common[font]->char_num)
		c = '?';
	oledgraph_fillChar(*cur_x, *cur_y, font_common[font]->char_width, font_common[font]->char_height, font, c);
	*cur_x += font_common[font]->char_width;
	if (x + w > *cur_x + font_common[font]->char_width + 1)
		*cur_x += 1;
	else
		handle_control(x, y, w, h, cur_x, cur_y, font, '\n');
}


void oledgraph_drawStringAbs(int x, int y, int w, int h, int font, int size, const char* str)
{
	int i = 0;
	int cur_x = x, cur_y = y;
	char c;

	oledgraph_fillRect(x, y, w, h, 0);
	while((c = *str++) != '\0' && i++ <= size) {
		if (c >= ' ')
			handle_character(x ,y, w, h, &cur_x, &cur_y, font, c);
		else
			handle_control(x, y, w, h, &cur_x, &cur_y, font, c);
	}
}


void oledgraph_drawStringCont(int x, int y, int w, int h, int font, int size, const char* str)
{
	int i = 0;
	int min_x, min_y, min_w, min_h;
	int scrolled = 0;
	int cur_x = g_common.scroll.cur_x;
	int cur_y = g_common.scroll.cur_y;
	char c;

	while((c = *str++) != '\0' && i++ <= size) {
		if (cur_x == x)
			oledgraph_fillRect(cur_x, cur_y, w, font_common[font]->char_height, 0);

		if (c >= ' ')
			handle_character(x ,y, w, h, &cur_x, &cur_y, font, c);
		else
			handle_control(x, y, w, h, &cur_x, &cur_y, font, c);

		if (y + h < cur_y + font_common[font]->char_height) {
			buffer_move_updown(x, y, w, h, font_common[font]->char_height + 1);
			scrolled = 1;
			cur_y = y + h - ( font_common[font]->char_height + 1);
		}
	}

	if (!scrolled) {
		min_y = g_common.scroll.cur_y;
		min_h = cur_y - g_common.scroll.cur_y + font_common[font]->char_height;
		if (cur_y > min_y) {
			min_x = x;
			min_w = w;
		}
		else {
			min_x = g_common.scroll.cur_x;
			min_w = cur_x - g_common.scroll.cur_x;
		}
	}
	else {
		min_x = x;
		min_y = y;
		min_w = w;
		min_h = h;
	}

	g_common.scroll.cur_x = cur_x;
	g_common.scroll.cur_y = cur_y;
	oledgraph_drawBuffer(min_x, min_y, min_w, min_h, 0);
}


void oledgraph_reset(int x, int y, int w, int h)
{
	g_common.scroll.cur_x = x;
	g_common.scroll.cur_y = y;
	oledgraph_fillRect(x, y, w, h, 0);
	oledgraph_drawBuffer(x, y, w, h, 1);
}


static int findRowStart(int row, int x, int endx)
{
	int col;
	for (col = x; col < endx; ++col) {
		if (BYTE_ELEM(g_common.buffer, row, col) !=
			BYTE_ELEM(g_common.dev_buffer, row, col)) {
			return col;
		}
	}
	return -1;
}

static int findRowEnd(int row, int x, int endx)
{
	int col, i;

	for (col = x; col < endx; ++col) {
		if (BYTE_ELEM(g_common.buffer, row, col) ==
			BYTE_ELEM(g_common.dev_buffer, row, col)) {
			for (i = 0; i + col < endx && i < CURSOR_COST; ++i) {
				if (BYTE_ELEM(g_common.buffer, row, i + col) !=
					BYTE_ELEM(g_common.dev_buffer, row, i + col)) {
					break;
				}
			}

			if (i >= CURSOR_COST)
				return col;
			else
				col += i;
		}
	}

	return endx - 1;
}


static void drawBufferRow(int row, int x, int w)
{
	int col = x;
	int i;
	int startx;
	while (col < x + w) {
		startx = findRowStart(row, col, x + w);
		if (startx < 0)
			return;

		col = findRowEnd(row, startx + 1, x + w);

		oledphy_sendCmd(startx & 0xf);
		oledphy_sendCmd(((startx >> 4) & 0xf) | 0x10);
		for (i = startx; i < col; ++i)
			oledphy_sendData(BYTE_ELEM(g_common.buffer, row, i));
	}
}


void oledgraph_drawBuffer(int x, int y, int w, int h, int force)
{
	int i, j;
	const int start_page = y / 8;
	const int end_page = (y + h - 1) / 8;

	/* Page addressing */
	oledphy_sendCmd(0x20);
	oledphy_sendCmd(2);

	for (i = start_page; i <= end_page; ++i) {
		oledphy_sendCmd(i | 0xb0);
		if (!force) {
			drawBufferRow(i, x, w);
		}
		else {
			oledphy_sendCmd(x & 0xf);
			oledphy_sendCmd(((x >> 4) & 0xf) | 0x10);
			for (j = x; j < x + w; ++j)
				oledphy_sendData(BYTE_ELEM(g_common.buffer, i, j));
		}
	}

	memcpy(g_common.dev_buffer, g_common.buffer, sizeof(g_common.dev_buffer));
}
