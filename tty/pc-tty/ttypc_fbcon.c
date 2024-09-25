/*
 * Phoenix-RTOS
 *
 * VGA framebuffer console
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

/* Some of the code is based on Tfblib, thus we're retaining its copyright notice */
/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2018, Vladislav Valtchev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <string.h>

#include <sys/io.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/threads.h>

#include "ttypc.h"
#include "ttypc_fbfont.h"
#include <sys/platform.h>
#include <phoenix/arch/ia32/ia32.h>


typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} color_t;


/* clang-format off */
color_t colors[16] = {
	{ 0x00, 0x00, 0x00 }, /* black */
	{ 0x00, 0x00, 0xAA }, /* blue */
	{ 0x00, 0xAA, 0x00 }, /* green */
	{ 0x00, 0xAA, 0xAA }, /* cyan */
	{ 0xAA, 0x00, 0x00 }, /* red */
	{ 0xAA, 0x00, 0xAA }, /* magenta */
	{ 0xAA, 0x55, 0x00 }, /* brown */
	{ 0xAA, 0xAA, 0xAA }, /* light gray */
	{ 0x55, 0x55, 0x55 }, /* dark gray */
	{ 0x55, 0x55, 0xFF }, /* light blue */
	{ 0x55, 0xFF, 0x55 }, /* light green */
	{ 0x55, 0xFF, 0xFF }, /* light cyan */
	{ 0xFF, 0x55, 0x55 }, /* light red */
	{ 0xFF, 0x55, 0xFF }, /* light magenta */
	{ 0xFF, 0xFF, 0x00 }, /* yellow */
	{ 0xFF, 0xFF, 0xFF }, /* white */
};
/* clang-format on */


/* WARN: this macro (and fbcon implementation in general) assumes ARGB ordering,
 * some weird framebuffers may use something else. In such case it's necessary
 * to extend pctl_graphmode query with shifts/masks information for each color
 * from platformctl. */
#define COLOR_T_TO_RGB32(color) ((0xff << 24) | (color.red << 16) | (color.green << 8) | (color.blue << 0))


static void _draw_pixel(ttypc_vt_t *vt, uint16_t x, uint16_t y, uint32_t color)
{
	ttypc_t *ttypc = vt->ttypc;
	*(uint32_t *)((addr_t)ttypc->fbaddr + x * (ttypc->fbbpp / 8) + y * ttypc->fbpitch) = color;
}


void _ttypc_fbcon_drawchar(ttypc_vt_t *vt, int col, int row, uint16_t val)
{
	uint8_t i;
	uint16_t char_pix_y;

	uint8_t fg_color = (val >> 8) & 0xf, bg_color = (val >> 12) & 0xf;
	const uint32_t arr[] = { COLOR_T_TO_RGB32(colors[fg_color]), COLOR_T_TO_RGB32(colors[bg_color]) };

	char c = val & 0xff;
	uint8_t *data = fb_font + TTYPC_FBFONT_BYTES_PER_GLYPH * c;
	uint16_t x = col * TTYPC_FBFONT_W, y = row * TTYPC_FBFONT_H;

	if (vt->ttypc->vt != vt || vt->ttypc->fbaddr == NULL) {
		/* this vt is not a current vt or the fbcon is unavailable, don't draw anything */
		return;
	}

	for (char_pix_y = y; char_pix_y < (y + TTYPC_FBFONT_H); char_pix_y++) {
		for (i = 0; i < 8; i++) {
			_draw_pixel(vt, x + (7 - i), char_pix_y, arr[!(*data & (1 << i))]);
		}
		data += TTYPC_FBFONT_W_BYTES;
	}
}


int ttypc_fbcon_init(ttypc_t *ttypc)
{
	int err;

	platformctl_t pctl = { .action = pctl_get, .type = pctl_graphmode };

	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	if (pctl.graphmode.bpp != 32) {
		return -ENOSYS;
	}

	ttypc->fbmemsz = (pctl.graphmode.pitch * pctl.graphmode.height * (pctl.graphmode.bpp / 8) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);

	ttypc->fbaddr = mmap(NULL, ttypc->fbmemsz, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_SHARED | MAP_UNCACHED | MAP_ANONYMOUS | MAP_PHYSMEM, -1, pctl.graphmode.framebuffer);
	if (ttypc->fbaddr == MAP_FAILED) {
		return err;
	}

	ttypc->fbw = pctl.graphmode.width;
	ttypc->fbh = pctl.graphmode.height;
	ttypc->fbbpp = pctl.graphmode.bpp;
	ttypc->fbpitch = pctl.graphmode.pitch;

	/* TODO use fbcols, fbrows to operate in higher resolutions than 80x25 in tty/vt */
	ttypc->fbcols = ttypc->fbw / TTYPC_FBFONT_W;
	ttypc->fbrows = ttypc->fbh / TTYPC_FBFONT_H;

	return EOK;
}


void ttypc_fbcon_destroy(ttypc_t *ttypc)
{
	munmap((void *)ttypc->fbaddr, ttypc->fbmemsz);
}
