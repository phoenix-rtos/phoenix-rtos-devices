/* 
 * Phoenix-RTOS
 *
 * VGA display
 *
 * Copyright 2008 Pawel Pisarczyk
 * Copyright 2017, 2012, 2019, 2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <sys/io.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/threads.h>

#include "ttypc.h"
#include "ttypc_vga.h"


/* Reads from CRTC register */
static uint16_t ttypc_vga_readreg(ttypc_t *ttypc, uint8_t reg)
{
	uint16_t val;

	outb(ttypc->crtc, reg);
	val = (uint16_t)inb(ttypc->crtc + 1) << 8;
	outb(ttypc->crtc, reg + 1);
	val |= inb(ttypc->crtc + 1);

	return val;
}


/* Writes to CRTC register */
static void ttypc_vga_writereg(ttypc_t *ttypc, uint8_t reg, uint16_t val)
{
	outb(ttypc->crtc, reg);
	outb(ttypc->crtc + 1, (val >> 8) & 0xff);
	outb(ttypc->crtc, reg + 1);
	outb(ttypc->crtc + 1, (val >> 0) & 0xff);
}


/* Sets cursor type */
static void _ttypc_vga_setcurt(ttypc_t *ttypc, uint8_t from, uint8_t to)
{
	uint16_t ctype = ttypc_vga_readreg(ttypc, CRTC_CURSTART);

	ctype = (ctype & 0xc0e0) | ((uint16_t)from << 8 | to);
	ttypc_vga_writereg(ttypc, CRTC_CURSTART, ctype);
}


void _ttypc_vga_switch(ttypc_vt_t *vt)
{
	ttypc_t *ttypc = vt->ttypc;
	ttypc_vt_t *cvt = ttypc->vt;

	if (vt == cvt)
		return;

	/* VGA memory -> VT memory */
	memcpy(cvt->mem, cvt->vram, cvt->rows * cvt->cols * CHR_VGA);
	cvt->vram = vt->mem;

	/* VT memory -> VGA memory */
	mutexLock(vt->lock);
	vt->vram = ttypc->vga;
	memcpy(vt->vram, vt->mem, vt->rows * vt->cols * CHR_VGA);
	mutexUnlock(vt->lock);

	/* Set active VT */
	ttypc->vt = vt;
	_ttypc_vga_setcursor(ttypc->vt);
}


void _ttypc_vga_rollup(ttypc_vt_t *vt, unsigned int n)
{
	/* Update scrollback buffer */
	if (vt->scrbsz + n > vt->rows * SCRB_PAGES) {
		/* Make space for n new lines */
		memmove(vt->scrb, vt->scrb + (vt->rows * SCRB_PAGES + 1 - vt->scrbsz) * vt->cols, (2 * vt->scrbsz - vt->rows * SCRB_PAGES - 1) * vt->cols * CHR_VGA);
		vt->scrbsz = vt->rows * SCRB_PAGES;
	}
	else {
		vt->scrbsz += n;
	}
	memcpy(vt->scrb + (vt->scrbsz - n) * vt->cols, vt->vram + vt->top * vt->cols, n * vt->cols * CHR_VGA);

	/* Roll up */
	memmove(vt->vram + vt->top * vt->cols, vt->vram + (vt->top + n) * vt->cols, (vt->bottom - vt->top - n + 1) * vt->cols * CHR_VGA);
	memsetw(vt->vram + (vt->bottom - n + 1) * vt->cols, vt->attr | ' ', n * vt->cols);
}


void _ttypc_vga_rolldown(ttypc_vt_t *vt, unsigned int n)
{
	unsigned int k;

	/* Roll down */
	if (vt->bottom > vt->crow) {
		k = min(n, vt->bottom - vt->crow);
		memmove(vt->vram + (vt->top + k) * vt->cols, vt->vram + vt->top * vt->cols, (vt->bottom - vt->top - k + 1) * vt->cols * CHR_VGA);
		memsetw(vt->vram + vt->top * vt->cols, vt->attr | ' ', k * vt->cols);

		if (n == k)
			return;
		n -= k;
	}

	/* Update scrollback buffer */
	if (vt->scrbsz + n > vt->rows * SCRB_PAGES) {
		/* Make space for n new lines */
		memmove(vt->scrb, vt->scrb + (vt->rows * SCRB_PAGES + 1 - vt->scrbsz) * vt->cols, (2 * vt->scrbsz - vt->rows * SCRB_PAGES - 1) * vt->cols * CHR_VGA);
		vt->scrbsz = vt->rows * SCRB_PAGES;
	}
	else {
		vt->scrbsz += n;
	}
	memsetw(vt->scrb + (vt->scrbsz - n) * vt->cols, vt->attr | ' ', n * vt->cols);
}


void _ttypc_vga_scrollup(ttypc_vt_t *vt, unsigned int n)
{
	/* Clip n */
	if (n > vt->scrbsz - vt->scrbpos) {
		if (!(vt->scrbsz - vt->scrbpos))
			return;
		n = vt->scrbsz - vt->scrbpos;
	}

	if (!n)
		return;

	/* Save scroll origin */
	if (!vt->scrbpos)
		memcpy(vt->scro, vt->vram, vt->rows * vt->cols * CHR_VGA);

	/* Copy scrollback */
	vt->scrbpos += n;
	memcpy(vt->vram, vt->scrb + (vt->scrbsz - vt->scrbpos) * vt->cols, min(vt->scrbpos, vt->rows) * vt->cols * CHR_VGA);

	/* Copy scroll origin */
	if (vt->scrbpos < vt->rows)
		memcpy(vt->vram + vt->scrbpos * vt->cols, vt->scro, (vt->rows - vt->scrbpos) * vt->cols * CHR_VGA);

	/* Update cursor */
	vt->cpos += n * vt->cols;
	vt->crow += n;

	if (vt == vt->ttypc->vt)
		_ttypc_vga_setcursor(vt);
}


void _ttypc_vga_scrolldown(ttypc_vt_t *vt, unsigned int n)
{
	/* Clip n */
	if (n > vt->scrbpos) {
		if (!vt->scrbpos)
			return;
		n = vt->scrbpos;
	}

	if (!n)
		return;

	/* Copy scrollback */
	vt->scrbpos -= n;
	memcpy(vt->vram, vt->scrb + (vt->scrbsz - vt->scrbpos) * vt->cols, min(vt->scrbpos, vt->rows) * vt->cols * CHR_VGA);

	/* Copy scroll origin */
	if (vt->scrbpos < vt->rows)
		memcpy(vt->vram + vt->scrbpos * vt->cols, vt->scro, (vt->rows - vt->scrbpos) * vt->cols * CHR_VGA);

	/* Update cursor */
	vt->cpos -= n * vt->cols;
	vt->crow -= n;

	if (vt == vt->ttypc->vt)
		_ttypc_vga_setcursor(vt);
}


void _ttypc_vga_scrollcancel(ttypc_vt_t *vt)
{
	if (!vt->scrbpos)
		return;

	/* Restore scroll origin */
	memcpy(vt->vram, vt->scro, vt->rows * vt->cols * CHR_VGA);

	/* Restore cursor position... */
	vt->cpos -= vt->scrbpos * vt->cols;
	vt->crow -= vt->scrbpos;
	vt->scrbpos = 0;

	if (vt == vt->ttypc->vt)
		_ttypc_vga_setcursor(vt);
}


void _ttypc_vga_getcursor(ttypc_vt_t *vt)
{
	vt->cpos = ttypc_vga_readreg(vt->ttypc, CRTC_CURSORH);
	vt->ccol = vt->cpos % vt->cols;
	vt->crow = vt->cpos / vt->cols;
}


void _ttypc_vga_setcursor(ttypc_vt_t *vt)
{
	ttypc_vga_writereg(vt->ttypc, CRTC_CURSORH, vt->cpos);
}


void _ttypc_vga_togglecursor(ttypc_vt_t *vt, uint8_t state)
{
	if (state) {
		/* Show cursor */
		_ttypc_vga_setcurt(vt->ttypc, vt->ctype, CUR_DEFH);
	}
	else {
		/* Hide cursor */
		outb(vt->ttypc->crtc, CRTC_CURSTART);
		outb(vt->ttypc->crtc + 1, CRTC_CUROFF);
	}
}


void ttypc_vga_destroy(ttypc_t *ttypc)
{
	munmap(ttypc->vga, _PAGE_SIZE);
}


int ttypc_vga_init(ttypc_t *ttypc)
{
	/* Test monitor type */
	ttypc->color = inb((void *)GN_MISCOUTR) & 0x01;
	ttypc->crtc = (ttypc->color) ? (void *)CRTC_COLOR : (void *)CRTC_MONO;

	/* Map video memory */
	if ((ttypc->vga = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, 0, OID_PHYSMEM, (ttypc->color) ? VGA_COLOR : VGA_MONO)) == MAP_FAILED)
		return -ENOMEM;

	return EOK;
}
