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


/* Reads CRTC registers */
static uint16_t ttypc_vga_readreg(ttypc_t *ttypc, uint8_t reg)
{
	uint16_t val;

	outb(ttypc->crtc, reg);
	val = (uint16_t)inb(ttypc->crtc + 1) << 8;
	outb(ttypc->crtc, reg + 1);
	val |= inb(ttypc->crtc + 1);

	return val;
}


/* Writes to CRTC registers */
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
	cvt->vram = cvt->mem;

	mutexLock(vt->lock);
	/* VT memory -> VGA memory */
	vt->vram = ttypc->vga;
	memcpy(vt->vram, vt->mem, vt->rows * vt->cols * CHR_VGA);
	/* Set cursor position... */
	_ttypc_vga_setcursor(vt);
	/* ... and visibility */
	_ttypc_vga_togglecursor(vt, vt->cst);
	mutexUnlock(vt->lock);

	/* Set active VT */
	ttypc->vt = vt;
}


/* Returns scrollback capacity in lines */
static unsigned int _ttypc_vga_scrollbackcapacity(ttypc_vt_t *vt)
{
	return (SCRB_PAGES * _PAGE_SIZE) / (vt->cols * CHR_VGA);
}


/* Makes space for n new lines in the scrollback buffer */
static void _ttypc_vga_allocscrollback(ttypc_vt_t *vt, unsigned int n)
{
	unsigned int scrbcap = _ttypc_vga_scrollbackcapacity(vt);

	if (vt->scrbsz + n > scrbcap) {
		memmove(vt->scrb, vt->scrb + (vt->scrbsz + n - scrbcap) * vt->cols, (scrbcap - n) * vt->cols * CHR_VGA);
		vt->scrbsz = scrbcap;
	}
	else {
		vt->scrbsz += n;
	}
}


void _ttypc_vga_rollup(ttypc_vt_t *vt, unsigned int n)
{
	unsigned int k = min(n, _ttypc_vga_scrollbackcapacity(vt));

	/* Update scrollback buffer */
	_ttypc_vga_allocscrollback(vt, k);
	memcpy(vt->scrb + (vt->scrbsz - k) * vt->cols, vt->vram + (vt->top + n - k) * vt->cols, k * vt->cols * CHR_VGA);

	/* Roll up */
	if (n < vt->bottom - vt->top) {
		memmove(vt->vram + vt->top * vt->cols, vt->vram + (vt->top + n) * vt->cols, (vt->bottom - vt->top + 1 - n) * vt->cols * CHR_VGA);
		memsetw(vt->vram + (vt->bottom + 1 - n) * vt->cols, vt->attr | ' ', n * vt->cols);
	}
}


void _ttypc_vga_rolldown(ttypc_vt_t *vt, unsigned int n)
{
	unsigned int k;

	/* Roll down */
	if (vt->bottom > vt->crow) {
		k = min(n, vt->bottom - vt->crow);
		memmove(vt->vram + (vt->top + k) * vt->cols, vt->vram + vt->top * vt->cols, (vt->bottom - vt->top + 1 - k) * vt->cols * CHR_VGA);
		memsetw(vt->vram + vt->top * vt->cols, vt->attr | ' ', k * vt->cols);

		if (n == k)
			return;
		n -= k;
	}
	k = min(n, _ttypc_vga_scrollbackcapacity(vt));

	/* Update scrollback buffer */	
	_ttypc_vga_allocscrollback(vt, k);
	memsetw(vt->scrb + (vt->scrbsz - k) * vt->cols, vt->attr | ' ', k * vt->cols);
}


void _ttypc_vga_scroll(ttypc_vt_t *vt, int n)
{
	if (n > vt->scrbsz - vt->scrbpos)
		n = vt->scrbsz - vt->scrbpos;
	else if (n < -vt->scrbpos)
		n = -vt->scrbpos;

	if (!n)
		return;

	/* Update cursor position */
	vt->cpos += n * vt->cols;
	vt->crow += n;

	/* Save scroll origin */
	if (!vt->scrbpos)
		memcpy(vt->scro, vt->vram, vt->rows * vt->cols * CHR_VGA);

	/* Copy scrollback */
	vt->scrbpos += n;
	memcpy(vt->vram, vt->scrb + (vt->scrbsz - vt->scrbpos) * vt->cols, min(vt->scrbpos, vt->rows) * vt->cols * CHR_VGA);

	/* Copy scroll origin */
	if (vt->scrbpos < vt->rows) {
		memcpy(vt->vram + vt->scrbpos * vt->cols, vt->scro, (vt->rows - vt->scrbpos) * vt->cols * CHR_VGA);

		if ((vt == vt->ttypc->vt) && vt->cst) {
			/* Show cursor */
			_ttypc_vga_setcursor(vt);
			_ttypc_vga_togglecursor(vt, 1);
		}
	}
	else {
		/* Hide cursor */
		_ttypc_vga_togglecursor(vt, 0);
	}
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
	/* ... and visibility */
	if ((vt == vt->ttypc->vt) && vt->cst) {
		_ttypc_vga_setcursor(vt);
		_ttypc_vga_togglecursor(vt, 1);
	}
	
	vt->scrbpos = 0;
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
