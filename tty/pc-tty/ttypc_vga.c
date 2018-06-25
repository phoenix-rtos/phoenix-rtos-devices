/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ttypc VGA support (implemented after reading FreeBSD 4.4 pcvt driver)
 *
 * Copyright 2017, 2012 Phoenix Systems
 * Copyright 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <string.h>
#include <sys/threads.h>

#include "ttypc.h"


enum { crtcCursorH = 0xe, crtcCursorL = 0xf };


void _ttypc_vga_cursor(ttypc_virt_t *virt)
{
	ttypc_t *ttypc = virt->ttypc;

	outb(ttypc->out_crtc, crtcCursorH);
	outb(ttypc->out_crtc + 1, virt->cur_offset >> 8);
	
	outb(ttypc->out_crtc, crtcCursorL);
	outb(ttypc->out_crtc + 1, virt->cur_offset & 0xff);
}


void _ttypc_vga_getcursor(ttypc_virt_t *virt)
{
	ttypc_t *ttypc = virt->ttypc;

	outb(ttypc->out_crtc, crtcCursorH);
	virt->cur_offset = (inb(ttypc->out_crtc + 1) << 8);
	
	outb(ttypc->out_crtc, crtcCursorL);
	virt->cur_offset |= inb(ttypc->out_crtc + 1);

	return;
}


void ttypc_vga_switch(ttypc_virt_t *virt)
{
	ttypc_t *ttypc = virt->ttypc;
	ttypc_virt_t *current;

	mutexLock(ttypc->mutex);
	current = ttypc->cv;

	/* video board memory -> kernel memory */
	mutexLock(current->mutex);
	memcpy(current->mem, current->vram, current->rows * current->maxcol * CHR);
	current->vram = current->mem;
	current->active = 0;
	mutexUnlock(current->mutex);

	/* kernel memory -> video board memory */
	mutexLock(virt->mutex);
	memcpy(ttypc->out_base, virt->mem, virt->rows * virt->maxcol * CHR);
	virt->vram = ttypc->out_base;
	virt->active = 1;
#if 0
	/* Restore cursor shape */
	outb(addr_6845, CRTC_STARTADRH);
	outb(addr_6845+1, 0);
	outb(addr_6845, CRTC_STARTADRL);
	outb(addr_6845+1, 0);

	select_vga_charset(vsp->vga_charset);

	if (vsp->maxcol != cols)
		vga_col(vsp, vsp->maxcol);	/* select 80/132 columns */
#endif

	_ttypc_vga_cursor(virt);
	
	/* show cursor */
/*	if(vsp->cursor_on) {
		outb(addr_6845, CRTC_CURSTART);
		outb(addr_6845+1, vsp->cursor_start);
		outb(addr_6845, CRTC_CUREND);
		outb(addr_6845+1, vsp->cursor_end);
	}*/
	mutexUnlock(virt->mutex);
	ttypc->cv = virt;

	mutexUnlock(ttypc->mutex);
}


/* scroll screen n lines up */
void _ttypc_vga_rollup(ttypc_virt_t *virt, unsigned int n)
{
	memcpy(virt->vram + (virt->scrr_beg * virt->maxcol),
		virt->vram + (virt->scrr_beg + n) * virt->maxcol,
		virt->maxcol * (virt->scrr_len - n) * CHR);
	
	memsetw(virt->vram + (virt->scrr_end - n + 1) * virt->maxcol, ' ' | virt->attr, n * virt->maxcol);
}


/* scroll screen n lines down */
void _ttypc_vga_rolldown(ttypc_virt_t *virt, unsigned int n)
{
	memcpy(virt->vram + (virt->scrr_beg + n) * virt->maxcol, virt->vram + virt->scrr_beg * virt->maxcol,
		virt->maxcol * (virt->scrr_len - n) * CHR);

	memsetw(virt->vram + virt->scrr_beg * virt->maxcol, ' ' | virt->attr, n * virt->maxcol);
}
