/*
 * Phoenix-RTOS
 *
 * Virtual Terminal Functions
 *
 * Copyright 2001, 2006 Pawel Pisarczyk
 * Copyright 2012, 2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _TTYPC_VTF_H_
#define _TTYPC_VTF_H_

#include "ttypc_vt.h"


/* Clears ANSI escape sequence parameter buffers */
extern void _ttypc_vtf_clrparms(ttypc_vt_t *vt);


/* DECSTR - Soft Terminal Reset (SOFT emulator runtime reset) */
extern void _ttypc_vtf_str(ttypc_vt_t *vt);


/* DECSC - Save Cursor & attributes */
extern void _ttypc_vtf_sc(ttypc_vt_t *vt);


/* DECRC - Restore Cursor & attributes */
extern void _ttypc_vtf_rc(ttypc_vt_t *vt);


/* Device attributes */
extern void _ttypc_vtf_da(ttypc_vt_t *vt);


/* Screen alignment display */
extern void _ttypc_vtf_aln(ttypc_vt_t *vt);


/* Scroll up */
extern void _ttypc_vtf_su(ttypc_vt_t *vt);


/* Scroll down */
extern void _ttypc_vtf_sd(ttypc_vt_t *vt);


/* CUU - cursor up */
extern void _ttypc_vtf_cuu(ttypc_vt_t *vt);


/* CUD - cursor down */
extern void _ttypc_vtf_cud(ttypc_vt_t *vt);


/* CUF - cursor forward */
extern void _ttypc_vtf_cuf(ttypc_vt_t *vt);


/* CUB - cursor backward */
extern void _ttypc_vtf_cub(ttypc_vt_t *vt);


/* ED - erase in display */
extern void _ttypc_vtf_clreos(ttypc_vt_t *vt);


/* EL - erase in line */
extern void _ttypc_vtf_clreol(ttypc_vt_t *vt);


/* CUP - cursor position / HVP - horizontal & vertical position */
extern void _ttypc_vtf_curadr(ttypc_vt_t *vt);


/* IL - insert line */
extern void _ttypc_vtf_il(ttypc_vt_t *vt);


/* ICH - insert character */
extern void _ttypc_vtf_ic(ttypc_vt_t *vt);


/* DL - delete line */
extern void _ttypc_vtf_dl(ttypc_vt_t *vt);


/* DCH - delete character */
extern void _ttypc_vtf_dch(ttypc_vt_t *vt);


/* RI - reverse index, move cursor up */
extern void _ttypc_vtf_ri(ttypc_vt_t *vt);


/* IND - index, move cursor down */
extern void _ttypc_vtf_ind(ttypc_vt_t *vt);


/* NEL - next line, first pos of next line */
extern void _ttypc_vtf_nel(ttypc_vt_t *vt);


/* Clear tab stop(s) */
extern void _ttypc_vtf_clrtab(ttypc_vt_t *vt);


/* RIS - reset to initial state (hard emulator runtime reset) */
extern void _ttypc_vtf_ris(ttypc_vt_t *vt);


/* ECH - erase character */
extern void _ttypc_vtf_ech(ttypc_vt_t *vt);


/* Media copy (NO PRINTER AVAILABLE IN KERNEL ...) */
extern void _ttypc_vtf_mc(ttypc_vt_t *vt);


/* Invoke selftest */
extern void _ttypc_vtf_tst(ttypc_vt_t *vt);


/* SGR - set graphic rendition */
extern void _ttypc_vtf_sgr(ttypc_vt_t *vt);


/* Device status reports */
extern void _ttypc_vtf_dsr(ttypc_vt_t *vt);


/* DECSTBM - set top and bottom margins */
extern void _ttypc_vtf_stbm(ttypc_vt_t *vt);


/* Set dec private modes, esc [ ? x h */
extern void _ttypc_vtf_setdecpriv(ttypc_vt_t *vt);


/* Reset dec private modes, esc [ ? x l */
extern void _ttypc_vtf_resetdecpriv(ttypc_vt_t *vt);


/* Set ansi modes, esc [ x */
extern void _ttypc_vtf_setansi(ttypc_vt_t *vt);


/* Reset ansi modes, esc [ x */
extern void _ttypc_vtf_resetansi(ttypc_vt_t *vt);


/* Request terminal parameters */
extern void _ttypc_vtf_reqtparm(ttypc_vt_t *vt);


/* Switch keypad to numeric mode */
extern void _ttypc_vtf_keynum(ttypc_vt_t *vt);


/* Switch keypad to application mode */
extern void _ttypc_vtf_keyappl(ttypc_vt_t *vt);


#endif
