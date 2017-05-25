/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ttypc VT220 emulator (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2001, 2006 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _DEV_TTYPC_VTF_H_
#define _DEV_TTYPC_VTF_H_


#include <hal/if.h>

#include <dev/ttypc/ttypc_virt.h>


/* VT220 -> internal color conversion table fields */
#define VT_NORMAL       0x00
#define VT_BOLD         0x01
#define VT_UNDER        0x02
#define VT_BLINK        0x04
#define VT_INVERSE      0x08


/* initialize ANSI escape sequence parameter buffers */
extern void _ttypc_vtf_clrparms(ttypc_virt_t *virt);


/* select character attributes */
extern void _ttypc_vtf_sca(ttypc_virt_t *virt);


/* DECSTR - soft terminal reset (SOFT emulator runtime reset) */
extern void _ttypc_vtf_str(ttypc_virt_t *virt);


/* DECSC - save cursor & attributes */
extern void _ttypc_vtf_sc(ttypc_virt_t *virt);


/* DECRC - restore cursor & attributes */
extern void _ttypc_vtf_rc(ttypc_virt_t *virt);


/* device attributes */
extern void _ttypc_vtf_da(ttypc_virt_t *virt);


/* scroll up */
extern void _ttypc_vtf_su(ttypc_virt_t *virt);


/* scroll down */
extern void _ttypc_vtf_sd(ttypc_virt_t *virt);


/* CUU - cursor up */
extern void _ttypc_vtf_cuu(ttypc_virt_t *virt);


/* CUD - cursor down */
extern void _ttypc_vtf_cud(ttypc_virt_t *virt);


/* CUF - cursor forward */
extern void _ttypc_vtf_cuf(ttypc_virt_t *virt);


/* CUB - cursor backward */
extern void _ttypc_vtf_cub(ttypc_virt_t *virt);


/* ED - erase in display */
extern void _ttypc_vtf_clreos(ttypc_virt_t *virt);


/* EL - erase in line */
extern void _ttypc_vtf_clreol(ttypc_virt_t *virt);


/* CUP - cursor position / HVP - horizontal & vertical position */
extern void _ttypc_vtf_curadr(ttypc_virt_t *virt);


/* IL - insert line */
extern void _ttypc_vtf_il(ttypc_virt_t *virt);


/* ICH - insert character */
extern void _ttypc_vtf_ic(ttypc_virt_t *virt);


/* DL - delete line */
extern void _ttypc_vtf_dl(ttypc_virt_t *virt);


/* DCH - delete character */
extern void _ttypc_vtf_dch(ttypc_virt_t *virt);


/* RI - reverse index, move cursor up */
extern void _ttypc_vtf_ri(ttypc_virt_t *virt);


/* IND - index, move cursor down */
extern void _ttypc_vtf_ind(ttypc_virt_t *virt);


/* NEL - next line, first pos of next line */
extern void _ttypc_vtf_nel(ttypc_virt_t *virt);


/* clear tab stop(s) */
extern void _ttypc_vtf_clrtab(ttypc_virt_t *virt);


/* RIS - reset to initial state (hard emulator runtime reset) */
extern void _ttypc_vtf_ris(ttypc_virt_t *virt);


/* ECH - erase character */
extern void _ttypc_vtf_ech(ttypc_virt_t *virt);


/* media copy	(NO PRINTER AVAILABLE IN KERNEL ...) */
extern void _ttypc_vtf_mc(ttypc_virt_t *virt);


/* invoke selftest */
extern void _ttypc_vtf_tst(ttypc_virt_t *virt);


/* SGR - set graphic rendition */
extern void _ttypc_vtf_sgr(ttypc_virt_t *virt);


/* device status reports */
extern void _ttypc_vtf_dsr(ttypc_virt_t *virt);


/* DECSTBM - set top and bottom margins */
extern void _ttypc_vtf_stbm(ttypc_virt_t *virt);


/* set ansi modes, esc [ x */
extern void _ttypc_vtf_set_ansi(ttypc_virt_t *virt);


/* reset ansi modes, esc [ x */
extern void _ttypc_vtf_reset_ansi(ttypc_virt_t *virt);


/* request terminal parameters */
extern void _ttypc_vtf_reqtparm(ttypc_virt_t *virt);


/* switch keypad to numeric mode */
extern void _ttypc_vtf_keynum(ttypc_virt_t *virt);


/* switch keypad to application mode */
extern void _ttypc_vtf_keyappl(ttypc_virt_t *virt);


#endif
