/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ttypc VT220 functions (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * %LICENSE%
 */

#include <sys/minmax.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ttypc.h"
#include "ttypc_vtf.h"
#include "ttypc_vga.h"


/* Color attributes for foreground text */

#define	FG_BLACK           0
#define	FG_BLUE            1
#define	FG_GREEN           2
#define	FG_CYAN            3
#define	FG_RED             4
#define	FG_MAGENTA         5
#define	FG_BROWN           6
#define	FG_LIGHTGREY       7
#define	FG_DARKGREY        8
#define	FG_LIGHTBLUE       9
#define	FG_LIGHTGREEN      10
#define	FG_LIGHTCYAN       11
#define	FG_LIGHTRED        12
#define	FG_LIGHTMAGENTA    13
#define	FG_YELLOW          14
#define	FG_WHITE           15
#define	FG_BLINK           0x80

/* Color attributes for text background */

#define	BG_BLACK           0x00
#define	BG_BLUE            0x10
#define	BG_GREEN           0x20
#define	BG_CYAN            0x30
#define	BG_RED             0x40
#define	BG_MAGENTA         0x50
#define	BG_BROWN           0x60
#define	BG_LIGHTGREY       0x70

/* Monochrome attributes for foreground text */

#define	FG_UNDERLINE       0x01
#define	FG_INTENSE         0x08

/* Monochrome attributes for text background */

#define	BG_INTENSE         0x10


/*---------------------------------------------------------------------------

	VT220 attributes -> internal emulator attributes conversion tables

	be careful when designing color combinations, because on
	EGA and VGA displays, bit 3 of the attribute byte is used
	for characterset switching, and is no longer available for
	foreground intensity (bold)!

---------------------------------------------------------------------------*/

/* color displays */

uint8_t sgr_tab_color[16] = {
	(BG_BLACK     | FG_LIGHTGREY),             /* normal               */
	(BG_BLUE      | FG_LIGHTGREY),             /* bold                 */
	(BG_BROWN     | FG_LIGHTGREY),             /* underline            */
	(BG_MAGENTA   | FG_LIGHTGREY),             /* bold+underline       */
	(BG_BLACK     | FG_LIGHTGREY | FG_BLINK),  /* blink                */
	(BG_BLUE      | FG_LIGHTGREY | FG_BLINK),  /* bold+blink           */
	(BG_BROWN     | FG_LIGHTGREY | FG_BLINK),  /* underline+blink      */
	(BG_MAGENTA   | FG_LIGHTGREY | FG_BLINK),  /* bold+underline+blink */
	(BG_LIGHTGREY | FG_BLACK),                 /* invers               */
	(BG_LIGHTGREY | FG_BLUE),                  /* bold+invers          */
	(BG_LIGHTGREY | FG_BROWN),                 /* underline+invers     */
	(BG_LIGHTGREY | FG_MAGENTA),               /* bold+underline+invers*/
	(BG_LIGHTGREY | FG_BLACK      | FG_BLINK), /* blink+invers         */
	(BG_LIGHTGREY | FG_BLUE       | FG_BLINK), /* bold+blink+invers    */
	(BG_LIGHTGREY | FG_BROWN      | FG_BLINK), /* underline+blink+invers*/
	(BG_LIGHTGREY | FG_MAGENTA    | FG_BLINK)  /* bold+underl+blink+invers */
};

/* monochrome displays (VGA version, no intensity) */

uint8_t sgr_tab_mono[16] = {
	(BG_BLACK     | FG_LIGHTGREY),            /* normal               */
	(BG_BLACK     | FG_UNDERLINE),            /* bold                 */
	(BG_BLACK     | FG_UNDERLINE),            /* underline            */
	(BG_BLACK     | FG_UNDERLINE),            /* bold+underline       */
	(BG_BLACK     | FG_LIGHTGREY | FG_BLINK), /* blink                */
	(BG_BLACK     | FG_UNDERLINE | FG_BLINK), /* bold+blink           */
	(BG_BLACK     | FG_UNDERLINE | FG_BLINK), /* underline+blink      */
	(BG_BLACK     | FG_UNDERLINE | FG_BLINK), /* bold+underline+blink */
	(BG_LIGHTGREY | FG_BLACK),                /* invers               */
	(BG_LIGHTGREY | FG_BLACK),                /* bold+invers          */
	(BG_LIGHTGREY | FG_BLACK),                /* underline+invers     */
	(BG_LIGHTGREY | FG_BLACK),                /* bold+underline+invers*/
	(BG_LIGHTGREY | FG_BLACK | FG_BLINK),     /* blink+invers         */
	(BG_LIGHTGREY | FG_BLACK | FG_BLINK),     /* bold+blink+invers    */
	(BG_LIGHTGREY | FG_BLACK | FG_BLINK),     /* underline+blink+invers */
	(BG_LIGHTGREY | FG_BLACK | FG_BLINK)      /* bold+underl+blink+invers */
};


/* foreground ANSI color -> pc */
uint8_t fgansitopc[] = {
	FG_BLACK, FG_RED, FG_GREEN, FG_BROWN, FG_BLUE,
	FG_MAGENTA, FG_CYAN, FG_LIGHTGREY
};


/* background ANSI color -> pc */
uint8_t bgansitopc[] = {
	BG_BLACK, BG_RED, BG_GREEN, BG_BROWN, BG_BLUE,
	BG_MAGENTA, BG_CYAN, BG_LIGHTGREY
};


/* initialize ANSI escape sequence parameter buffers */
void _ttypc_vtf_clrparms(ttypc_virt_t *virt)
{
	register int i;

	for (i = 0; i < MAXPARMS; i++)
		virt->parms[i] = 0;
	virt->parmi = 0;
}


/* select character attributes */
void _ttypc_vtf_sca(ttypc_virt_t *virt)
{
	switch (virt->parms[0]) {
	case 1:
//		virt->selchar = 1;
		break;
	case 0:
	case 2:
	default:
//		virt->selchar = 0;
		break;
	}
}


/* DECSTR - soft terminal reset (SOFT emulator runtime reset) */
void _ttypc_vtf_str(ttypc_virt_t *virt)
{
	ttypc_t *ttypc = virt->ttypc;
	int i;

	_ttypc_vtf_clrparms(virt);
	virt->state = STATE_INIT;
	virt->sc_flag = 0;

	/* setup tabstops */
	for (i = 0; i < MAXTAB; i++) {
		if (i % 8 == 0)
			virt->tab_stops[i] = 1;
		else
			virt->tab_stops[i] = 0;
	}

	virt->m_irm = 0;
	virt->m_awm = 1;
	virt->m_ckm = 1;
	
	virt->scrr_beg = 0;
	virt->scrr_len = virt->rows;
	virt->scrr_end = virt->scrr_len - 1;

	virt->G0 = csd_ascii;
	virt->G1 = csd_ascii;
	virt->G2 = csd_supplemental;
	virt->G3 = csd_supplemental;
	virt->GL = &virt->G0;
	virt->GR = &virt->G2;

	virt->vtsgr = VT_NORMAL;

	if (ttypc->color)
		virt->attr = ((sgr_tab_color[virt->vtsgr]) << 8);
	else
		virt->attr = ((sgr_tab_mono[virt->vtsgr]) << 8);
}


/* DECSC - save cursor & attributes */
void _ttypc_vtf_sc(ttypc_virt_t *virt)
{
	virt->sc_flag = 1;
	virt->sc_row = virt->row;
	virt->sc_col = virt->col;
	virt->sc_cur_offset = virt->cur_offset;
	virt->sc_attr = virt->attr;

	virt->sc_awm = virt->m_awm;

	virt->sc_G0 = virt->G0;
	virt->sc_G1 = virt->G1;
	virt->sc_G2 = virt->G2;
	virt->sc_G3 = virt->G3;
	virt->sc_GL = virt->GL;
	virt->sc_GR = virt->GR;
}


/* DECRC - restore cursor & attributes */
void _ttypc_vtf_rc(ttypc_virt_t *virt)
{
	if (virt->sc_flag == 1)
		return;
	
	virt->sc_flag = 0;
	virt->row = virt->sc_row;
	virt->col = virt->sc_col;
	virt->cur_offset = virt->sc_cur_offset;
	virt->attr = virt->sc_attr;
	
	virt->m_awm = virt->sc_awm;

	virt->G0 = virt->sc_G0;
	virt->G1 = virt->sc_G1;
	virt->G2 = virt->sc_G2;
	virt->G3 = virt->sc_G3;
	virt->GL = virt->sc_GL;
	virt->GR = virt->sc_GR;
}


/* device attributes */
void _ttypc_vtf_da(ttypc_virt_t *virt)
{
#if 0
	static uint8_t *response = (u_char *)DA_VT220;

	svsp->report_chars = response;
	svsp->report_count = 18;
	respond(svsp);
#endif
}


/* scroll up */
void _ttypc_vtf_su(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if (p <= 0)
		p = 1;
	else if (p > virt->rows - 1)
		p = virt->rows - 1;

	_ttypc_vga_rollup(virt, p);
}


/* scroll down */
void _ttypc_vtf_sd(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if (p <= 0)
		p = 1;
	else if (p > virt->rows - 1)
		p = virt->rows - 1;

	_ttypc_vga_rolldown(virt, p);
}


/* CUU - cursor up */
void _ttypc_vtf_cuu(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	p = min(p <= 0 ? 1 : p, virt->row - virt->scrr_beg);

	if (p <= 0)
		return;

	virt->cur_offset -= virt->maxcol * p;
}


/* CUD - cursor down */
void _ttypc_vtf_cud(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	p = min(p <= 0 ? 1 : p, virt->scrr_end - virt->row);
	virt->cur_offset += (virt->maxcol * p);
}


/* CUF - cursor forward */
void _ttypc_vtf_cuf(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if (p <= 0)
		p = 1;

	p = min(p, virt->maxcol - virt->col - 1);
	
	virt->cur_offset += p;
	virt->col += p;
}


/* CUB - cursor backward */
void _ttypc_vtf_cub(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if (p <= 0)
		p = 1;

	p = min(p, virt->col);

	virt->cur_offset -= p;
	virt->col -= p;
}


/* ED - erase in display */
void _ttypc_vtf_clreos(ttypc_virt_t *virt)
{
	switch (virt->parms[0]) {
	case 0:
		memsetw(virt->vram + virt->cur_offset, ' ' | virt->attr, virt->maxcol * virt->rows - virt->cur_offset);
		break;

	case 1:
		memsetw(virt->vram, ' ' | virt->attr, virt->cur_offset + 1);
		break;

	case 2:
		memsetw(virt->vram, ' ' | virt->attr, virt->maxcol * virt->rows);
		break;
	}
}


/* EL - erase in line */
void _ttypc_vtf_clreol(ttypc_virt_t *virt)
{
	switch (virt->parms[0]) {
	case 0:
		memsetw(virt->vram + virt->cur_offset, ' ' | virt->attr, virt->maxcol - virt->col);
		break;

	case 1:
		memsetw(virt->vram + virt->cur_offset - virt->col, ' ' | virt->attr, virt->col + 1);
		break;

	case 2:
		memsetw(virt->vram + virt->cur_offset - virt->col, ' ' | virt->attr, virt->maxcol);
		break;
	}
}


/* CUP - cursor position */
void _ttypc_vtf_curadr(ttypc_virt_t *virt)
{
	/* relative to screen start */
	if ((virt->parms[0] == 0) && (virt->parms[1] == 0)) {
		virt->cur_offset = 0;
		virt->col = 0;
		return;
	}

	if (virt->parms[0] <= 0)
		virt->parms[0] = 1;
	if (virt->parms[0] > virt->rows)
		virt->parms[0] = virt->rows;

	if (virt->parms[1] <= 0)
		virt->parms[1] = 1;
	if (virt->parms[1] > virt->maxcol)
		virt->parms[1] = virt->maxcol;

	virt->cur_offset = (((virt->parms[0] - 1) * virt->maxcol) + (virt->parms[1] - 1));
	virt->col = virt->parms[1] - 1;
}


/* IL - insert line */
void _ttypc_vtf_il(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if ((virt->row >= virt->scrr_beg) && (virt->row <= virt->scrr_end)) {
		if (p <= 0)
			p = 1;
		else if (p > virt->scrr_end - virt->row)
			p = virt->scrr_end - virt->row;

		virt->cur_offset -= virt->col;
		virt->col = 0;
	
		if (virt->row == virt->scrr_beg)
			_ttypc_vga_rolldown(virt, p);
		else {
			memcpy(virt->vram + virt->cur_offset + p * virt->maxcol, virt->vram + virt->cur_offset,
				virt->maxcol * (virt->scrr_end - virt->row + 1 - p) * CHR);

			memsetw(virt->vram + virt->cur_offset, ' ' | virt->attr, p * virt->maxcol);
		}
	}
}


/* ICH - insert character */
void _ttypc_vtf_ic(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if (p <= 0)
		p = 1;
	else if (p > virt->maxcol - virt->col)
		p = virt->maxcol - virt->col;

	memcpy(virt->vram + virt->cur_offset + p, virt->vram + virt->cur_offset, virt->maxcol - p - virt->col);
	memsetw(virt->vram + virt->cur_offset, ' ' | virt->attr, p);
}


/* DL - delete line */
void _ttypc_vtf_dl(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if ((virt->row >= virt->scrr_beg) && (virt->row <= virt->scrr_end)) {
		if (p <= 0)
			p = 1;
		else if (p > virt->scrr_end - virt->row)
			p = virt->scrr_end - virt->row;

		virt->cur_offset -= virt->col;
		virt->col = 0;

		if (virt->row == virt->scrr_beg) {
			_ttypc_vga_rollup(virt, p);
		}
		else {
			memcpy(virt->vram + virt->cur_offset, virt->vram + virt->cur_offset + p * virt->maxcol,
			  virt->maxcol * (virt->scrr_end - virt->row + 1 - p) * CHR);

			memsetw(virt->vram + (virt->scrr_end - p + 1) * virt->maxcol, ' ' | virt->attr, p * virt->maxcol);
		}
	}
}


/* DCH - delete character */
void _ttypc_vtf_dch(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if (p <= 0)
		p = 1;

	p = min(p, virt->maxcol - virt->col);
	
	memcpy(virt->vram + virt->cur_offset, virt->vram + virt->cur_offset + p, (virt->maxcol - p - virt->col) * CHR);
	memsetw(virt->vram + virt->cur_offset + virt->maxcol - p, ' ' | virt->attr, p);	
}


/* RI - reverse index, move cursor up */
void _ttypc_vtf_ri(ttypc_virt_t *virt)
{
	if (virt->cur_offset >= virt->scrr_beg * virt->maxcol + virt->maxcol)
		virt->cur_offset -= virt->maxcol;
	else
		_ttypc_vga_rolldown(virt, 1);
}


/* IND - index, move cursor down */
void _ttypc_vtf_ind(ttypc_virt_t *virt)
{
	if (virt->cur_offset < virt->scrr_end * virt->maxcol)
		virt->cur_offset += virt->maxcol;
	else
		_ttypc_vga_rollup(virt, 1);
}


/* NEL - next line, first pos of next line */
void _ttypc_vtf_nel(ttypc_virt_t *virt)
{
	if (virt->cur_offset < virt->scrr_end * virt->maxcol) {
		virt->cur_offset += virt->maxcol - virt->col;
		virt->col = 0;
	}
	else {
		_ttypc_vga_rollup(virt, 1);
		virt->cur_offset -= virt->col;
		virt->col = 0;
	}
}


/* clear tab stop(s) */
void _ttypc_vtf_clrtab(ttypc_virt_t *virt)
{
	int i;

	if (virt->parms[0] == 0)
		virt->tab_stops[virt->col] = 0;

	if (virt->parms[0] == 3) {
		for (i = 0; i < MAXTAB; i++)
			virt->tab_stops[i] = 0;
	}
}


/* RIS - reset to initial state (hard emulator runtime reset) */
void _ttypc_vtf_ris(ttypc_virt_t *virt)
{	
	virt->cur_offset = 0;
	virt->col = 0;
	virt->row = 0;
	virt->m_lnm = 0;

	memsetw(virt->vram + virt->cur_offset, ' ' | virt->attr, virt->maxcol * virt->rows);

	_ttypc_vtf_str(virt);
}


/* ECH - erase character */
void _ttypc_vtf_ech(ttypc_virt_t *virt)
{
	register int p = virt->parms[0];

	if (p <= 0)
		p = 1;
	else if (p > virt->maxcol - virt->col)
		p = virt->maxcol - virt->col;

	memsetw(virt->vram + virt->cur_offset, ' ' | virt->attr, p);
}


/* media copy	(NO PRINTER AVAILABLE IN KERNEL ...) */
void _ttypc_vtf_mc(ttypc_virt_t *virt)
{
}


/* invoke selftest */
void _ttypc_vtf_tst(ttypc_virt_t *virt)
{
}


/* SGR - set graphic rendition */
void _ttypc_vtf_sgr(ttypc_virt_t *virt)
{
	ttypc_t *ttypc = virt->ttypc;
	register int i = 0;
	uint16_t setcolor = 0;
	char colortouched = 0;

	do {
		switch (virt->parms[i++]) {
		case 0:                                 /* reset to normal attributes */
			virt->vtsgr = VT_NORMAL;
			break;
		case 1:                                 /* bold */
			virt->vtsgr |= VT_BOLD;
			break;
		case 4:                                 /* underline */
			virt->vtsgr |= VT_UNDER;
			break;
		case 5:                                 /* blinking */
			virt->vtsgr |= VT_BLINK;
			break;
		case 7:                                 /* reverse */
			virt->vtsgr |= VT_INVERSE;
			break;
		case 22:                                /* not bold */
			virt->vtsgr &= ~VT_BOLD;
			break;
		case 24:                                /* not underlined */
			virt->vtsgr &= ~VT_UNDER;
			break;
		case 25:                                /* not blinking */
			virt->vtsgr &= ~VT_BLINK;
			break;
		case 27:                                /* not reverse */
			virt->vtsgr &= ~VT_INVERSE;
			break;

		case 30:                                /* foreground colors */
		case 31:
		case 32:
		case 33:
		case 34:
		case 35:
		case 36:
		case 37:
			if (ttypc->color) {
				colortouched = 1;
				setcolor |= ((fgansitopc[(virt->parms[i - 1] - 30) & 7]) << 8);
			}
			break;

		case 40:                                /* background colors */
		case 41:
		case 42:
		case 43:
		case 44:
		case 45:
		case 46:
		case 47:
			if (ttypc->color) {
				colortouched = 1;
				setcolor |= ((bgansitopc[(virt->parms[i - 1] - 40) & 7]) << 8);
			}
			break;
		}
	} while (i <= virt->parmi);
	
	if (ttypc->color) {
		if (colortouched)
			virt->attr = setcolor;
		else
			virt->attr = ((sgr_tab_color[virt->vtsgr]) << 8);
	}
	else
		virt->attr = ((sgr_tab_mono[virt->vtsgr]) << 8);
}


/* device status reports */
void _ttypc_vtf_dsr(ttypc_virt_t *virt)
{
#if 0
	static uint8_t *answr = (uint8_t *)"\033[0n";
	static uint8_t *panswr = (uint8_t *)"\033[?13n";      /* Printer Unattached */
	static uint8_t *udkanswr = (uint8_t *)"\033[?21n";    /* UDK Locked */
	static uint8_t *langanswr = (uint8_t *)"\033[?27;1n"; /* North American*/
#endif
#if 0
	static uint8_t buffer[16];

	int i = 0;

	switch (virt->parms[0]) {
	
	/* return status */
	case 5:
		/* respond(answr); */
		break;

	/* return cursor position */
	case 6:
		buffer[i++] = 0x1b;
		buffer[i++] = '[';
		
		if (virt->row + 1 > 10)
			buffer[i++] = (virt->row + 1) / 10 + '0';
		buffer[i++] = (virt->row + 1) % 10 + '0';
		buffer[i++] = ';';

		if (virt->col + 1 > 10)
			buffer[i++] = (virt->col + 1) / 10 + '0';
		buffer[i++] = (virt->col + 1) % 10 + '0';
		buffer[i++] = 'R';
		buffer[i++] = '\0';

		/* respond(buffer) */;
		break;

	/* return printer status */
	case 15:
		/* respond(panswr); */
		break;

	/* return udk status */
	case 25:
		/* respond(udkanswr); */
		break;

	/* return language status */
	case 26:
		/* respond(langanswr); */
		break;

	default:
		break;
	}
#endif
}


/* DECSTBM - set top and bottom margins */
void _ttypc_vtf_stbm(ttypc_virt_t *virt)
{

	if (virt->parms[1] <= virt->parms[0])
		return;

	/* both 0 => scrolling region = entire screen */
	if (!virt->parms[0] && !virt->parms[1]) {
		virt->cur_offset = 0;
		virt->scrr_beg = 0;
		virt->scrr_len = virt->rows;
		virt->scrr_end = virt->scrr_len - 1;
		virt->col = 0;
		return;
	}

	/* range parm 1 */
	if (virt->parms[0] < 1)
		virt->parms[0] = 1;
	else if (virt->parms[0] > virt->rows - 1)
		virt->parms[0] = virt->rows - 1;

	/* range parm 2 */
	if (virt->parms[1] < 2)
		virt->parms[1] = 2;
	else if (virt->parms[1] > virt->rows)
		virt->parms[1] = virt->rows;

	virt->scrr_beg = virt->parms[0] - 1;
	virt->scrr_len = virt->parms[1] - virt->parms[0] + 1;
	virt->scrr_end = virt->parms[1] - 1;

	/* cursor to first pos */
	virt->cur_offset = 0;

	virt->col = 0;
}


/* set ansi modes, esc [ x */
void _ttypc_vtf_set_ansi(ttypc_virt_t *virt)
{
	switch(virt->parms[0]) {
	case 0:		/* error, ignored */
	case 1:		/* GATM - guarded area transfer mode */
	case 2:		/* KAM - keyboard action mode */
	case 3:		/* CRM - Control Representation mode */
		break;

	case 4:		/* IRM - insert replacement mode */
		virt->m_irm = 1;
		break;

	case 5:		/* SRTM - status report transfer mode */
	case 6:		/* ERM - erasue mode */
	case 7:		/* VEM - vertical editing mode */
	case 10:	/* HEM - horizontal editing mode */
	case 11:	/* PUM - position unit mode */
	case 12:	/* SRM - send-receive mode */
	case 13:	/* FEAM - format effector action mode */
	case 14:	/* FETM - format effector transfer mode */
	case 15:	/* MATM - multiple area transfer mode */
	case 16:	/* TTM - transfer termination */
	case 17:	/* SATM - selected area transfer mode */
	case 18:	/* TSM - tabulation stop mode */
	case 19:	/* EBM - editing boundary mode */
		break;
	case 20:	/* LNM - line feed / newline mode */
		virt->m_lnm = 1;
		break;
	}
}


/* reset ansi modes, esc [ x */
void _ttypc_vtf_reset_ansi(ttypc_virt_t *virt)
{
	switch (virt->parms[0]) {
	case 0:		/* error, ignored */
	case 1:		/* GATM - guarded area transfer mode */
	case 2:		/* KAM - keyboard action mode */
	case 3:		/* CRM - Control Representation mode */
		break;

	case 4:		/* IRM - insert replacement mode */
		virt->m_irm = 0;
		break;

	case 5:		/* SRTM - status report transfer mode */
	case 6:		/* ERM - erasue mode */
	case 7:		/* VEM - vertical editing mode */
	case 10:	/* HEM - horizontal editing mode */
	case 11:	/* PUM - position unit mode */
	case 12:	/* SRM - send-receive mode */
	case 13:	/* FEAM - format effector action mode */
	case 14:	/* FETM - format effector transfer mode */
	case 15:	/* MATM - multiple area transfer mode */
	case 16:	/* TTM - transfer termination */
	case 17:	/* SATM - selected area transfer mode */
	case 18:	/* TSM - tabulation stop mode */
	case 19:	/* EBM - editing boundary mode */
		break;

	case 20:	/* LNM - line feed / newline mode */
		virt->m_lnm = 0;
		break;
	}
}


/* request terminal parameters */
void _ttypc_vtf_reqtparm(ttypc_virt_t *virt)
{
	/* static uint8_t *answr = (uint8_t *)"\033[3;1;1;120;120;1;0x"; */

	/* respond(answr); */
}


/* switch keypad to numeric mode */
void _ttypc_vtf_keynum(ttypc_virt_t *virt)
{
/*	virt->num_lock = 1;
	update_led();*/
}


/* switch keypad to application mode */
void _ttypc_vtf_keyappl(ttypc_virt_t *virt)
{
/*	svsp->num_lock = 0;
	update_led(); */
}
