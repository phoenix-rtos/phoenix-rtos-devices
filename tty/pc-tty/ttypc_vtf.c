/*
 * Phoenix-RTOS
 *
 * Virtual Terminal Functions
 *
 * Copyright 2008 Pawel Pisarczyk
 * Copyright 2012, 2019, 2020, 2023 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski, Gerard Swiderski
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "ttypc_bioskbd.h"
#include "ttypc_kbd.h"
#include "ttypc_vga.h"
#include "ttypc_vtf.h"


/* Color display SGR attributes */
static const uint8_t csgr[] = {
	(BG_BLACK     | FG_LIGHTGREY),            /* Normal */
	(BG_BLACK     | FG_WHITE),                /* Bold */
	(BG_BROWN     | FG_LIGHTGREY),            /* Underline */
	(BG_MAGENTA   | FG_WHITE),                /* Bold + underline */
	(BG_BLACK     | FG_LIGHTGREY | FG_BLINK), /* Blink */
	(BG_BLUE      | FG_WHITE     | FG_BLINK), /* Bold + blink */
	(BG_BROWN     | FG_LIGHTGREY | FG_BLINK), /* Underline + blink */
	(BG_MAGENTA   | FG_WHITE     | FG_BLINK), /* Bold + underline + blink */
	(BG_LIGHTGREY | FG_BLACK),                /* Inversed */
	(BG_LIGHTGREY | FG_WHITE),                /* Bold + inversed */
	(BG_LIGHTGREY | FG_BROWN),                /* Underline + inversed */
	(BG_LIGHTGREY | FG_WHITE),                /* Bold + underline + inversed */
	(BG_LIGHTGREY | FG_BLACK     | FG_BLINK), /* Blink + inversed */
	(BG_LIGHTGREY | FG_WHITE     | FG_BLINK), /* Bold + blink + inversed */
	(BG_LIGHTGREY | FG_BROWN     | FG_BLINK), /* Underline + blink + inversed */
	(BG_LIGHTGREY | FG_WHITE     | FG_BLINK)  /* Bold + underline + blink + inversed */
};


/* Monochrome display SGR attributes */
static const uint8_t msgr[] = {
	(BG_BLACK     | FG_LIGHTGREY),            /* Normal */
	(BG_BLACK     | FG_UNDERLINE),            /* Bold */
	(BG_BLACK     | FG_UNDERLINE),            /* Underline */
	(BG_BLACK     | FG_UNDERLINE),            /* Bold + underline */
	(BG_BLACK     | FG_LIGHTGREY | FG_BLINK), /* Blink */
	(BG_BLACK     | FG_UNDERLINE | FG_BLINK), /* Bold + blink */
	(BG_BLACK     | FG_UNDERLINE | FG_BLINK), /* Underline + blink */
	(BG_BLACK     | FG_UNDERLINE | FG_BLINK), /* Bold + underline + blink */
	(BG_LIGHTGREY | FG_BLACK),                /* Inversed */
	(BG_LIGHTGREY | FG_BLACK),                /* Bold + inversed */
	(BG_LIGHTGREY | FG_BLACK),                /* Underline + inversed */
	(BG_LIGHTGREY | FG_BLACK),                /* Bold + underline + inversed */
	(BG_LIGHTGREY | FG_BLACK     | FG_BLINK), /* Blink + inversed */
	(BG_LIGHTGREY | FG_BLACK     | FG_BLINK), /* Bold + blink + inversed */
	(BG_LIGHTGREY | FG_BLACK     | FG_BLINK), /* Underline + blink + inversed */
	(BG_LIGHTGREY | FG_BLACK     | FG_BLINK)  /* Bold + underline + blink + inversed */
};


/* Internal attributes indexes */
enum {
	VT_NORMAL   = 0x00,
	VT_BOLD     = 0x01,
	VT_UNDER    = 0x02,
	VT_BLINK    = 0x04,
	VT_INVERSED = 0x08
};


/* Foreground ANSI color code to PC conversion table */
static const uint8_t fgansitopc[] = {
	FG_BLACK,    /* 0 */
	FG_RED,      /* 1 */
	FG_GREEN,    /* 2 */
	FG_BROWN,    /* 3 */
	FG_BLUE,     /* 4 */
	FG_MAGENTA,  /* 5 */
	FG_CYAN,     /* 6 */
	FG_LIGHTGREY /* 7 */
};


/* Background ANSI color code to PC conversion table */
static const uint8_t bgansitopc[] = {
	BG_BLACK,    /* 0 */
	BG_RED,      /* 1 */
	BG_GREEN,    /* 2 */
	BG_BROWN,    /* 3 */
	BG_BLUE,     /* 4 */
	BG_MAGENTA,  /* 5 */
	BG_CYAN,     /* 6 */
	BG_LIGHTGREY /* 7 */
};


/* ASCII character set */
static const uint16_t ascii[] = {
	0x20, 0x21, 0x22, 0x23, /* 20 */
	0x24, 0x25, 0x26, 0x27, /* 24 */
	0x28, 0x29, 0x2A, 0x2B, /* 28 */
	0x2C, 0x2D, 0x2E, 0x2F, /* 2C */

	0x30, 0x31, 0x32, 0x33, /* 30 */
	0x34, 0x35, 0x36, 0x37, /* 34 */
	0x38, 0x39, 0x3A, 0x3B, /* 38 */
	0x3C, 0x3D, 0x3E, 0x3F, /* 3C */

	0x40, 0x41, 0x42, 0x43, /* 40 */
	0x44, 0x45, 0x46, 0x47, /* 44 */
	0x48, 0x49, 0x4A, 0x4B, /* 48 */
	0x4C, 0x4D, 0x4E, 0x4F, /* 4C */

	0x50, 0x51, 0x52, 0x53, /* 50 */
	0x54, 0x55, 0x56, 0x57, /* 54 */
	0x58, 0x59, 0x5A, 0x5B, /* 58 */
	0x5C, 0x5D, 0x5E, 0x5F, /* 5C */

	0x60, 0x61, 0x62, 0x63, /* 60 */
	0x64, 0x65, 0x66, 0x67, /* 64 */
	0x68, 0x69, 0x6A, 0x6B, /* 68 */
	0x6C, 0x6D, 0x6E, 0x6F, /* 6C */

	0x70, 0x71, 0x72, 0x73, /* 70 */
	0x74, 0x75, 0x76, 0x77, /* 74 */
	0x78, 0x79, 0x7A, 0x7B, /* 78 */
	0x7C, 0x7D, 0x7E, 0x7F  /* 7C */
};


/* Supplemental Graphic character set */
static const uint16_t supg[] = {
	0x20, 0xAD, 0x9B, 0x9C, /* 20 */
	0x20, 0x9D, 0x20, 0x20, /* 24 */
	0x20, 0x20, 0xA6, 0xAE, /* 28 */
	0x20, 0x20, 0x20, 0x20, /* 2C */

	0xF8, 0xF1, 0xFD, 0x20, /* 30 */
	0x20, 0xE6, 0x20, 0x20, /* 34 */
	0x20, 0x20, 0xA7, 0xAF, /* 38 */
	0xAC, 0xAB, 0x20, 0xA8, /* 3C */

	0x20, 0x20, 0x20, 0x20, /* 40 */
	0x8E, 0x8F, 0x92, 0x80, /* 44 */
	0x20, 0x90, 0x20, 0x20, /* 48 */
	0x8D, 0xA1, 0x8C, 0x8B, /* 4C */

	0x20, 0xA5, 0x20, 0x20, /* 50 */
	0x20, 0x20, 0x99, 0x20, /* 54 */
	0x20, 0x20, 0x20, 0x20, /* 58 */
	0x9A, 0x20, 0x20, 0xE1, /* 5C */

	0x85, 0xA0, 0x83, 0x20, /* 60 */
	0x84, 0x86, 0x91, 0x87, /* 64 */
	0x8A, 0x82, 0x88, 0x89, /* 68 */
	0x8D, 0xA1, 0x8C, 0x8B, /* 6C */

	0x20, 0xA4, 0x95, 0xA2, /* 70 */
	0x93, 0x20, 0x94, 0x20, /* 74 */
	0x20, 0x97, 0xA3, 0x96, /* 78 */
	0x81, 0x98, 0x20, 0x20  /* 7C */
};


void _ttypc_vtf_clrparms(ttypc_vt_t *vt)
{
	int i;

	for (i = 0; i < MAXPARMS; i++)
		vt->parms[i] = 0;
	vt->parmi = 0;
}


void _ttypc_vtf_str(ttypc_vt_t *vt)
{
	int i;

	/* Reset screen margins */
	vt->top = 0;
	vt->bottom = vt->rows - 1;

	/* Reset cursor type and visibility */
	vt->cst = 1;
	vt->ctype = CUR_DEF;
	_ttypc_vga_togglecursor(vt, 1);

	/* Reset character processing state and attributes */
	vt->dcsst = DCS_INIT;
	vt->escst = ESC_INIT;
	vt->sgr = VT_NORMAL;
	vt->attr = (uint16_t)((vt->ttypc->color) ? csgr[vt->sgr] : msgr[vt->sgr]) << 8;

	/* Setup tabstops */
	for (i = 0; i < MAXTABS; i++)
		vt->tabs[i] = !(i % 8);

	/* Clear escape sequence parameters */
	_ttypc_vtf_clrparms(vt);

	/* Reset keyboard modes */
	vt->ttypc->lockst = 0;
	if (vt->ttypc->ktype)
		_ttypc_kbd_updateled(vt->ttypc);
	else
		_ttypc_bioskbd_updateled(vt->ttypc);

	/* Reset character processing modes */
	vt->awm = 1;
	vt->om = 0;
	vt->ckm = 0;
	vt->irm = 0;
	vt->arm = 1;
	vt->lnm = 0;
	vt->sc = 0;

	/* Reset character sets */
	vt->G0 = ascii;
	vt->G1 = ascii;
	vt->G2 = supg;
	vt->G3 = supg;
	vt->GL = &vt->G0;
	vt->GR = &vt->G2;
}


void _ttypc_vtf_sc(ttypc_vt_t *vt)
{
	/* Save cursor position */
	vt->scccol = vt->ccol;
	vt->sccrow = vt->crow;
	vt->sccpos = vt->cpos;

	/* Save character sets */
	vt->scG0 = vt->G0;
	vt->scG1 = vt->G1;
	vt->scG2 = vt->G2;
	vt->scG3 = vt->G3;
	vt->scGL = vt->GL;
	vt->scGR = vt->GR;

	/* Save VT100 modes & graphic attributes */
	vt->scawm = vt->awm;
	vt->scom = vt->om;
	vt->scsgr = vt->sgr;
	vt->scattr = vt->attr;

	vt->sc = 1;
}


void _ttypc_vtf_rc(ttypc_vt_t *vt)
{
	if (!vt->sc)
		return;

	/* Restore cursor position */
	vt->ccol = vt->scccol;
	vt->crow = vt->sccrow;
	vt->cpos = vt->sccpos;

	/* Restore character set */
	vt->G0 = vt->scG0;
	vt->G1 = vt->scG1;
	vt->G2 = vt->scG2;
	vt->G3 = vt->scG3;
	vt->GL = vt->scGL;
	vt->GR = vt->scGR;

	/* Restore VT100 modes & graphic attributes */
	vt->awm = vt->scawm;
	vt->om = vt->scom;
	vt->sgr = vt->scsgr;
	vt->attr = vt->scattr;

	vt->sc = 0;
}


void _ttypc_vtf_da(ttypc_vt_t *vt)
{
	/* 62 - class 2 terminal, c - end of attributes list */
	static const char da[] = "\033[62;c";
	ttypc_vt_respond(vt, da);
}


void _ttypc_vtf_aln(ttypc_vt_t *vt)
{
	vt->cpos = 0;
	vt->ccol = 0;
	vt->crow = 0;
	_ttypc_vga_set(vt->vram, vt->attr | 'E' , vt->rows * vt->cols);
}


void _ttypc_vtf_su(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->rows - 1)
		p = vt->rows - 1;

	_ttypc_vga_rollup(vt, p);
}


void _ttypc_vtf_sd(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->rows - 1)
		p = vt->rows - 1;

	_ttypc_vga_rolldown(vt, p);
}


void _ttypc_vtf_cuu(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->crow - vt->top)
		p = vt->crow - vt->top;

	vt->cpos -= vt->cols * p;
}


void _ttypc_vtf_cud(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->bottom - vt->crow)
		p = vt->bottom - vt->crow;

	vt->cpos += vt->cols * p;
}


void _ttypc_vtf_cuf(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->cols - vt->ccol - 1)
		p = vt->cols - vt->ccol - 1;

	vt->cpos += p;
	vt->ccol += p;
}


void _ttypc_vtf_cub(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->ccol)
		p = vt->ccol;

	vt->cpos -= p;
	vt->ccol -= p;
}


void _ttypc_vtf_clreos(ttypc_vt_t *vt)
{
	switch (vt->parms[0]) {
	case 0:
		_ttypc_vga_set(vt->vram + vt->cpos, vt->attr | ' ', vt->cols * vt->rows - vt->cpos);
		break;

	case 1:
		_ttypc_vga_set(vt->vram, vt->attr | ' ', vt->cpos + 1);
		break;

	case 2:
		_ttypc_vga_set(vt->vram, vt->attr | ' ', vt->cols * vt->rows);
		break;
	}
}


void _ttypc_vtf_clreol(ttypc_vt_t *vt)
{
	switch (vt->parms[0]) {
	case 0:
		_ttypc_vga_set(vt->vram + vt->cpos, vt->attr | ' ', vt->cols - vt->ccol);
		break;

	case 1:
		_ttypc_vga_set(vt->vram + vt->cpos - vt->ccol, vt->attr | ' ', vt->ccol + 1);
		break;

	case 2:
		_ttypc_vga_set(vt->vram + vt->cpos - vt->ccol, vt->attr | ' ', vt->cols);
		break;
	}
}


void _ttypc_vtf_curadr(ttypc_vt_t *vt)
{
	if (!vt->parms[0] && !vt->parms[1]) {
		vt->cpos = (vt->om) ? vt->top * vt->cols : 0;
		vt->ccol = 0;
		return;
	}

	if (vt->parms[0] < 1)
		vt->parms[0] = 1;
	else if (vt->parms[0] > ((vt->om) ? vt->bottom - vt->top + 1 : vt->rows))
		vt->parms[0] = (vt->om) ? vt->bottom - vt->top + 1 : vt->rows;

	if (vt->parms[1] < 1)
		vt->parms[1] = 1;
	else if (vt->parms[1] > vt->cols)
		vt->parms[1] = vt->cols;

	vt->cpos = (vt->parms[0] - 1) * vt->cols + vt->parms[1] - 1 + ((vt->om) ? vt->top * vt->cols : 0);
	vt->ccol = vt->parms[1] - 1;
}


void _ttypc_vtf_il(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if ((vt->crow >= vt->top) && (vt->crow <= vt->bottom)) {
		if (p < 1)
			p = 1;
		else if (p > vt->bottom - vt->crow)
			p = vt->bottom - vt->crow;

		vt->cpos -= vt->ccol;
		vt->ccol = 0;

		if (vt->crow == vt->top) {
			_ttypc_vga_rolldown(vt, p);
		}
		else {
			_ttypc_vga_move(vt->vram + vt->cpos + p * vt->cols, vt->vram + vt->cpos, (vt->bottom - vt->crow + 1 - p) * vt->cols);
			_ttypc_vga_set(vt->vram + vt->cpos, vt->attr | ' ', p * vt->cols);
		}
	}
}


void _ttypc_vtf_ic(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->cols - vt->ccol)
		p = vt->cols - vt->ccol;

	_ttypc_vga_move(vt->vram + vt->cpos + p, vt->vram + vt->cpos, vt->cols - vt->ccol - p);
	_ttypc_vga_set(vt->vram + vt->cpos, vt->attr | ' ', p);
}


void _ttypc_vtf_dl(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if ((vt->crow >= vt->top) && (vt->crow <= vt->bottom)) {
		if (p < 1)
			p = 1;
		else if (p > vt->bottom - vt->crow)
			p = vt->bottom - vt->crow;

		vt->cpos -= vt->ccol;
		vt->ccol = 0;

		if (vt->crow == vt->top) {
			_ttypc_vga_rollup(vt, p);
		}
		else {
			_ttypc_vga_move(vt->vram + vt->cpos, vt->vram + vt->cpos + p * vt->cols, (vt->bottom - vt->crow + 1 - p) * vt->cols);
			_ttypc_vga_set(vt->vram + (vt->bottom + 1 - p) * vt->cols, vt->attr | ' ', p * vt->cols);
		}
	}
}


void _ttypc_vtf_dch(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->cols - vt->ccol)
		p = vt->cols - vt->ccol;

	_ttypc_vga_move(vt->vram + vt->cpos, vt->vram + vt->cpos + p, vt->cols - vt->ccol - p);
	_ttypc_vga_set(vt->vram + vt->cpos + vt->cols - p, vt->attr | ' ', p);
}


void _ttypc_vtf_ri(ttypc_vt_t *vt)
{
	if (vt->cpos >= (vt->top + 1) * vt->cols)
		vt->cpos -= vt->cols;
	else
		_ttypc_vga_rolldown(vt, 1);
}


void _ttypc_vtf_ind(ttypc_vt_t *vt)
{
	if (vt->cpos < vt->bottom * vt->cols)
		vt->cpos += vt->cols;
	else
		_ttypc_vga_rollup(vt, 1);
}


void _ttypc_vtf_nel(ttypc_vt_t *vt)
{
	if (vt->cpos < vt->bottom * vt->cols) {
		vt->cpos += vt->cols - vt->ccol;
		vt->ccol = 0;
	}
	else {
		_ttypc_vga_rollup(vt, 1);
		vt->cpos -= vt->ccol;
		vt->ccol = 0;
	}
}


void _ttypc_vtf_clrtab(ttypc_vt_t *vt)
{
	int i;

	if (!vt->parms[0]) {
		vt->tabs[vt->ccol] = 0;
	}
	else if (vt->parms[0] == 3) {
		for (i = 0; i < MAXTABS; i++)
			vt->tabs[i] = 0;
	}
}


void _ttypc_vtf_ris(ttypc_vt_t *vt)
{
	/* Reset cursor and scrollback */
	vt->cpos = 0;
	vt->ccol = 0;
	vt->crow = 0;
	vt->scrbsz = 0;
	vt->scrbpos = 0;

	/* Clear screen */
	_ttypc_vga_set(vt->vram, vt->attr | ' ', vt->cols * vt->rows);
	/* Soft reset */
	_ttypc_vtf_str(vt);
}


/* ECH - erase character */
void _ttypc_vtf_ech(ttypc_vt_t *vt)
{
	int p = vt->parms[0];

	if (p < 1)
		p = 1;
	else if (p > vt->cols - vt->ccol)
		p = vt->cols - vt->ccol;

	_ttypc_vga_set(vt->vram + vt->cpos, vt->attr | ' ', p);
}


void _ttypc_vtf_mc(ttypc_vt_t *vt)
{
}


void _ttypc_vtf_tst(ttypc_vt_t *vt)
{
}


void _ttypc_vtf_sgr(ttypc_vt_t *vt)
{
	ttypc_t *ttypc = vt->ttypc;
	uint16_t attr = vt->attr;
	int cc = 0, i = 0;

	do {
		switch (vt->parms[i++]) {
		case 0: /* Reset to normal attributes */
			vt->sgr = VT_NORMAL;
			cc = 0;
			break;

		case 1: /* Bold */
			vt->sgr |= VT_BOLD;
			break;

		case 4: /* Underline */
			vt->sgr |= VT_UNDER;
			break;

		case 5: /* Blinking */
			vt->sgr |= VT_BLINK;
			break;

		case 7: /* Inversed */
			vt->sgr |= VT_INVERSED;
			break;

		case 22: /* Not bold */
			vt->sgr &= ~VT_BOLD;
			break;

		case 24: /* Not underlined */
			vt->sgr &= ~VT_UNDER;
			break;

		case 25: /* Not blinking */
			vt->sgr &= ~VT_BLINK;
			break;

		case 27: /* Not inversed */
			vt->sgr &= ~VT_INVERSED;
			break;

		case 30: /* Foreground colors */
		case 31:
		case 32:
		case 33:
		case 34:
		case 35:
		case 36:
		case 37:
			if (ttypc->color) {
				cc = 1;
				attr = (attr & 0xf000) | (fgansitopc[(vt->parms[i - 1] - 30) & 7] << 8);
			}
			break;

		case 39: /* Reset foreground color to default */
			if (ttypc->color) {
				cc = 1;
				attr = (attr & 0xf000) | (fgansitopc[FG_LIGHTGREY] << 8);
			}
			break;

		case 40: /* Background colors */
		case 41:
		case 42:
		case 43:
		case 44:
		case 45:
		case 46:
		case 47:
			if (ttypc->color) {
				cc = 1;
				attr = (bgansitopc[(vt->parms[i - 1] - 40) & 7] << 8) | (attr & 0x0f00);
			}
			break;

		case 49: /* Reset background color to default */
			if (ttypc->color) {
				cc = 1;
				attr = (bgansitopc[BG_BLACK] << 8) | (attr & 0x0f00);
			}
			break;

		case 90: /* Bright Foreground colors */
		case 91:
		case 92:
		case 93:
		case 94:
		case 95:
		case 96:
		case 97:
			if (ttypc->color) {
				cc = 1;
				attr = (attr & 0xf000) | (fgansitopc[(vt->parms[i - 1] - 90) & 7] << 8);
				attr |= 0x0800;
			}
			break;

		case 100: /* Bright Background colors */
		case 101:
		case 102:
		case 103:
		case 104:
		case 105:
		case 106:
		case 107:
			if (ttypc->color) {
				cc = 1;
				attr = (bgansitopc[(vt->parms[i - 1] - 100) & 7] << 8) | (attr & 0x0f00);
				attr |= 0x8000;
			}
			break;
		}
	} while (i <= vt->parmi);

	if (ttypc->color != 0) {
		if (cc == 0) {
			attr = csgr[vt->sgr] << 8;
		}

		if ((vt->sgr & VT_BOLD) != 0) {
			if ((vt->sgr & VT_INVERSED) != 0) {
				attr &= ~FG_BRIGHT;
				attr |= BG_BRIGHT;
			}
			else {
				attr |= FG_BRIGHT;
				attr &= ~BG_BRIGHT;
			}
		}

		vt->attr = attr;
	}
	else {
		vt->attr = msgr[vt->sgr] << 8;
	}
}


/* Device status reports */
void _ttypc_vtf_dsr(ttypc_vt_t *vt)
{
	static const char stat[] = "\033[0n";     /* Status */
	static const char print[] = "\033[?13n";  /* Printer Unattached */
	static const char udk[] = "\033[?21n";    /* UDK Locked */
	static const char lang[] = "\033[?27;1n"; /* North American */
	char buff[16];
	int i = 0;

	switch (vt->parms[0]) {
	/* Status */
	case 5:
		ttypc_vt_respond(vt, stat);
		break;

	/* Cursor position */
	case 6:
		buff[i++] = 0x1b;
		buff[i++] = '[';

		if (vt->crow + 1 > 10)
			buff[i++] = '0' + (vt->crow + 1) / 10;
		buff[i++] = '0' + (vt->crow + 1) % 10;
		buff[i++] = ';';

		if (vt->ccol + 1 > 10)
			buff[i++] = '0' + (vt->ccol + 1) / 10;
		buff[i++] = '0' + (vt->ccol + 1) % 10;
		buff[i++] = 'R';
		buff[i++] = '\0';

		ttypc_vt_respond(vt, buff);
		break;

	/* Printer status */
	case 15:
		ttypc_vt_respond(vt, print);
		break;

	/* User Defined Keys status */
	case 25:
		ttypc_vt_respond(vt, udk);
		break;

	/* Language status */
	case 26:
		ttypc_vt_respond(vt, lang);
		break;

	default:
		break;
	}
}


void _ttypc_vtf_stbm(ttypc_vt_t *vt)
{
	/* Both 0 => scrolling region = entire screen */
	if (!vt->parms[0] && !vt->parms[1]) {
		vt->cpos = 0;
		vt->top = 0;
		vt->bottom = vt->rows - 1;
		vt->ccol = 0;
		return;
	}

	if (vt->parms[1] <= vt->parms[0])
		return;

	/* Range parm 1 */
	if (vt->parms[0] < 1)
		vt->parms[0] = 1;
	else if (vt->parms[0] > vt->rows - 1)
		vt->parms[0] = vt->rows - 1;

	/* Range parm 2 */
	if (vt->parms[1] < 2)
		vt->parms[1] = 2;
	else if (vt->parms[1] > vt->rows)
		vt->parms[1] = vt->rows;

	vt->top = vt->parms[0] - 1;
	vt->bottom = vt->parms[1] - 1;

	/* Cursor to origin position */
	vt->cpos = (vt->om) ? vt->top * vt->cols : 0;
	vt->ccol = 0;
}


/* Sets SGR attr to a VGA character */
static void _ttypc_vtf_setattr(volatile uint16_t *ptr, uint16_t attr)
{
	uint16_t nattr = attr;

	/* Keep old fg color if it's different than new bg color */
	if ((*ptr & 0x0f00) ^ ((attr & 0xf000) >> 4))
		nattr = (*ptr & 0x0f00) | (nattr & 0xf0ff);
	/* Keep old bg color if it's different than new fg color */
	if (((*ptr & 0xf000) >> 4) ^ (attr & 0x0f00))
		nattr = (*ptr & 0xf000) | (nattr & 0x0fff);

	*ptr = (*ptr & 0x00ff) | nattr;
}


/* Applies SGR attr to VGA screen buffer */
static void _ttypc_vtf_applyattr(volatile uint16_t *begin, volatile uint16_t *end, uint16_t attr)
{
	for (; begin < end; begin++)
		_ttypc_vtf_setattr(begin, attr);
}


/* Applies SGR mode globally */
static void _ttypc_vtf_applysgr(ttypc_vt_t *vt, uint8_t sgr)
{
	vt->sgr = sgr;
	vt->attr = ((vt->ttypc->color) ? csgr[sgr] : msgr[sgr]) << 8;

	/* Apply attr to vram */
	_ttypc_vtf_applyattr(vt->vram, vt->vram + vt->rows * vt->cols, vt->attr);
	/* Apply attr to scrollback */
	_ttypc_vtf_applyattr(vt->scrb, vt->scrb + vt->scrbsz * vt->cols, vt->attr);
	/* Apply attr to scroll origin */
	if (vt->scrbpos)
		_ttypc_vtf_applyattr(vt->scro, vt->scro + vt->rows * vt->cols, vt->attr);
}


void _ttypc_vtf_setdecpriv(ttypc_vt_t *vt)
{
	switch (vt->parms[0]) {
	case 0:  /* error, ignored */
	case 1:  /* CKM - cursor key mode */
		vt->ckm = 1;
		break;

	case 2:  /* ANM - ansi/vt52 mode */
	case 3:  /* COLM - column mode */
	case 4:  /* SCLM - scrolling mode */
		break;

	case 5:  /* SCNM - screen mode */
		_ttypc_vtf_applysgr(vt, VT_INVERSED);
		break;

	case 6:  /* OM - origin mode */
		vt->om = 1;
		break;

	case 7:  /* AWM - auto wrap mode */
		vt->awm = 1;
		break;

	case 8:  /* ARM - auto repeat mode */
		vt->arm = 1;
		break;

	case 9:  /* INLM - interlace mode */
	case 10: /* EDM - edit mode */
	case 11: /* LTM - line transmit mode */
	case 12: /* ? */
	case 13: /* SCFDM - space compression / field delimiting */
	case 14: /* TEM - transmit execution mode */
	case 15: /* ? */
	case 16: /* EKEM - edit key execution mode */
		break;

	case 25: /* TCEM - text cursor enable mode */
		vt->cst = 1;
		_ttypc_vga_togglecursor(vt, 1);
		break;

	case 42: /* NRCM - 7bit NRC characters */
		break;
	}
}


/* Reset dec private modes, esc [ ? x l */
void _ttypc_vtf_resetdecpriv(ttypc_vt_t *vt)
{
	switch (vt->parms[0]) {
	case 0:  /* error, ignored */
	case 1:  /* CKM - cursor key mode */
		vt->ckm = 0;
		break;

	case 2:  /* ANM - ansi/vt52 mode */
	case 3:  /* COLM - column mode */
	case 4:  /* SCLM - scrolling mode */
		break;

	case 5:  /* SCNM - screen mode */
		_ttypc_vtf_applysgr(vt, VT_NORMAL);
		break;

	case 6:  /* OM - origin mode */
		vt->om = 0;
		break;

	case 7:  /* AWM - auto wrap mode */
		vt->awm = 0;
		break;

	case 8:  /* ARM - auto repeat mode */
		vt->arm = 0;
		break;

	case 9:  /* INLM - interlace mode */
	case 10: /* EDM - edit mode */
	case 11: /* LTM - line transmit mode */
	case 12: /* ? */
	case 13: /* SCFDM - space compression / field delimiting */
	case 14: /* TEM - transmit execution mode */
	case 15: /* ? */
	case 16: /* EKEM - edit key execution mode */
		break;

	case 25: /* TCEM - text cursor enable mode */
		vt->cst = 0;
		_ttypc_vga_togglecursor(vt, 0);
		break;

	case 42: /* NRCM - 7bit NRC characters */
		break;
	}
}


void _ttypc_vtf_setansi(ttypc_vt_t *vt)
{
	switch (vt->parms[0]) {
	case 0:  /* error, ignored */
	case 1:  /* GATM - guarded area transfer mode */
	case 2:  /* KAM - keyboard action mode */
	case 3:  /* CRM - Control Representation mode */
		break;

	case 4:  /* IRM - insert replacement mode */
		vt->irm = 1;
		break;

	case 5:  /* SRTM - status report transfer mode */
	case 6:  /* ERM - erasue mode */
	case 7:  /* VEM - vertical editing mode */
	case 10: /* HEM - horizontal editing mode */
	case 11: /* PUM - position unit mode */
	case 12: /* SRM - send-receive mode */
	case 13: /* FEAM - format effector action mode */
	case 14: /* FETM - format effector transfer mode */
	case 15: /* MATM - multiple area transfer mode */
	case 16: /* TTM - transfer termination */
	case 17: /* SATM - selected area transfer mode */
	case 18: /* TSM - tabulation stop mode */
	case 19: /* EBM - editing boundary mode */
		break;

	case 20: /* LNM - line feed / newline mode */
		vt->lnm = 1;
		break;
	}
}


void _ttypc_vtf_resetansi(ttypc_vt_t *vt)
{
	switch (vt->parms[0]) {
	case 0:  /* error, ignored */
	case 1:  /* GATM - guarded area transfer mode */
	case 2:  /* KAM - keyboard action mode */
	case 3:  /* CRM - Control Representation mode */
		break;

	case 4:  /* IRM - insert replacement mode */
		vt->irm = 0;
		break;

	case 5:  /* SRTM - status report transfer mode */
	case 6:  /* ERM - erasue mode */
	case 7:  /* VEM - vertical editing mode */
	case 10: /* HEM - horizontal editing mode */
	case 11: /* PUM - position unit mode */
	case 12: /* SRM - send-receive mode */
	case 13: /* FEAM - format effector action mode */
	case 14: /* FETM - format effector transfer mode */
	case 15: /* MATM - multiple area transfer mode */
	case 16: /* TTM - transfer termination */
	case 17: /* SATM - selected area transfer mode */
	case 18: /* TSM - tabulation stop mode */
	case 19: /* EBM - editing boundary mode */
		break;

	case 20: /* LNM - line feed / newline mode */
		vt->lnm = 0;
		break;
	}
}


void _ttypc_vtf_reqtparm(ttypc_vt_t *vt)
{
	static const char tparm[] = "\033[3;1;1;120;120;1;0x";
	ttypc_vt_respond(vt, tparm);
}


void _ttypc_vtf_keynum(ttypc_vt_t *vt)
{
	vt->ttypc->lockst |= KB_NUM;
	if (vt->ttypc->ktype)
		_ttypc_kbd_updateled(vt->ttypc);
	else
		_ttypc_bioskbd_updateled(vt->ttypc);
}


void _ttypc_vtf_keyappl(ttypc_vt_t *vt)
{
	vt->ttypc->lockst &= ~KB_NUM;
	if (vt->ttypc->ktype)
		_ttypc_kbd_updateled(vt->ttypc);
	else
		_ttypc_bioskbd_updateled(vt->ttypc);
}
