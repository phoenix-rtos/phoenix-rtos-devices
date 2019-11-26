/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ttypc VT220 emulator (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2012, 2018, 2019 Phoenix Systems
 * Copyright 2007-2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/threads.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ttypc.h"
#include "ttypc_vtf.h"
#include "ttypc_vga.h"


uint16_t csd_ascii[CSSIZE] = {
/* 20 */	0x20 | CSL, 0x21 | CSL, 0x22 | CSL, 0x23 | CSL,
/* 24 */	0x24 | CSL, 0x25 | CSL, 0x26 | CSL, 0x27 | CSL,
/* 28 */	0x28 | CSL, 0x29 | CSL, 0x2A | CSL, 0x2B | CSL,
/* 2C */	0x2C | CSL, 0x2D | CSL, 0x2E | CSL, 0x2F | CSL,

/* 30 */	0x30 | CSL, 0x31 | CSL, 0x32 | CSL, 0x33 | CSL,
/* 34 */	0x34 | CSL, 0x35 | CSL, 0x36 | CSL, 0x37 | CSL,
/* 38 */	0x38 | CSL, 0x39 | CSL, 0x3A | CSL, 0x3B | CSL,
/* 3C */	0x3C | CSL, 0x3D | CSL, 0x3E | CSL, 0x3F | CSL,

/* 40 */	0x40 | CSL, 0x41 | CSL, 0x42 | CSL, 0x43 | CSL,
/* 44 */	0x44 | CSL, 0x45 | CSL, 0x46 | CSL, 0x47 | CSL,
/* 48 */	0x48 | CSL, 0x49 | CSL, 0x4A | CSL, 0x4B | CSL,
/* 4C */	0x4C | CSL, 0x4D | CSL, 0x4E | CSL, 0x4F | CSL,

/* 50 */	0x50 | CSL, 0x51 | CSL, 0x52 | CSL, 0x53 | CSL,
/* 54 */	0x54 | CSL, 0x55 | CSL, 0x56 | CSL, 0x57 | CSL,
/* 58 */	0x58 | CSL, 0x59 | CSL, 0x5A | CSL, 0x5B | CSL,
/* 5C */	0x5C | CSL, 0x5D | CSL, 0x5E | CSL, 0x5F | CSL,

/* 60 */	0x60 | CSL, 0x61 | CSL, 0x62 | CSL, 0x63 | CSL,
/* 64 */	0x64 | CSL, 0x65 | CSL, 0x66 | CSL, 0x67 | CSL,
/* 68 */	0x68 | CSL, 0x69 | CSL, 0x6A | CSL, 0x6B | CSL,
/* 6C */	0x6C | CSL, 0x6D | CSL, 0x6E | CSL, 0x6F | CSL,

/* 70 */	0x70 | CSL, 0x71 | CSL, 0x72 | CSL, 0x73 | CSL,
/* 74 */	0x74 | CSL, 0x75 | CSL, 0x76 | CSL, 0x77 | CSL,
/* 78 */	0x78 | CSL, 0x79 | CSL, 0x7A | CSL, 0x7B | CSL,
/* 7C */	0x7C | CSL, 0x7D | CSL, 0x7E | CSL, 0x7F | CSL,
};


/* DEC Supplemental Graphic Characterset */
uint16_t csd_supplemental[CSSIZE] = {
/* 20 */	0x20 | CSL, 0xAD | CSL, 0x9B | CSL, 0x9C | CSL,
/* 24 */	0x20 | CSL, 0x9D | CSL, 0x20 | CSL, 0x20 | CSL,
/* 28 */	0x20 | CSL, 0x20 | CSL, 0xA6 | CSL, 0xAE | CSL,
/* 2C */	0x20 | CSL, 0x20 | CSL, 0x20 | CSL, 0x20 | CSL,

/* 30 */	0xF8 | CSL, 0xF1 | CSL, 0xFD | CSL, 0x20 | CSL,
/* 34 */	0x20 | CSL, 0xE6 | CSL, 0x20 | CSL, 0x20 | CSL,
/* 38 */	0x20 | CSL, 0x20 | CSL, 0xA7 | CSL, 0xAF | CSL,
/* 3C */	0xAC | CSL, 0xAB | CSL, 0x20 | CSL, 0xA8 | CSL,

/* 40 */	0x20 | CSL, 0x20 | CSL, 0x20 | CSL, 0x20 | CSL,
/* 44 */	0x8E | CSL, 0x8F | CSL, 0x92 | CSL, 0x80 | CSL,
/* 48 */	0x20 | CSL, 0x90 | CSL, 0x20 | CSL, 0x20 | CSL,
/* 4C */	0x8D | CSL, 0xA1 | CSL, 0x8C | CSL, 0x8B | CSL,

/* 50 */	0x20 | CSL, 0xA5 | CSL, 0x20 | CSL, 0x20 | CSL,
/* 54 */	0x20 | CSL, 0x20 | CSL, 0x99 | CSL, 0x20 | CSL,
/* 58 */	0x20 | CSL, 0x20 | CSL, 0x20 | CSL, 0x20 | CSL,
/* 5C */	0x9A | CSL, 0x20 | CSL, 0x20 | CSL, 0xE1 | CSL,

/* 60 */	0x85 | CSL, 0xA0 | CSL, 0x83 | CSL, 0x20 | CSL,
/* 64 */	0x84 | CSL, 0x86 | CSL, 0x91 | CSL, 0x87 | CSL,
/* 68 */	0x8A | CSL, 0x82 | CSL, 0x88 | CSL, 0x89 | CSL,
/* 6C */	0x8D | CSL, 0xA1 | CSL, 0x8C | CSL, 0x8B | CSL,

/* 70 */	0x20 | CSL, 0xA4 | CSL, 0x95 | CSL, 0xA2 | CSL,
/* 74 */	0x93 | CSL, 0x20 | CSL, 0x94 | CSL, 0x20 | CSL,
/* 78 */	0x20 | CSL, 0x97 | CSL, 0xA3 | CSL, 0x96 | CSL,
/* 7C */	0x81 | CSL, 0x98 | CSL, 0x20 | CSL, 0x20 | CSL
};


/* character is a control character */
#define CTL_VALID(c) ((unsigned char)(c) < 0x20 || (c) == 0x7f)


static void set_baudrate(void *_virt, speed_t baud)
{
	/* TODO */
}


static void set_cflag(void *_virt, tcflag_t *cflag)
{
	/* TODO */
}


static void signal_txready(void *_virt)
{
	ttypc_virt_t *virt = (ttypc_virt_t *)_virt;

	while (libtty_txready(&virt->tty))
		ttypc_virt_sput(virt, libtty_getchar(&virt->tty, NULL));
}


#if 0
static int check_scrollback(video_state_t *svsp)
{
	/* still waiting for scrollback memory or not on current page */
	if (!svsp->scrollback || svsp != vsp)
		return 0;

	/* remove first line of scrollback buffer to make room for new line */
	if (svsp->scr_offset == svsp->max_off)
		hal_memcpy(svsp->Scrollback, svsp->Scrollback + svsp->maxcol, svsp->maxcol * svsp->max_off * CHR);

	/* still room left, increase scroll offset (lines) */
	else
		svsp->scr_offset++;

	return 1;
}
#endif


/* check if we must scroll up screen */
static void _ttypc_virt_scroll(ttypc_virt_t *virt)
{		
	if (virt->cur_offset >= (virt->scrr_end + 1) * virt->maxcol) {
		_ttypc_vga_rollup(virt, 1);
		virt->cur_offset -= virt->maxcol;
	}
}


#define video (virt->vram + virt->cur_offset)


/* Function put char c to the screen according to video state given by svsp */
static void _ttypc_virt_writechar(ttypc_virt_t *virt, uint16_t attrib, uint16_t c)
{
	if ((c >= 0x20) && (c <= 0x7f)) {               /* use GL if ch >= 0x20 */
		*video = attrib | (*virt->GL)[c - 0x20];

		if (virt->ss)
			virt->ss = 0;
	}
	else {
		virt->ss = 0;

		if (c >= 0xa0)                             /* display controls C1 */
			*video = attrib | (*virt->GR)[c - 0xa0];
		else                                       /* display controls C0 */
			*video = attrib | c;
	}
}


int ttypc_virt_sput(ttypc_virt_t *virt, char c)
{
	int ret = 1;

	mutexLock(virt->mutex);

	/* always process control-chars in the range 0x00..0x1f, 0x7f !!! */
	if (CTL_VALID(c)) {

		switch (c) {
		case 0x00:  /* NUL */
		case 0x01:  /* SOH */
		case 0x02:  /* STX */
		case 0x03:  /* ETX */
		case 0x04:  /* EOT */
		case 0x05:  /* ENQ */
		case 0x06:  /* ACK */
		case 0x07:  /* BEL */
			break;

		case 0x08:  /* BS  */
			if (virt->col) {
				virt->cur_offset--;
				virt->col--;
			}
			ret = 0;
			break;

		case 0x09:  /* TAB */
			while (virt->col < virt->maxcol - 1) {
				virt->cur_offset++;
				if (virt->tab_stops[++virt->col])
					break;
			}
			break;

		case 0x0a:  /* LF */
		case 0x0b:  /* VT */
		case 0x0c:  /* FF */
#if 0
			if (ttypc_virt_scrollback(virt)) {
				extra = (svsp->cur_offset % svsp->maxcol) ? svsp->col : 0;
				hal_memcpy(svsp->scrollback + svsp->scr_offset * svsp->maxcol,
				           svsp->crtat + svsp->cur_offset - extra,
				           svsp->maxcol * CHR);
			}
#endif
			virt->cur_offset += virt->maxcol;

			_ttypc_virt_scroll(virt);
			break;

		case 0x0d:  /* CR */
			virt->cur_offset -= virt->col;
			virt->col = 0;
			break;

		case 0x0e:  /* SO */
			virt->GL = &virt->G1;
			break;
		case 0x0f:  /* SI */
			virt->GL = &virt->G0;
			break;

		case 0x10:  /* DLE */
		case 0x11:  /* DC1/XON */
		case 0x12:  /* DC2 */
		case 0x13:  /* DC3/XOFF */
		case 0x14:  /* DC4 */
		case 0x15:  /* NAK */
		case 0x16:  /* SYN */
		case 0x17:  /* ETB */
			break;

		case 0x18:  /* CAN */
			virt->state = STATE_INIT;
			_ttypc_vtf_clrparms(virt);
			break;
		
		case 0x19:  /* EM */
			break;

		case 0x1a:  /* SUB */
			virt->state = STATE_INIT;
			_ttypc_vtf_clrparms(virt);
			break;

		case 0x1b:  /* ESC */
			virt->state = STATE_ESC;
			_ttypc_vtf_clrparms(virt);
			break;

		case 0x1c:  /* FS */
		case 0x1d:  /* GS */
		case 0x1e:  /* RS */
		case 0x1f:  /* US */
		case 0x7f:  /* DEL */
			break;
		}
	}
	
	/* char range 0x20...0x73, 0x80...0xff processing depends on current state */
	else {
		switch (virt->state) {

		case STATE_INIT:
			if (virt->lastchar && virt->m_awm && (virt->lastrow == virt->row)) {
				virt->cur_offset++;
				virt->col = 0;
				virt->lastchar = 0;
#if 0
				if (_ttypc_virt_scrollback(virt)) {
					hal_memcpy(virt->scrollback + (virt->scr_offset * virt->maxcol),
					           virt->vram + virt->cur_offset - virt->maxcol,
					           virt->maxcol * CHR);
				}
#endif
				_ttypc_virt_scroll(virt);
			}

			if (virt->m_irm)
				memcpy(virt->vram + virt->cur_offset + 1, virt->vram + virt->cur_offset, (virt->maxcol - 1 - virt->col) * CHR);

			_ttypc_virt_writechar(virt, virt->attr, (uint16_t)c);
			//_ttypc_vtf_selattr(virt);

			if (virt->col >= virt->maxcol - 1) {
				virt->lastchar = 1;
				virt->lastrow = virt->row;
			}
			else {
				virt->lastchar = 0;
				virt->cur_offset++;
				virt->col++;
			}
			break;

		case STATE_ESC:

			switch(c) {
			case ' ':                             /* ESC sp family */
				virt->state = STATE_BLANK;
				break;
			case '#':                             /* ESC # family */
				virt->state = STATE_HASH;
				break;
			case '&':                             /* ESC & family (HP) */
				virt->state = STATE_INIT;
				break;
			case '(':                             /* ESC ( family */
				virt->state = STATE_BROPN;
				break;
			case ')':                             /* ESC ) family */
				virt->state = STATE_BRCLO;
				break;
			case '*':                             /* ESC * family */
				virt->state = STATE_STAR;
				break;
			case '+':                             /* ESC + family */
				virt->state = STATE_PLUS;
				break;
			case '-':                             /* ESC - family */
				virt->state = STATE_MINUS;
				break;
			case '.':                             /* ESC . family */
				virt->state = STATE_DOT;
				break;
			case '/':                             /* ESC / family */
				virt->state = STATE_SLASH;
				break;
			case '7':                             /* SAVE CURSOR */
				_ttypc_vtf_sc(virt);
				virt->state = STATE_INIT;
				break;
			case '8':                             /* RESTORE CURSOR */
				_ttypc_vtf_rc(virt);
				virt->state = STATE_INIT;
				break;
			case '=':                             /* keypad application mode */
				_ttypc_vtf_keyappl(virt);
				virt->state = STATE_INIT;
				break;
			case '>':                             /* keypad numeric mode */
				_ttypc_vtf_keynum(virt);
				virt->state = STATE_INIT;
				break;
			case 'D':                             /* INDEX */
				_ttypc_vtf_ind(virt);
				virt->state = STATE_INIT;
				break;
			case 'E':                             /* NEXT LINE */
				_ttypc_vtf_nel(virt);
				virt->state = STATE_INIT;
				break;
			case 'H':                             /* set TAB at current col */
				virt->tab_stops[virt->col] = 1;
				virt->state = STATE_INIT;
				break;
			case 'M':                             /* REVERSE INDEX */
				_ttypc_vtf_ri(virt);
				virt->state = STATE_INIT;
				break;
			case 'N':                             /* SINGLE SHIFT G2 */
				virt->Gs = &virt->G2;
				virt->ss = 1;
				virt->state = STATE_INIT;
				break;
			case 'O':                             /* SINGLE SHIFT G3 */
				virt->Gs = &virt->G3;
				virt->ss = 1;
				virt->state = STATE_INIT;
				break;
#if 0
			case 'P':                             /* DCS detected */
				virt->dcs_state = DCS_INIT;
				virt->state = STATE_DCS;
				break;
#endif
			case 'Z':                             /* What are you = ESC [ c */
				_ttypc_vtf_da(virt);
				virt->state = STATE_INIT;
				break;
			case '[':                             /* CSI detected */
				_ttypc_vtf_clrparms(virt);
				virt->state = STATE_CSI;
				break;
			case '\\':                            /* String Terminator */
				virt->state = STATE_INIT;
				break;
			case 'c':                             /* hard reset */
				_ttypc_vtf_ris(virt);
				virt->state = STATE_INIT;
				break;
			case 'd':                             /* set color sgr */
				virt->state = STATE_INIT;
				break;
			case 'n':                             /* lock Shift G2 -> GL */
				virt->GL = &virt->G2;
				virt->state = STATE_INIT;
				break;
			case 'o':                             /* Lock Shift G3 -> GL */
				virt->GL = &virt->G3;
				virt->state = STATE_INIT;
				break;
			case '}':                             /* Lock Shift G2 -> GR */
				virt->GR = &virt->G2;
				virt->state = STATE_INIT;
				break;
			case '|':                             /* Lock Shift G3 -> GR */
				virt->GR = &virt->G3;
				virt->state = STATE_INIT;
				break;
			case '~':                             /* Lock Shift G1 -> GR */
				virt->GR = &virt->G1;
				virt->state = STATE_INIT;
				break;
			default:
				virt->state = STATE_INIT;
				break;
			}
			break;

		case STATE_BLANK:                         /* ESC space [FG], which are */
			virt->state = STATE_INIT;             /* currently ignored*/
			break;
		
		case STATE_HASH:
			switch(c) {
			case '3':                             /* double height top half */
			case '4':                             /* double height bottom half */
			case '5':                             /* single width sngle height */
			case '6':                             /* double width sngle height */
			case '8':                             /* fill sceen with 'E's */
			default:                              /* anything else */
				virt->state = STATE_INIT;
				break;
			}
			break;

		case STATE_BROPN:                       /* designate G0 */
		case STATE_BRCLO:                       /* designate G1 */
		case STATE_STAR:                        /* designate G2 */
		case STATE_PLUS:                        /* designate G3 */
		case STATE_MINUS:                       /* designate G1 (96) */
		case STATE_DOT:                         /* designate G2 (96) */
		case STATE_SLASH:                       /* designate G3 (96) */
			virt->state = STATE_INIT;
			break;

		case STATE_CSI:
			switch (c) {

			/* parameters */
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				virt->parms[virt->parmi] *= 10;
				virt->parms[virt->parmi] += (c - '0');
				break;
			case ';':
				virt->parmi = (virt->parmi + 1 < MAXPARMS) ? virt->parmi + 1 : virt->parmi;
				break;

			case '@':                             /* insert char */
				_ttypc_vtf_ic(virt);
				virt->state = STATE_INIT;
				break;
			case '"':                             /* select char attribute */
				virt->state = STATE_SCA;
				break;
			case '!':                             /* soft terminal reset */
				virt->state = STATE_STR;
				break;

			case 'A':                             /* cursor up */
				_ttypc_vtf_cuu(virt);
				virt->state = STATE_INIT;
				break;
			case 'B':                             /* cursor down */
				_ttypc_vtf_cud(virt);
				virt->state = STATE_INIT;
				break;
			case 'C':                             /* cursor forward */
				_ttypc_vtf_cuf(virt);
				virt->state = STATE_INIT;
				break;
			case 'D':                             /* cursor backward */
				_ttypc_vtf_cub(virt);
				virt->state = STATE_INIT;
				break;
			case 'H':                             /* direct cursor addressing*/
				_ttypc_vtf_curadr(virt);
				virt->state = STATE_INIT;
				break;

			case 'J':                             /* erase screen */
				_ttypc_vtf_clreos(virt);
				virt->state = STATE_INIT;
				break;
			case 'K':                             /* erase line */
				_ttypc_vtf_clreol(virt);
				virt->state = STATE_INIT;
				if (virt->scr_offset > 0 && virt->active)
					virt->scr_offset--;
				break;

			case 'L':                             /* insert line */
				_ttypc_vtf_il(virt);
				virt->state = STATE_INIT;
				break;
			case 'M':                             /* delete line */
				_ttypc_vtf_dl(virt);
				virt->state = STATE_INIT;
				break;
			case 'P':                             /* delete character */
				_ttypc_vtf_dch(virt);
				virt->state = STATE_INIT;
				break;

			case 'S':                             /* scroll up */
				_ttypc_vtf_su(virt);
				virt->state = STATE_INIT;
				break;
			case 'T':	                          /* scroll down */
				_ttypc_vtf_sd(virt);
				virt->state = STATE_INIT;
				break;

			case 'X':                             /* erase character */
				_ttypc_vtf_ech(virt);
				virt->state = STATE_INIT;
				break;

			case 'c':                             /* device attributes */
				_ttypc_vtf_da(virt);
				virt->state = STATE_INIT;
				break;

			case 'f':                             /* direct cursor addressing*/
				_ttypc_vtf_curadr(virt);
				virt->state = STATE_INIT;
				break;

			case 'g':                             /* clear tabs */
				_ttypc_vtf_clrtab(virt);
				virt->state = STATE_INIT;
				break;

			case 'h':                             /* set mode(s) */
				_ttypc_vtf_set_ansi(virt);
				virt->state = STATE_INIT;
				break;

			case 'i':                             /* media copy */
				_ttypc_vtf_mc(virt);
				virt->state = STATE_INIT;
				break;

			case 'l':                             /* reset mode(s) */
				_ttypc_vtf_reset_ansi(virt);
				virt->state = STATE_INIT;
				break;

			case 'm':                             /* select graphic rendition*/
				_ttypc_vtf_sgr(virt);
				virt->state = STATE_INIT;
				break;

			case 'n':                             /* reports */
				_ttypc_vtf_dsr(virt);
				virt->state = STATE_INIT;
				break;

			case 'r':                             /* set scrolling region */
				_ttypc_vtf_stbm(virt);
				virt->state = STATE_INIT;
				break;

			case 'x':                             /* request/report parameters */
				_ttypc_vtf_reqtparm(virt);
				virt->state = STATE_INIT;
				break;

			case 'y':                             /* invoke selftest(s) */
				_ttypc_vtf_tst(virt);
				virt->state = STATE_INIT;
				break;

			default:
				virt->state = STATE_INIT;
				break;
			}
			break;

#if 0
		case STATE_DCS:
				_ttypc_vtf_dcsentry(c, svsp);
				break;
#endif

		case STATE_SCA:
			switch(c) {
			case 'q':
				_ttypc_vtf_sca(virt);
				virt->state = STATE_INIT;
				break;
			default:
				virt->state = STATE_INIT;
				break;
			}
			break;

		/* soft terminal reset */
		case STATE_STR:
			switch(c) {
				case 'p':
					_ttypc_vtf_str(virt);
					virt->state = STATE_INIT;
					break;
				default:
					virt->state = STATE_INIT;
					break;
				}
				break;

		default:
			virt->state = STATE_INIT;
			break;
		}
	}

	virt->row = virt->cur_offset / virt->maxcol;

	/* take care of last character on line behaviour */
	if (virt->lastchar && (virt->col < (virt->maxcol - 1)))
		virt->lastchar = 0;

	/* update cursor */
	if (virt->active)
		_ttypc_vga_cursor(virt);

	mutexUnlock(virt->mutex);

	return ret;
}


int ttypc_virt_swrite(ttypc_virt_t *virt, char *buff, size_t len)
{
	size_t i;
	int ret = 0;

	for (i = 0; i < len; i++)
		ret += ttypc_virt_sput(virt, *(buff + i));

	return ret;
}


int ttypc_virt_sadd(ttypc_virt_t *virt, char *buff, size_t len, int mode)
{
	return libtty_write(&virt->tty, buff, len, mode);
}


int ttypc_virt_sget(ttypc_virt_t *virt, char *buff, size_t len, int mode)
{
	return libtty_read(&virt->tty, buff, len, mode);
}


int ttypc_virt_poll_status(ttypc_virt_t *virt)
{
	return libtty_poll_status(&virt->tty);
}


int ttypc_virt_ioctl(ttypc_virt_t *virt, pid_t sender_pid, unsigned int cmd, const void *in_arg, const void **out_arg)
{
	return libtty_ioctl(&virt->tty, sender_pid, cmd, in_arg, out_arg);
}


int _ttypc_virt_init(ttypc_virt_t *virt, unsigned int bufsize, ttypc_t *ttypc)
{
	libtty_callbacks_t callbacks;

	callbacks.arg = virt;
	callbacks.set_baudrate = set_baudrate;
	callbacks.set_cflag = set_cflag;
	callbacks.signal_txready = signal_txready;

	libtty_init(&virt->tty, &callbacks, bufsize);

	virt->active = 0;

	/*
	 * (MOD) add mapping attributes
	 */
	virt->ttypc = ttypc;
	virt->mem = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_PRIVATE, NULL, 0);
	virt->memsz = 4096;
	virt->attr = 0x7 << 8;

	virt->vram = virt->mem;
	memsetw(virt->vram, ' ' | virt->attr, virt->memsz / 2);

	virt->rows = 25;
	virt->maxcol = 80;

	virt->cur_offset = 0;
	virt->state = STATE_INIT;

	virt->ss = 0;
	virt->Gs = NULL;

	mutexCreate(&virt->mutex);

	/* init emulator */
	_ttypc_vtf_str(virt);

	return 0;
}
