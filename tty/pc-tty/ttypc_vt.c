/*
 * Phoenix-RTOS
 *
 * Virtual Terminal (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2007-2008 Pawel Pisarczyk
 * Copyright 2012, 2018, 2019, 2020, 2023 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/threads.h>

#include "ttypc_vga.h"
#include "ttypc_vt.h"
#include "ttypc_vtf.h"


/* Writes character to screen buffer */
static void _ttypc_vt_sdraw(ttypc_vt_t *vt, char c)
{
	if ((c >= 0x20) && (c <= 0x7f))
		c = (vt->ss) ? (*vt->Gs)[c - 0x20] : (*vt->GL)[c - 0x20];
	else if (c >= 0xa0)
		c = (*vt->GR)[c - 0xa0];

	*(vt->vram + vt->cpos) = vt->attr | c;
	vt->ss = 0;
}


static void _ttypc_vt_sput(ttypc_vt_t *vt, char c)
{
	/* Cancel scrolling */
	_ttypc_vga_scrollcancel(vt);

	/* Process control character */
	if ((c < 0x20) || (c == 0x7f)) {
		switch (c) {
		case 0x00: /* NUL */
		case 0x01: /* SOH */
		case 0x02: /* STX */
		case 0x03: /* ETX */
		case 0x04: /* EOT */
		case 0x05: /* ENQ */
		case 0x06: /* ACK */
		case 0x07: /* BEL */
			break;

		case 0x08: /* BS  */
			if (vt->ccol) {
				vt->ccol--;
				vt->cpos--;
			}
			else {
				_ttypc_vtf_ri(vt);
				vt->ccol = vt->cols - 1;
				vt->cpos += vt->cols - 1;
			}
			break;

		case 0x09: /* TAB */
			while (vt->ccol < vt->cols - 1) {
				vt->cpos++;
				if (vt->tabs[++vt->ccol])
					break;
			}
			break;

		case 0x0a: /* LF */
		case 0x0b: /* VT */
		case 0x0c: /* FF */
			if (!vt->lc) {
				if (vt->lnm) {
					vt->cpos -= vt->ccol;
					vt->ccol = 0;
				}
				vt->cpos += vt->cols;
			}
			vt->lc = 0;
			break;

		case 0x0d: /* CR */
			vt->cpos -= vt->ccol;
			vt->ccol = 0;
			break;

		case 0x0e: /* SO */
			vt->GL = &vt->G1;
			break;

		case 0x0f: /* SI */
			vt->GL = &vt->G0;
			break;

		case 0x10: /* DLE */
		case 0x11: /* DC1/XON */
		case 0x12: /* DC2 */
		case 0x13: /* DC3/XOFF */
		case 0x14: /* DC4 */
		case 0x15: /* NAK */
		case 0x16: /* SYN */
		case 0x17: /* ETB */
			break;

		case 0x18: /* CAN */
			vt->escst = ESC_INIT;
			_ttypc_vtf_clrparms(vt);
			break;

		case 0x19: /* EM */
			break;

		case 0x1a: /* SUB */
			vt->escst = ESC_INIT;
			_ttypc_vtf_clrparms(vt);
			break;

		case 0x1b: /* ESC */
			vt->escst = ESC_ESC;
			_ttypc_vtf_clrparms(vt);
			break;

		case 0x1c: /* FS */
		case 0x1d: /* GS */
		case 0x1e: /* RS */
		case 0x1f: /* US */
		case 0x7f: /* DEL */
			break;
		}
	}
	/* Process character according to escape sequence state */
	else {
		switch (vt->escst) {
		case ESC_INIT:
			/* InseRt Mode */
			if (vt->irm)
				_ttypc_vga_move(vt->vram + vt->cpos + 1, vt->vram + vt->cpos, vt->cols - vt->ccol - 1);

			_ttypc_vt_sdraw(vt, c);

			vt->lc = 0;
			if (vt->ccol < vt->cols - 1) {
				vt->ccol++;
				vt->cpos++;
			}
			else if (vt->awm) {
				vt->ccol = 0;
				vt->cpos++;
				vt->lc = 1;
			}
			break;

		case ESC_ESC:
			switch (c) {
			case ' ':   /* ESC space family */
				vt->escst = ESC_BLANK;
				break;

			case '#':   /* ESC # family */
				vt->escst = ESC_HASH;
				break;

			case '&':   /* ESC & family (HP) */
				vt->escst = ESC_INIT;
				break;

			case '(':   /* ESC ( family */
				vt->escst = ESC_BROPN;
				break;

			case ')':   /* ESC ) family */
				vt->escst = ESC_BRCLO;
				break;

			case '*':   /* ESC * family */
				vt->escst = ESC_STAR;
				break;

			case '+':   /* ESC + family */
				vt->escst = ESC_PLUS;
				break;

			case '-':   /* ESC - family */
				vt->escst = ESC_MINUS;
				break;

			case '.':   /* ESC . family */
				vt->escst = ESC_DOT;
				break;

			case '/':   /* ESC / family */
				vt->escst = ESC_SLASH;
				break;

			case '7':   /* Save Cursor & Attributes */
				_ttypc_vtf_sc(vt);
				vt->escst = ESC_INIT;
				break;

			case '8':   /* Restore cursor & Attributes */
				_ttypc_vtf_rc(vt);
				vt->escst = ESC_INIT;
				break;

			case '=':   /* Keypad application mode */
				_ttypc_vtf_keyappl(vt);
				vt->escst = ESC_INIT;
				break;

			case '>':   /* Keypad numeric mode */
				_ttypc_vtf_keynum(vt);
				vt->escst = ESC_INIT;
				break;

			case 'D':   /* Index */
				_ttypc_vtf_ind(vt);
				vt->escst = ESC_INIT;
				break;

			case 'E':   /* Next line */
				_ttypc_vtf_nel(vt);
				vt->escst = ESC_INIT;
				break;

			case 'H':   /* Set TAB at current col */
				vt->tabs[vt->ccol] = 1;
				vt->escst = ESC_INIT;
				break;

			case 'M':   /* Reverse index */
				_ttypc_vtf_ri(vt);
				vt->escst = ESC_INIT;
				break;

			case 'N':   /* Single shift G2 */
				vt->Gs = &vt->G2;
				vt->ss = 1;
				vt->escst = ESC_INIT;
				break;

			case 'O':   /* Single shift G3 */
				vt->Gs = &vt->G3;
				vt->ss = 1;
				vt->escst = ESC_INIT;
				break;

			case 'P':   /* DCS detected */
				vt->dcsst = DCS_INIT;
				vt->escst = ESC_DCS;
				break;

			case 'Z':   /* Terminal attributes */
				_ttypc_vtf_da(vt);
				vt->escst = ESC_INIT;
				break;

			case '[':   /* CSI detected */
				_ttypc_vtf_clrparms(vt);
				vt->escst = ESC_CSI;
				break;

			case '\\':  /* String terminator */
				vt->escst = ESC_INIT;
				break;

			case 'c':   /* Hard reset */
				_ttypc_vtf_ris(vt);
				vt->escst = ESC_INIT;
				break;

			case 'd':   /* Set color sgr */
				vt->escst = ESC_INIT;
				break;

			case 'n':   /* Lock shift G2 -> GL */
				vt->GL = &vt->G2;
				vt->escst = ESC_INIT;
				break;

			case 'o':   /* Lock shift G3 -> GL */
				vt->GL = &vt->G3;
				vt->escst = ESC_INIT;
				break;

			case '}':   /* Lock shift G2 -> GR */
				vt->GR = &vt->G2;
				vt->escst = ESC_INIT;
				break;

			case '|':   /* Lock shift G3 -> GR */
				vt->GR = &vt->G3;
				vt->escst = ESC_INIT;
				break;

			case '~':   /* Lock shift G1 -> GR */
				vt->GR = &vt->G1;
				vt->escst = ESC_INIT;
				break;

			default:
				vt->escst = ESC_INIT;
				break;
			}
			break;

		case ESC_BLANK: /* ESC space */
			vt->escst = ESC_INIT;
			break;

		case ESC_HASH:
			switch (c) {
			case '3':   /* Double height top half */
			case '4':   /* Double height bottom half */
			case '5':   /* Single width single height */
			case '6':   /* Double width single height */
				vt->escst = ESC_INIT;
				break;

			case '8':   /* Fill screen with 'E's */
				_ttypc_vtf_aln(vt);
				vt->escst = ESC_INIT;
				break;

			default:    /* Anything else */
				vt->escst = ESC_INIT;
				break;
			}
			break;

		case ESC_BROPN: /* Designate G0 */
		case ESC_BRCLO: /* Designate G1 */
		case ESC_STAR:  /* Designate G2 */
		case ESC_PLUS:  /* Designate G3 */
		case ESC_MINUS: /* Designate G1 (96) */
		case ESC_DOT:   /* Designate G2 (96) */
		case ESC_SLASH: /* Designate G3 (96) */
			/* TODO: custom character sets support */
			vt->escst = ESC_INIT;
			break;

		case ESC_CSI:
			switch (c) {
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':   /* Parameters */
				vt->parms[vt->parmi] *= 10;
				vt->parms[vt->parmi] += (c - '0');
				break;

			case ';':   /* Next parameter */
				if (vt->parmi + 1 < MAXPARMS)
					vt->parmi++;
				break;

			case '?':   /* ESC [ ? family */
				vt->escst = ESC_CSIQM;
				break;

			case '@':   /* Insert char */
				_ttypc_vtf_ic(vt);
				vt->escst = ESC_INIT;
				break;

			case '"':   /* Select char attribute */
				vt->escst = ESC_SCA;
				break;

			case '!':   /* Soft terminal reset */
				vt->escst = ESC_STR;
				break;

			case 'A':   /* Cursor up */
				_ttypc_vtf_cuu(vt);
				vt->escst = ESC_INIT;
				break;

			case 'B':   /* Cursor down */
				_ttypc_vtf_cud(vt);
				vt->escst = ESC_INIT;
				break;

			case 'C':   /* Cursor forward */
				_ttypc_vtf_cuf(vt);
				vt->escst = ESC_INIT;
				break;

			case 'D':   /* Cursor backward */
				_ttypc_vtf_cub(vt);
				vt->escst = ESC_INIT;
				break;

			case 'H':   /* Direct cursor addressing */
				_ttypc_vtf_curadr(vt);
				vt->escst = ESC_INIT;
				break;

			case 'J':   /* Erase screen */
				_ttypc_vtf_clreos(vt);
				vt->escst = ESC_INIT;
				break;

			case 'K':   /* Erase line */
				_ttypc_vtf_clreol(vt);
				vt->escst = ESC_INIT;
				break;

			case 'L':   /* Insert line */
				_ttypc_vtf_il(vt);
				vt->escst = ESC_INIT;
				break;

			case 'M':   /* Delete line */
				_ttypc_vtf_dl(vt);
				vt->escst = ESC_INIT;
				break;

			case 'P':   /* Delete character */
				_ttypc_vtf_dch(vt);
				vt->escst = ESC_INIT;
				break;

			case 'S':   /* Scroll up */
				_ttypc_vtf_su(vt);
				vt->escst = ESC_INIT;
				break;

			case 'T':   /* Scroll down */
				_ttypc_vtf_sd(vt);
				vt->escst = ESC_INIT;
				break;

			case 'X':   /* Erase character */
				_ttypc_vtf_ech(vt);
				vt->escst = ESC_INIT;
				break;

			case 'c':   /* Device attributes */
				_ttypc_vtf_da(vt);
				vt->escst = ESC_INIT;
				break;

			case 'f':   /* Direct cursor addressing */
				_ttypc_vtf_curadr(vt);
				vt->escst = ESC_INIT;
				break;

			case 'g':   /* Clear tabs */
				_ttypc_vtf_clrtab(vt);
				vt->escst = ESC_INIT;
				break;

			case 'h':   /* Set ANSI modes */
				_ttypc_vtf_setansi(vt);
				vt->escst = ESC_INIT;
				break;

			case 'i':   /* Media copy */
				_ttypc_vtf_mc(vt);
				vt->escst = ESC_INIT;
				break;

			case 'l':   /* Reset ANSI modes */
				_ttypc_vtf_resetansi(vt);
				vt->escst = ESC_INIT;
				break;

			case 'm':   /* Select graphic rendition */
				_ttypc_vtf_sgr(vt);
				vt->escst = ESC_INIT;
				break;

			case 'n':   /* Reports */
				_ttypc_vtf_dsr(vt);
				vt->escst = ESC_INIT;
				break;

			case 'r':   /* Set scrolling region */
				_ttypc_vtf_stbm(vt);
				vt->escst = ESC_INIT;
				break;

			case 'x':   /* Request/report parameters */
				_ttypc_vtf_reqtparm(vt);
				vt->escst = ESC_INIT;
				break;

			case 'y':   /* Invoke selftest(s) */
				_ttypc_vtf_tst(vt);
				vt->escst = ESC_INIT;
				break;

			default:
				vt->escst = ESC_INIT;
				break;
			}
			break;

		case ESC_CSIQM: /* DEC private modes */
			switch (c) {
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':   /* Parameters */
				vt->parms[vt->parmi] *= 10;
				vt->parms[vt->parmi] += (c - '0');
				break;

			case ';':   /* Next parameter */
				if (vt->parmi + 1 < MAXPARMS)
					vt->parmi++;
				break;

			case 'h':   /* Set modes */
				_ttypc_vtf_setdecpriv(vt);
				vt->escst = ESC_INIT;
				break;

			case 'l':   /* Reset modes */
				_ttypc_vtf_resetdecpriv(vt);
				vt->escst = ESC_INIT;
				break;

			case 'n':   /* Reports */
				_ttypc_vtf_dsr(vt);
				vt->escst = ESC_INIT;
				break;

			case 'K':   /* Selective erase in line */
				vt->escst = ESC_INIT;
				break;

			case 'J':   /* Selective erase in display */
				vt->escst = ESC_INIT;
				break;

			default:
				vt->escst = ESC_INIT;
				break;
			}
			break;

		case ESC_DCS:
			/* TODO */
			vt->dcsst = DCS_INIT;
			vt->escst = ESC_INIT;
			break;

		case ESC_SCA:
			switch (c) {
			case 'q':
				vt->escst = ESC_INIT;
				break;

			default:
				vt->escst = ESC_INIT;
				break;
			}
			break;

		case ESC_STR:
			switch (c) {
			case 'p':   /* Soft terminal reset */
				_ttypc_vtf_str(vt);
				vt->escst = ESC_INIT;
				break;

			default:
				vt->escst = ESC_INIT;
				break;
			}
			break;

		default:
			vt->escst = ESC_INIT;
			break;
		}
	}

	/* Update screen and scrollback buffer */
	if (vt->cpos >= (vt->bottom + 1) * vt->cols) {
		_ttypc_vga_rollup(vt, 1);
		vt->cpos -= vt->cols;
	}
	vt->crow = vt->cpos / vt->cols;

	/* Update cursor */
	if (vt == vt->ttypc->vt)
		_ttypc_vga_setcursor(vt);
}


ssize_t ttypc_vt_read(ttypc_vt_t *vt, int mode, char *buff, size_t len)
{
	return libtty_read(&vt->tty, buff, len, mode);
}


ssize_t ttypc_vt_write(ttypc_vt_t *vt, int mode, const char *buff, size_t len)
{
	return libtty_write(&vt->tty, buff, len, mode);
}


int ttypc_vt_respond(ttypc_vt_t *vt, const char *buff)
{
	int err = 0;
	while ((*buff != '\0') && (err == 0)) {
		err = libtty_putchar(&vt->tty, *(buff++), NULL);
	}
	return err;
}


int ttypc_vt_pollstatus(ttypc_vt_t *vt)
{
	return libtty_poll_status(&vt->tty);
}


int ttypc_vt_ioctl(ttypc_vt_t *vt, pid_t pid, unsigned int cmd, const void *idata, const void **odata)
{
	return libtty_ioctl(&vt->tty, pid, cmd, idata, odata);
}


void ttypc_vt_destroy(ttypc_vt_t *vt)
{
	libtty_destroy(&vt->tty);
	if (SCRB_PAGES) {
		munmap(vt->scrb, SCRB_PAGES * _PAGE_SIZE);
		munmap(vt->scro, _PAGE_SIZE);
	}
	munmap(vt->mem, _PAGE_SIZE);
	resourceDestroy(vt->lock);
}


static void _ttypc_vt_setbaudrate(void *arg, speed_t baud)
{
	/* TODO */
}


static void _ttypc_vt_setcflag(void *arg, tcflag_t *cflag)
{
	/* TODO */
}


static void _ttypc_vt_signaltxready(void *arg)
{
	ttypc_vt_t *vt = (ttypc_vt_t *)arg;

	while (libtty_txready(&vt->tty))
		_ttypc_vt_sput(vt, (char)libtty_popchar(&vt->tty));

	libtty_wake_writer(&vt->tty);
}


int ttypc_vt_init(ttypc_t *ttypc, unsigned int ttybuffsz, ttypc_vt_t *vt)
{
	libtty_callbacks_t cb = {
		.arg            = vt,
		.set_baudrate   = _ttypc_vt_setbaudrate,
		.set_cflag      = _ttypc_vt_setcflag,
		.signal_txready = _ttypc_vt_signaltxready
	};
	int err;

	if ((err = mutexCreate(&vt->lock)) < 0)
		return err;

	vt->mem = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (vt->mem == MAP_FAILED) {
		resourceDestroy(vt->lock);
		return -ENOMEM;
	}
	vt->vram = vt->mem;

	if (SCRB_PAGES) {
		vt->scro = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (vt->scro == MAP_FAILED) {
			resourceDestroy(vt->lock);
			munmap(vt->mem, _PAGE_SIZE);
			return -ENOMEM;
		}
		vt->scrb = mmap(NULL, SCRB_PAGES * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (vt->scrb == MAP_FAILED) {
			resourceDestroy(vt->lock);
			munmap(vt->mem, _PAGE_SIZE);
			munmap(vt->scro, _PAGE_SIZE);
			return -ENOMEM;
		}
	}

	if ((err = libtty_init(&vt->tty, &cb, ttybuffsz, TTYDEF_SPEED)) < 0) {
		resourceDestroy(vt->lock);
		munmap(vt->mem, _PAGE_SIZE);
		if (SCRB_PAGES) {
			munmap(vt->scro, _PAGE_SIZE);
			munmap(vt->scrb, SCRB_PAGES * _PAGE_SIZE);
		}
		return err;
	}

	/* Disable default libtty tab expansion */
	vt->tty.term.c_oflag &= ~(XTABS);

	/* Init emulator */
	vt->ttypc = ttypc;
	vt->cols = 80;
	vt->rows = 25;
	_ttypc_vtf_str(vt);

	/* Clear screen */
	_ttypc_vga_set(vt->vram, vt->attr | ' ', vt->cols * vt->rows);

	return EOK;
}
