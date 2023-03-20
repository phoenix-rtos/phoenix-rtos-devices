/*
 * Phoenix-RTOS
 *
 * VGA display
 *
 * Copyright 2001, 2006 Pawel Pisarczyk
 * Copyright 2012, 2019, 2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_VGA_H_
#define _TTYPC_VGA_H_

#include <stdint.h>

#include "ttypc_vt.h"


/* clang-format off */


/* VGA screen memory address */
enum {
	VGA_GRAPHICS   = 0xa0000, /* Graphics mode */
	VGA_MONO       = 0xb0000, /* Text mode, monochrome */
	VGA_COLOR      = 0xb8000  /* Text/CGA-compatible mode, color */
};


/* VGA CRTC Registers */
enum {
	CRTC_MONO      = 0x3b4,   /* CRTC monochrome index register */
	CRTC_COLOR     = 0x3d4    /* CRTC color index register */
};


/* VGA CRTC register layout */
enum {
	CRTC_ADDR      = 0x00, /* Index register */
	CRTC_HTOTAL    = 0x00, /* Horizontal total */
	CRTC_HDISPLE   = 0x01, /* Horizontal display end */
	CRTC_HBLANKS   = 0x02, /* Horizontal blank start */
	CRTC_HBLANKE   = 0x03, /* Horizontal blank end */
	CRTC_HSYNCS    = 0x04, /* Horizontal sync start */
	CRTC_HSYNCE    = 0x05, /* Horizontal sync end */
	CRTC_VTOTAL    = 0x06, /* Vertical total */
	CRTC_OVERFLL   = 0x07, /* Overflow low */
	CRTC_IROWADDR  = 0x08, /* Inital row address */
	CRTC_MAXROW    = 0x09, /* Maximum row address */
	CRTC_CURSTART  = 0x0a, /* Cursor start row address */
	CRTC_CUREND    = 0x0b, /* Cursor end row address */
	CRTC_STARTADRH = 0x0c, /* Linear start address mid */
	CRTC_STARTADRL = 0x0d, /* Linear start address low */
	CRTC_CURSORH   = 0x0e, /* Cursor address high */
	CRTC_CURSORL   = 0x0f, /* Cursor address low */
	CRTC_VSYNCS    = 0x10, /* Vertical sync start */
	CRTC_VSYNCE    = 0x11, /* Vertical sync end */
	CRTC_VDE       = 0x12, /* Vertical display end */
	CRTC_OFFSET    = 0x13, /* Row offset */
	CRTC_ULOC      = 0x14, /* Underline row address */
	CRTC_VBSTART   = 0x15, /* Vertical blank start */
	CRTC_VBEND     = 0x16, /* Vertical blank end */
	CRTC_MODE      = 0x17, /* CRTC mode register */
	CRTC_SPLITL    = 0x18, /* Split screen start low */
	CRTC_CUROFF    = 0x20  /* Cursor off */
};


/* VGA GENERAL/EXTERNAL Registers */
enum {
	GN_MISCOUTR    = 0x3cc, /* Misc output register read */
	GN_MISCOUTW    = 0x3c2, /* Misc output register write */
	GN_INPSTAT0    = 0x3c2, /* Input status 0, r/o */
	GN_INPSTAT1M   = 0x3ba, /* Input status 1, r/o, monochrome */
	GN_INPSTAT1C   = 0x3da, /* Input status 1, r/o, color */
	GN_FEATR       = 0x3ca, /* Feature control, read */
	GN_FEATWM      = 0x3ba, /* Feature control, write, monochrome */
	GN_FEATWC      = 0x3da, /* Feature control, write, color */
	GN_VSUBSYS     = 0x3c3, /* Video subsystem register r/w */
	GN_DMCNTLM     = 0x3b8, /* Display mode control, r/w, monochrome */
	GN_DMCNTLC     = 0x3d8, /* Display mode control, r/w, color */
	GN_COLORSEL    = 0x3d9  /* Color select register, w/o */
};


/* Foreground color attributes */
enum {
	FG_BLACK        = 0x00,
	FG_BLUE         = 0x01,
	FG_GREEN        = 0x02,
	FG_CYAN         = 0x03,
	FG_RED          = 0x04,
	FG_MAGENTA      = 0x05,
	FG_BROWN        = 0x06,
	FG_LIGHTGREY    = 0x07,
	FG_DARKGREY     = 0x08,
	FG_LIGHTBLUE    = 0x09,
	FG_LIGHTGREEN   = 0x0a,
	FG_LIGHTCYAN    = 0x0b,
	FG_LIGHTRED     = 0x0c,
	FG_LIGHTMAGENTA = 0x0d,
	FG_YELLOW       = 0x0e,
	FG_WHITE        = 0x0f,
	FG_BLINK        = 0x80,
	FG_BRIGHT       = 0x800
};


/* Background color attributes */
enum {
	BG_BLACK        = 0x00,
	BG_BLUE         = 0x10,
	BG_GREEN        = 0x20,
	BG_CYAN         = 0x30,
	BG_RED          = 0x40,
	BG_MAGENTA      = 0x50,
	BG_BROWN        = 0x60,
	BG_LIGHTGREY    = 0x70,
	BG_BRIGHT       = 0x8000
};


/* Foreground monochrome attributes */
enum {
	FG_UNDERLINE    = 0x01,
	FG_INTENSE      = 0x08
};


/* Background monochrome attributes */
enum {
	BG_INTENSE      = 0x10
};


/* Misc definitions */
#define CHR_VGA     2               /* Bytes per word in VGA screen memory */
#define FONT_DEFH   16              /* Default font height in dots */
#define CUR_DEFH    (FONT_DEFH - 2) /* Default cursor height in dots */


/* Cursor types */
enum {
	CUR_BLOCK       = 0,
	CUR_TWO_THIRDS  = (CUR_DEFH / 3),
	CUR_LOWER_HALF  = (CUR_DEFH / 2),
	CUR_LOWER_THIRD = (2 * CUR_DEFH / 3),
	CUR_UNDERLINE   = (CUR_DEFH - 1),
	CUR_NONE        = (CUR_DEFH + 1),
	CUR_DEF         = CUR_UNDERLINE
};



/* clang-format off */

/* Copies VGA screen buffer to buff */
extern ssize_t _ttypc_vga_read(volatile uint16_t *vga, uint16_t *buff, size_t n);


/* Copies buff to VGA screen buffer */
extern ssize_t _ttypc_vga_write(volatile uint16_t *vga, uint16_t *buff, size_t n);


/* Sets VGA screen buffer characters to val */
extern volatile uint16_t *_ttypc_vga_set(volatile uint16_t *vga, uint16_t val, size_t n);


/* Moves VGA screen memory */
extern volatile uint16_t *_ttypc_vga_move(volatile uint16_t *dvga, volatile uint16_t *svga, size_t n);


/* Switches to another VT */
extern void _ttypc_vga_switch(ttypc_vt_t *vt);


/* Rolls VT screen n lines up (updates scrollback buffer) */
extern void _ttypc_vga_rollup(ttypc_vt_t *vt, unsigned int n);


/* Rolls VT screen n lines down (updates scrollback buffer) */
extern void _ttypc_vga_rolldown(ttypc_vt_t *vt, unsigned int n);


/* Scrolls VT screen by n lines */
extern void _ttypc_vga_scroll(ttypc_vt_t *vt, int n);


/* Cancels VT screen scrolling */
extern void _ttypc_vga_scrollcancel(ttypc_vt_t *vt);


/* Retrieves cursor position */
extern void _ttypc_vga_getcursor(ttypc_vt_t *vt);


/* Sets currsor position */
extern void _ttypc_vga_setcursor(ttypc_vt_t *vt);


/* Toggles cursor visibility */
extern void _ttypc_vga_togglecursor(ttypc_vt_t *vt, uint8_t state);


/* Destroys VGA display */
extern void ttypc_vga_destroy(ttypc_t *ttypc);


/* Initializes VGA display */
extern int ttypc_vga_init(ttypc_t *ttypc);


#endif
