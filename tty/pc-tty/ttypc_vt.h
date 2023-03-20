/*
 * Phoenix-RTOS
 *
 * Virtual Terminal (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2006-2008 Pawel Pisarczyk
 * Copyright 2019, 2020, 2023 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_VT_H_
#define _TTYPC_VT_H_

#include <stdint.h>
#include <sys/types.h>

#include <libtty.h>


/* Escape sequence detection states */
enum {
	ESC_INIT,   /* normal */
	ESC_ESC,    /* ESC */
	ESC_BLANK,  /* ESC space */
	ESC_HASH,   /* ESC # */
	ESC_BROPN,  /* ESC ( */
	ESC_BRCLO,  /* ESC ) */
	ESC_CSI,    /* ESC [ */
	ESC_CSIQM,  /* ESC [ ? */
	ESC_AMPSND, /* ESC & */
	ESC_STAR,   /* ESC * */
	ESC_PLUS,   /* ESC + */
	ESC_DCS,    /* ESC P */
	ESC_SCA,    /* ESC <Ps> " */
	ESC_STR,    /* ESC ! */
	ESC_MINUS,  /* ESC - */
	ESC_DOT,    /* ESC . */
	ESC_SLASH   /* ESC */
};


/* Device Control String states */
enum {
	DCS_INIT    /* normal */
	/* TODO */
};


/* Misc definitions */
#define MAXTABS    132       /* Max number of possible tab stops */
#define MAXPARMS   10        /* Max number of escape sequence parameters */
#define SCRB_PAGES 32        /* Number of scrollback buffer pages */


/* Forward declare ttypc_t */
typedef struct _ttypc_t ttypc_t;


typedef struct {
	ttypc_t *ttypc;          /* Parent ttypc_t object */

	/* Screen */
	volatile uint16_t *vram; /* Screen buffer address (if VT is active points to ttypc->vga, mem otherwise) */
	uint16_t *mem;           /* Screen memory buffer */
	uint8_t rows;            /* Screen height - number of rows */
	uint8_t cols;            /* Screen width - number of columns */
	uint8_t top;             /* Screen top margin */
	uint8_t bottom;          /* Screen bottom margin */

	/* Cursor */
	uint8_t ctype;           /* Cursor type */
	uint8_t cst;             /* Cursor state */
	uint8_t ccol;            /* Cursor column */
	uint8_t crow;            /* Cursor row */
	uint16_t cpos;           /* Cursor position offset */

	/* Scrollback */
	uint16_t *scro;          /* Scroll origin buffer */
	uint16_t *scrb;          /* Scrollback buffer */
	uint16_t scrbsz;         /* Scrollback size (in lines) */
	uint16_t scrbpos;        /* Scrollback position offset */

	/* Character sets */
	const uint16_t *G0;      /* G0 conversion table */
	const uint16_t *G1;      /* G1 conversion table */
	const uint16_t *G2;      /* G2 conversion table */
	const uint16_t *G3;      /* G3 conversion table */
	const uint16_t **GL;     /* GL conversion table */
	const uint16_t **GR;     /* GR conversion table */
	const uint16_t **Gs;     /* G2/G3 conversion table */
	uint8_t ss;              /* Single shift G2 / G3 -> GL */

	/* VT100 modes */
	uint8_t awm;             /* Auto Wrap Mode */
	uint8_t om;              /* Origin Mode */
	uint8_t ckm;             /* Cursor Key Mode */
	uint8_t irm;             /* InseRt Mode */
	uint8_t arm;             /* AutoRepeat Mode */
	uint8_t lnm;             /* Line feed/New line Mode */
	uint8_t sc;              /* Saved Cursor & attributes */
	uint8_t lc;              /* Last Character in row flag */

	/* SCa saved state */
	uint8_t scawm;           /* Saved Auto Wrap Mode */
	uint8_t scom;            /* Saved Origin Mode */
	uint8_t scattr;          /* Saved character attributes */
	uint8_t scsgr;           /* Saved SGR configuration */
	uint8_t scccol;          /* Saved cursor column */
	uint8_t sccrow;          /* Saved cursor row */
	uint16_t sccpos;         /* Saved cursor position offset */
	const uint16_t *scG0;    /* Saved G0 conversion table */
	const uint16_t *scG1;    /* Saved G1 conversion table */
	const uint16_t *scG2;    /* Saved G2 conversion table */
	const uint16_t *scG3;    /* Saved G3 conversion table */
	const uint16_t **scGL;   /* Saved GL conversion table */
	const uint16_t **scGR;   /* Saved GR conversion table */

	/* Character processing */
	uint16_t attr;           /* Current character attributes */
	uint8_t sgr;             /* SGR configuration */
	uint8_t dcsst;           /* Device Control String state machine */
	uint8_t escst;           /* Escape sequence state machine */
	uint8_t parmi;           /* Escape sequence parameter index */
	uint16_t parms[MAXPARMS]; /* Escape sequence parameter array */
	char tabs[MAXTABS];      /* Table of active tab stops */
	libtty_common_t tty;     /* Terminal character processing (using libtty) */

	/* Synchronization */
	handle_t lock; /* Access lock */
} ttypc_vt_t;


/* Include ttypc.h after ttypc_vt_t definition (ttypc_t requires complete ttypc_vt_t definition) */
#include "ttypc.h"


/* Reads characters from virtual terminal */
extern ssize_t ttypc_vt_read(ttypc_vt_t *vt, int mode, char *buff, size_t len);


/* Write characters to virtual terminal */
extern ssize_t ttypc_vt_write(ttypc_vt_t *vt, int mode, const char *buff, size_t len);


/* Respond to requests for a terminal reports */
extern int ttypc_vt_respond(ttypc_vt_t *vt, const char *buff);


/* Polls virtual terminal status */
extern int ttypc_vt_pollstatus(ttypc_vt_t *vt);


/* Virtual terminal ioctl */
extern int ttypc_vt_ioctl(ttypc_vt_t *vt, pid_t pid, unsigned int cmd, const void *idata, const void **odata);


/* Destroys virtual terminal */
extern void ttypc_vt_destroy(ttypc_vt_t *vt);


/* Initializes virtual terminal */
extern int ttypc_vt_init(ttypc_t *ttypc, unsigned int ttybuffsz, ttypc_vt_t *vt);


#endif
