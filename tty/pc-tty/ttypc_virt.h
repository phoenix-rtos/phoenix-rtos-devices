/* 
 * Phoenix-RTOS
 *
 * ttypc VT220 emulator (based on FreeBSD 4.4 pcvt)
 *
 * Virtual console implementation
 *
 * Copyright 2019 Phoenix Systems
 * Copyright 2006-2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_VIRT_H_
#define _TTYPC_VIRT_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <libtty.h>


/* escape detection state machine */
#define STATE_INIT   0  /* normal */
#define STATE_ESC    1  /* got ESC */
#define STATE_BLANK  2  /* got ESC space */
#define STATE_HASH   3  /* got ESC # */
#define STATE_BROPN  4  /* got ESC ( */
#define STATE_BRCLO  5  /* got ESC ) */
#define STATE_CSI    6  /* got ESC [ */
#define STATE_CSIQM  7  /* got ESC [ ? */
#define STATE_AMPSND 8  /* got ESC & */
#define STATE_STAR   9  /* got ESC * */
#define STATE_PLUS  10  /* got ESC + */
#define STATE_DCS   11  /* got ESC P */
#define STATE_SCA   12  /* got ESC <Ps> " */
#define STATE_STR   13  /* got ESC ! */
#define STATE_MINUS 14  /* got ESC - */
#define STATE_DOT   15  /* got ESC . */
#define STATE_SLASH 16  /* got ESC / */


#define MAXTAB   132      /* no of possible tab stops */
#define MAXPARMS 10       /* for storing escape sequence parameters */
#define CHR      2        /* bytes per word in screen mem */


/* charset tables */
#define CSL     0x0000    /* ega/vga charset, lower half of 512 */
#define CSH     0x0800    /* ega/vga charset, upper half of 512 */
#define CSSIZE  96        /* (physical) size of a character set */

typedef struct _ttypc_t ttypc_t;

typedef struct _ttypc_virt_t {
	char active;
	handle_t mutex;

	uint16_t *vram;                    /* video page start addr */
	uint16_t *mem;                     /* malloc'ed memory start address */
	unsigned int memsz;

	uint8_t m_awm;                     /* flag, vt100 mode, auto wrap */
	uint8_t m_ckm;                     /* true = cursor key normal mode */
	uint8_t m_irm;                     /* true = insert mode */

	uint8_t row;
	uint8_t col;                       /* current presentation component position */
	uint16_t attr;                     /* current character attributes */
	uint8_t state;                     /* escape sequence state machine */
	uint8_t vtsgr;

	uint8_t rows;
	uint8_t maxcol;
	char tab_stops[MAXTAB];            /* table of active tab stops */

	uint8_t scrr_beg;                  /* scrolling region, begin */
	uint8_t scrr_len;                  /* scrolling region, length */
	uint8_t scrr_end;                  /* scrolling region, end */

	uint8_t lastrow;                   /* save row, --------- " -----------  */
	uint8_t lastchar;                  /* flag, vt100 behaviour of last char */

	uint16_t cur_offset;               /* current cursor position offset */

	uint8_t cursor_start;              /* Start of cursor */
	uint8_t cursor_end;                /* End of cursor */
	uint8_t cursor_on;                 /* cursor switched on */

	uint8_t ss;                        /* flag, single shift G2 / G3 -> GL */
	uint16_t **Gs;                     /* ptr to cur. G2/G3 conversion table*/
	uint16_t **GL;                     /* ptr to current GL conversion table*/
	uint16_t **GR;                     /* ptr to current GR conversion table*/

	uint16_t *G0;                      /* ptr to current G0 conversion table*/
	uint16_t *G1;                      /* ptr to current G1 conversion table*/
	uint16_t *G2;                      /* ptr to current G2 conversion table*/
	uint16_t *G3;                      /* ptr to current G3 conversion table*/
	
	uint8_t parmi;                     /* parameter index */
	uint8_t parms[MAXPARMS];           /* parameter array */

	uint16_t scr_offset;               /* current scrollback offset (lines) */
//	uint16_t scrollback_pages;         /* size of scrollback buffer */
//	short scrolling;                   /* current scrollback page */
//	uint16_t max_off;                  /* maximum scrollback offset */

	uint8_t sc_flag;
	uint8_t sc_row;
	uint8_t sc_col;
	uint16_t sc_cur_offset;
	uint8_t sc_attr;
	uint8_t sc_awm;
	
	uint16_t *sc_G0;
	uint16_t *sc_G1;
	uint16_t *sc_G2;
	uint16_t *sc_G3;
	uint16_t **sc_GL;
	uint16_t **sc_GR;

	oid_t oid;
	libtty_common_t tty;
	ttypc_t *ttypc;
} ttypc_virt_t;


extern uint16_t csd_ascii[CSSIZE];


extern uint16_t csd_supplemental[CSSIZE];


/* Function emulates a character */
extern int ttypc_virt_sput(ttypc_virt_t *virt, char c);


/* Function emulates a buffer */
extern int ttypc_virt_swrite(ttypc_virt_t *virt, char *buff, size_t len);


/* Function adds characters to input buffer */
extern int ttypc_virt_sadd(ttypc_virt_t *virt, char *buff, size_t len, int mode);


/* Function gets characters from input buffer */
extern int ttypc_virt_sget(ttypc_virt_t *virt, char *buff, size_t len, int mode);


extern int ttypc_virt_poll_status(ttypc_virt_t *virt);
extern int ttypc_virt_ioctl(ttypc_virt_t *virt, pid_t sender_pid, unsigned int cmd, const void *in_arg, const void **out_arg);


/* Function initializes ttypc virtual terminal handler */
extern int _ttypc_virt_init(ttypc_virt_t *virt, unsigned int bufsize, ttypc_t *ttypc);


#endif
