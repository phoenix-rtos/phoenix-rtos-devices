/* 
 * Phoenix-RTOS
 *
 * ttypc VT220 emulator (based on FreeBSD 4.4 pcvt)
 *
 * Virtual consoel implementation
 *
 * Copyright 2006-2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_VIRT_H_
#define _TTYPC_VIRT_H_

#include <stdio.h>

#include "ttypc.h"


/* escape detection state machine */
#define STATE_INIT   0  /* normal	*/
#define	STATE_ESC    1  /* got ESC */
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
#define MAXPARMS  10      /* for storing escape sequence parameters */

#define CHR      2        /* bytes per word in screen mem */


/* charset tables */
#define CSL     0x0000    /* ega/vga charset, lower half of 512 */
#define CSH     0x0800    /* ega/vga charset, upper half of 512 */
#define CSSIZE  96        /* (physical) size of a character set */


typedef struct _ttypc_virt_t {
	char active;

	handle_t mutex;
	handle_t cond;

	u16 *vram;                    /* video page start addr */
	u16 *mem;                     /* malloc'ed memory start address */
	unsigned int memsz;

	u8 m_awm;                     /* flag, vt100 mode, auto wrap */
	u8 m_ckm;                     /* true = cursor key normal mode */
	u8 m_irm;                     /* true = insert mode */
	u8 m_lnm;                     /* Line Feed/New Line Mode */
	u8 m_echo;					  /* true = echo mode */

	u8 row;
	u8 col;                       /* current presentation component position */
	u16 attr;                     /* current character attributes */
	u8 state;                     /* escape sequence state machine */
	u8 vtsgr;

	u8 rows;
	u8 maxcol;
	char tab_stops[MAXTAB];       /* table of active tab stops */

	u8 scrr_beg;                  /* scrolling region, begin */
	u8 scrr_len;                  /* scrolling region, length */
	u8 scrr_end;                  /* scrolling region, end */

	u8 lastrow;                   /* save row, --------- " -----------  */
	u8 lastchar;                  /* flag, vt100 behaviour of last char */

	u16 cur_offset;               /* current cursor position offset */

	u8 cursor_start;              /* Start of cursor */
	u8 cursor_end;                /* End of cursor */
	u8 cursor_on;                 /* cursor switched on */

	u8 ss;                        /* flag, single shift G2 / G3 -> GL */
	u16 **Gs;                     /* ptr to cur. G2/G3 conversion table*/
	u16 **GL;                     /* ptr to current GL conversion table*/
	u16 **GR;                     /* ptr to current GR conversion table*/

	u16 *G0;                      /* ptr to current G0 conversion table*/
	u16 *G1;                      /* ptr to current G1 conversion table*/
	u16 *G2;                      /* ptr to current G2 conversion table*/
	u16 *G3;                      /* ptr to current G3 conversion table*/
	
	u8 parmi;                     /* parameter index */
	u8 parms[MAXPARMS];           /* parameter array */

	u16 scr_offset;               /* current scrollback offset (lines) */
//	u16 scrollback_pages;         /* size of scrollback buffer */
//	short scrolling;              /* current scrollback page */
//	u16 max_off;                  /* maximum scrollback offset */

	u8 sc_flag;
	u8 sc_row;
	u8 sc_col;
	u16 sc_cur_offset;
	u8 sc_attr;
	u8 sc_awm;
	
	u16 *sc_G0;
	u16 *sc_G1;
	u16 *sc_G2;
	u16 *sc_G3;
	u16 **sc_GL;
	u16 **sc_GR;
  
	u8 *rbuff;
	unsigned int rbuffsz;
	unsigned int rb;
	unsigned int rp;
	unsigned int ready;

	struct _ttypc_t *ttypc;
} ttypc_virt_t;


extern u16 csd_ascii[CSSIZE];


extern u16 csd_supplemental[CSSIZE];


/* Emulator main entry */
extern int ttypc_virt_sput(ttypc_virt_t *virt, u8 *s, int len);


/* Function adds characters to input buffer */
extern int ttypc_virt_sadd(ttypc_virt_t *virt, u8 *s, unsigned int len);


extern int ttypc_virt_sget(ttypc_virt_t *virt, char *buff, unsigned int len);


/* Function initializes ttypc virtual terminal handler */
extern int _ttypc_virt_init(ttypc_virt_t *virt, size_t rbuffsz, struct _ttypc_t *ttypc);


#endif
