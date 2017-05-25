/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * PC 101-key keyboard handler derived from the BSD 4.4 Lite kernel.
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2001, 2006 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_TTYPC_KBD_H_
#define _DEV_TTYPC_KBD_H_

#include <hal/if.h>

#include <dev/ttypc/ttypc.h>


#define KB_NONE    1
#define KB_ASCII   2
#define KB_CTL     4
#define KB_SHIFT   8
#define KB_KP      16
#define KB_ALT     32
#define KB_CAPS    64
#define KB_NUM     128
#define KB_SCROLL  256
#define KB_ALTGR   512
#define KB_FUNC    1024


typedef struct _keymap_t {
	unsigned int type;
	char *unshift;
	char *shift;
	char *ctl;
	char *altgr;
	char *shift_altgr;
} keymap_t;


/* Function initializes ttypc keyboard handler */
extern int _ttypc_kbd_init(struct _ttypc_t *ttypc);


#endif
