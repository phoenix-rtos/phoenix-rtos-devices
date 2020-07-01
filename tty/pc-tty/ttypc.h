/* 
 * Phoenix-RTOS
 *
 * Terminal emulator using VGA and 101-key US keyboard (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2017 Pawel Pisarczyk
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_H_
#define _TTYPC_H_

#include <sys/types.h>

#include "ttypc_vt.h"


/* Number of virtual terminals */
#define NVTS 4


/* Keyboard type */
enum {
	KBD_BIOS,
	KBD_PS2
};


/* Keyboard key type */
enum {
	KB_NONE    = 0,
	KB_CTL     = 1,
	KB_SHIFT   = 2,
	KB_ALT     = 4,
	KB_ALTGR   = 8,
	KB_SCROLL  = 16,
	KB_NUM     = 32,
	KB_CAPS    = 64,
	KB_FUNC    = 128,
	KB_ASCII   = 256,
	KB_KP      = 512,
	KB_EXT     = 1024
};


struct _ttypc_t {
	unsigned int port;     /* Driver port */

	/* VGA */
	unsigned color;        /* Color support */
	void *vga;             /* VGA screen memory */
	void *crtc;            /* Video Display Controller (CRTC) */

	/* KBD */
	void *kbd;             /* Keyboard controller */
	unsigned char ktype;   /* Keyboard type */
	unsigned char lockst;  /* Lock keys state */
	unsigned char shiftst; /* Shift keys state */
	unsigned int kirq;     /* Interrupt number */
	handle_t klock;        /* Interrupt mutex */
	handle_t kcond;        /* Interrupt condition variable */
	handle_t kinth;        /* Interrupt handle */

	/* Virtual terminals */
	ttypc_vt_t *vt;        /* Active virtual terminal */
	ttypc_vt_t vts[NVTS];  /* Virtual Terminals */

	/* Synchronization */
	handle_t lock;         /* Access mutex */

	/* Thread stacks */
	char kstack[2048] __attribute__ ((aligned(8)));
	char pstack[2048] __attribute__ ((aligned(8)));
};


#endif
