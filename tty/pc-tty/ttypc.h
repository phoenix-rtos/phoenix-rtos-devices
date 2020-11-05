/*
 * Phoenix-RTOS
 *
 * Terminal emulator using VGA display and 101-key US keyboard (based on FreeBSD 4.4 pcvt)
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


/* Keyboard types */
enum { KBD_BIOS, KBD_PS2 };


/* Keyboard key types */
enum {
	KB_NONE    = 0x0000,
	KB_CTL     = 0x0001,
	KB_SHIFT   = 0x0002,
	KB_ALT     = 0x0004,
	KB_ALTGR   = 0x0008,
	KB_SCROLL  = 0x0010,
	KB_NUM     = 0x0020,
	KB_CAPS    = 0x0040,
	KB_FUNC    = 0x0080,
	KB_ASCII   = 0x0100,
	KB_KP      = 0x0200,
	KB_EXT     = 0x0400
};


struct _ttypc_t {
	unsigned int port;     /* Driver port */

	/* VGA */
	volatile void *vga;    /* VGA screen memory */
	void *crtc;            /* Video Display Controller (CRTC) */
	unsigned color;        /* Color support */

	/* KBD */
	volatile void *kbd;    /* Keyboard controller */
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
