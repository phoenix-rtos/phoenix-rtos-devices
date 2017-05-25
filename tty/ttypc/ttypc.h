/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * PC 101-key keyboard handler derived from the BSD 4.4 Lite kernel.
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2001, 2006-2008 Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTY_TTYPC_H_
#define _TTY_TTYPC_H_

#include <hal/if.h>
#include <proc/if.h>

#include <dev/ttypc/ttypc_virt.h>
#include <dev/ttypc/ttypc_vtf.h>
#include <dev/ttypc/ttypc_kbd.h>
#include <dev/ttypc/ttypc_vga.h>


#define SIZE_TTYPC_RBUFF  128
#define SIZE_VIRTUALS     4


#define VRAM_MONO     (void *)0xb0000    /* VRAM address of mono 80x25 mode */
#define VRAM_COLOR    (void *)0xb8000    /* VRAM address of color 80x25 mode */

#define CRTR_MONO     0x3b4              /* crtc index register address mono */
#define CRTR_COLOR	  0x3d4              /* crtc index register address color */

#define CRTC_CURSORH  0x0e               /* cursor address mid */
#define CRTC_CURSORL  0x0f               /* cursor address low */

#define MAIN_MISCIN   0x3cc              /* miscellaneous input register */


typedef struct _ttypc_t {
	spinlock_t intrspinlock;
	thq_t waitq;

	semaphore_t mutex;	
	int do_initialization;

	ttypc_virt_t virtuals[SIZE_VIRTUALS];
	ttypc_virt_t *cv;

	int color;
	unsigned int inp_irq;
	void *inp_base;
	void *out_base;
	void *out_crtc;
	
	u8 extended;
	unsigned int lockst;
	unsigned int shiftst;
	
	u8 **rbuff;
	unsigned int rbuffsz;
	unsigned int rb;
	unsigned int rp;
} ttypc_t;


/* Function initializes PC tty */
extern void _ttypc_init(void);


#endif
