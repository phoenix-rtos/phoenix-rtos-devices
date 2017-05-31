/* 
 * Phoenix-RTOS
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





/* Function initializes PC tty */
extern void _ttypc_init(void);


#endif
