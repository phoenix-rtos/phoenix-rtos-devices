/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ttypc vga support
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2001, 2006 Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_TTYPC_VGA_H_
#define _DEV_TTYPC_VGA_H_


#include <libphoenix.h>

#include "ttypc_virt.h"


extern void _ttypc_vga_cursor(ttypc_virt_t *virt);


extern void ttypc_vga_switch(ttypc_virt_t *virt);


extern void _ttypc_vga_rollup(ttypc_virt_t *virt, unsigned int n);


extern void _ttypc_vga_rolldown(ttypc_virt_t *virt, unsigned int n);


#endif
