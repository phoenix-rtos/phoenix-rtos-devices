/*
 * Phoenix-RTOS
 *
 * PS/2 101-key US keyboard (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2001, 2006 Pawel Pisarczyk
 * Copyright 2012, 2019, 2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_KBD_H_
#define _TTYPC_KBD_H_

#include "ttypc.h"


/* Updates keyboard LEDs */
extern int _ttypc_kbd_updateled(ttypc_t *ttypc);


/* Destroys keyboard */
extern void ttypc_kbd_destroy(ttypc_t *ttypc);


/* Initializes PS/2 keyboard */
extern int ttypc_kbd_init(ttypc_t *ttypc);


#endif
