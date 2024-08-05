/*
 * Phoenix-RTOS
 *
 * PS/2 3-button mouse
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_MOUSE_H_
#define _TTYPC_MOUSE_H_

#include "ttypc_vt.h"


/* Initializes PS/2 mouse */
extern int ttypc_mouse_init(ttypc_t *ttypc);


/* Reads a mouse event from I/O buffer and handles it */
extern int ttypc_mouse_handle_event(ttypc_t *ttypc, unsigned char b);


/* Destroys PS/2 mouse */
extern void ttypc_mouse_destroy(ttypc_t *ttypc);


#endif
