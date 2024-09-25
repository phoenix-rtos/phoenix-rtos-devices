/*
 * Phoenix-RTOS
 *
 * VGA framebuffer console
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_FBCON_H_
#define _TTYPC_FBCON_H_

#include <stdint.h>

#include "ttypc.h"


extern void _ttypc_fbcon_drawchar(ttypc_vt_t *vt, int col, int row, uint16_t val);


extern int ttypc_fbcon_init(ttypc_t *ttypc);


extern int ttypc_fbcon_destroy(ttypc_t *ttypc);


#endif
