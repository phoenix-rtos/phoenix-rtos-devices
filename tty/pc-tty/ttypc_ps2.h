/*
 * Phoenix-RTOS
 *
 * PS/2 common utilities
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_PS2_H_
#define _TTYPC_PS2_H_

#include "ttypc_vt.h"


/* Writes a byte to PS/2 control buffer */
extern int ttypc_ps2_write_ctrl(ttypc_t *ttypc, unsigned char byte);


/* Reads a byte from PS/2 I/O buffer */
extern int ttypc_ps2_read(ttypc_t *ttypc);


/* Writes a byte to PS/2 I/O buffer */
extern int ttypc_ps2_write(ttypc_t *ttypc, unsigned char byte);


/* Reads a byte from PS/2 control buffer */
extern unsigned char ttypc_ps2_read_ctrl(ttypc_t *ttypc);


#endif
