/*
 * Phoenix-RTOS
 *
 * PS/2 common utilities
 *
 * Copyright 2001, 2007-2008 Pawel Pisarczyk
 * Copyright 2012, 2017, 2019, 2020, 2024 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski, Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_PS2_H_
#define _TTYPC_PS2_H_

#include "ttypc_vt.h"

/* Waits for PS/2 controller status bit with small timeout */
int ttypc_ps2_waitstatus(ttypc_t *ttypc, unsigned char bit, unsigned char state);

/* Writes a byte to PS/2 control buffer */
int ttypc_ps2_write_ctrl(ttypc_t *ttypc, unsigned char byte);

/* Reads a byte from PS/2 I/O buffer */
int ttypc_ps2_read(ttypc_t *ttypc);

/* Writes a byte to PS/2 I/O buffer */
int ttypc_ps2_write(ttypc_t *ttypc, unsigned char byte);

/* Writes a byte to PS/2 control buffer */
int ttypc_ps2_write_ctrl(ttypc_t *ttypc, unsigned char byte);

/* Reads a byte from PS/2 control buffer */
unsigned char ttypc_ps2_read_ctrl(ttypc_t *ttypc);

#endif
