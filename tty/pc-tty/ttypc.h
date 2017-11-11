/* 
 * Phoenix-RTOS
 *
 * ttypc VT220 emulator (based on FreeBSD 4.4 pcvt)
 *
 * Virtual consoel implementation
 *
 * Copyright 217 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_H_
#define _TTYPC_H_

#include <stdio.h>

#include "ttypc_virt.h"


typedef struct _ttypc_t {
	handle_t mutex;
	ttypc_virt_t virtuals[4];
	ttypc_virt_t *cv;

	int color;
	unsigned int inp_irq;
	void *inp_base;
	void *out_base;
	void *out_crtc;
	
	unsigned char extended;
	unsigned int lockst;
	unsigned int shiftst;

	handle_t rlock;
	handle_t rcond;
	unsigned char **rbuff;
	unsigned int rbuffsz;
	unsigned int rb;
	unsigned int rp;
} ttypc_t;


#endif
