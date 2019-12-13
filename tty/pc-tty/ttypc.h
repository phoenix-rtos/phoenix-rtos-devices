/*
 * Phoenix-RTOS
 *
 * ttypc VT220 emulator (based on FreeBSD 4.4 pcvt)
 *
 * Virtual console implementation
 *
 * Copyright 2019 Phoenix Systems
 * Copyright 2017 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_H_
#define _TTYPC_H_

#include <sys/types.h>

#include "ttypc_virt.h"


typedef struct _ttypc_t {
	handle_t mutex;

	char poolthr_stack[2][2 * 4096] __attribute__ ((aligned(8)));
	char kbdthr_stack[2 * 4096] __attribute__ ((aligned(8)));

	ttypc_virt_t virtuals[4];
	ttypc_virt_t *cv;

	int color;
	unsigned int irq;
	void *base;
	void *out_base;
	void *out_crtc;

	unsigned char extended;
	unsigned int lockst;
	unsigned int shiftst;

	handle_t rlock;
	handle_t rcond;
	handle_t inth;
} ttypc_t;


#endif
