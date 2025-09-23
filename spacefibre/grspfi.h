/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Andrzej Tlomak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef GRSPFI_H
#define GRSPFI_H

#include <sys/msg.h>


enum {
	id_spfi0 = 0u,
	id_spfi1,
	id_spfi2,
	id_spfi3,
	id_spfi4,
	id_spfi5,
};


typedef struct {
	/* clang-format off */
	enum { spfi_vc_set = 0 } type;
	/* clang-format on */
	union {
		struct {
			uint8_t dma;
			uint32_t vc; /* virtual channels enabled for the dma channel, set bits on vc number */
		} dma_mapping;
	} task;
} spfi_t;


_Static_assert(sizeof(spfi_t) <= sizeof(((msg_t *)0)->i.raw), "spfi_t exceeds size of msg.i.raw");


typedef struct {
	unsigned int val;
} spfi_o_t;


_Static_assert(sizeof(spfi_o_t) <= sizeof(((msg_t *)0)->i.raw), "spfi_o_t exceeds size of msg.i.raw");

#endif
