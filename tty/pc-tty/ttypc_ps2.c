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

#include <errno.h>
#include <unistd.h>

#include <sys/io.h>
#include <sys/minmax.h>

#include "ttypc_vt.h"


#define SLEEP_MS               10u
#define MAX_TIMEOUT_DEFAULT_MS 10000u


/* Waits for PS/2 controller status bit with small timeout */
static int _ttypc_ps2_waitstatus(ttypc_t *ttypc, unsigned int bit, unsigned int state, unsigned int max_timeout)
{
	unsigned int i, b, rounds;

	if (max_timeout == 0u) {
		max_timeout = MAX_TIMEOUT_DEFAULT_MS;
	}

	rounds = max(1, max_timeout / SLEEP_MS);

	for (i = 0; i < rounds; i++) {
		b = inb((void *)((uintptr_t)ttypc->kbd + 4u));

		b &= (1u << bit) ^ (state << bit);
		if (b == 0u) {
			return EOK;
		}

		if (i < rounds - 1u) {
			usleep(SLEEP_MS * 1000);
		}
	}

	return -ETIMEDOUT;
}


static int _ttypc_ps2_write_to_port(ttypc_t *ttypc, void *port, unsigned char byte)
{
	int err;

	/* Wait for input buffer to be empty */
	err = _ttypc_ps2_waitstatus(ttypc, 1, 0, 0);
	if (err < 0) {
		return err;
	}

	outb(port, byte);

	return EOK;
}


int ttypc_ps2_write(ttypc_t *ttypc, unsigned char byte)
{
	return _ttypc_ps2_write_to_port(ttypc, (void *)ttypc->kbd, byte);
}


int ttypc_ps2_read(ttypc_t *ttypc)
{
	int err;

	/* Wait for output buffer not to be empty */
	err = _ttypc_ps2_waitstatus(ttypc, 0, 1, 0);
	if (err < 0) {
		return err;
	}

	return inb((void *)ttypc->kbd);
}


int ttypc_ps2_write_ctrl(ttypc_t *ttypc, unsigned char byte)
{
	return _ttypc_ps2_write_to_port(ttypc, (void *)((uintptr_t)ttypc->kbd + 4u), byte);
}


unsigned char ttypc_ps2_read_ctrl(ttypc_t *ttypc)
{
	return inb((void *)((uintptr_t)ttypc->kbd + 4u));
}
