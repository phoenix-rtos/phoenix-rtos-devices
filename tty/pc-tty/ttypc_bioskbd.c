/*
 * Phoenix-RTOS
 *
 * PC 101-key BIOS keyboard
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/reboot.h>

#include "ttypc_bioskbd.h"
#include "ttypc_vga.h"


static void ttypc_bioskbd_ctlthr(void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;
	uint16_t head;

	for (;;) {
		/* Wait for keystrokes to show up in keyboard buffer */
		while ((head = *(uint16_t *)((uintptr_t)ttypc->kbd + 0x41a)) == *(uint16_t *)((uintptr_t)ttypc->kbd + 0x41c))
			usleep(1000);

		/* Add the keystroke and update head position */
		libtty_putchar(&ttypc->vt->tty, *(unsigned char *)((uintptr_t)ttypc->kbd + 0x400 + head), NULL);
		*(uint16_t *)((uintptr_t)ttypc->kbd + 0x41a) = 0x1e + (head - 0x1e + 2) % 32;
	}
}


int _ttypc_bioskbd_updateled(ttypc_t *ttypc)
{
	return EOK;
}


void ttypc_bioskbd_destroy(ttypc_t *ttypc)
{
	munmap(ttypc->kbd, _PAGE_SIZE);
}


int ttypc_bioskbd_init(ttypc_t *ttypc)
{
	int err;

	if ((ttypc->kbd = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, 0, OID_PHYSMEM, 0x0)) == MAP_FAILED)
		return -ENOMEM;

	/* Launch keyboard control thread */
	if ((err = beginthread(ttypc_bioskbd_ctlthr, 1, &ttypc->kstack, sizeof(ttypc->kstack), (void *)ttypc)) < 0)
		return err;

	return EOK;
}
