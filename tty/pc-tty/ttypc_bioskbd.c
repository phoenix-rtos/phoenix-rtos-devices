/*
 * Phoenix-RTOS
 *
 * 101-key US BIOS keyboard
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
#include <unistd.h>

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/reboot.h>

#include "ttypc_bioskbd.h"
#include "ttypc_vga.h"


static void ttypc_bioskbd_ctlthr(void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;
	ttypc_vt_t *cvt;
	uint16_t head;

	for (;;) {
		/* Wait for keystrokes to show up in keyboard buffer */
		while ((head = *(volatile uint16_t *)((uintptr_t)ttypc->kbd + 0x41a)) == *(volatile uint16_t *)((uintptr_t)ttypc->kbd + 0x41c))
			usleep(1000);

		mutexLock(ttypc->lock);
		mutexLock((cvt = ttypc->vt)->lock);

		/* Add the keystroke and update head position */
		libtty_putchar(&ttypc->vt->tty, *(volatile unsigned char *)((uintptr_t)ttypc->kbd + 0x400 + head), NULL);
		*(volatile uint16_t *)((uintptr_t)ttypc->kbd + 0x41a) = 0x1e + (head - 0x1e + 2) % 32;

		mutexUnlock(cvt->lock);
		mutexUnlock(ttypc->lock);
	}
}


int _ttypc_bioskbd_updateled(ttypc_t *ttypc)
{
	return EOK;
}


void ttypc_bioskbd_destroy(ttypc_t *ttypc)
{
	munmap((void *)ttypc->kbd, _PAGE_SIZE);
}


int ttypc_bioskbd_init(ttypc_t *ttypc)
{
	int err;

	if ((ttypc->kbd = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x0)) == MAP_FAILED)
		return -ENOMEM;

	/* Launch keyboard control thread */
	if ((err = beginthread(ttypc_bioskbd_ctlthr, 1, ttypc->kstack, sizeof(ttypc->kstack), ttypc)) < 0)
		return err;

	return EOK;
}
