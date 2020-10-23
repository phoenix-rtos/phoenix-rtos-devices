/*
 * Phoenix-RTOS
 *
 * VirtIO block device server
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "virtio-blk.h"


int main(int argc, char **argv)
{
	int err;

	/* Wait for console */
	while (write(STDOUT_FILENO, "", 0) < 0)
		usleep(10000);

	if ((err = virtio_blk_init()) < 0) {
		fprintf(stderr, "virtio-blk: failed to detect VirtIO block devices\n");
		return err;
	}

	return EOK;
}
