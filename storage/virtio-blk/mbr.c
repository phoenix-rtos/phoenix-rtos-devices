/*
 * Phoenix-RTOS
 *
 * Master Boot Record
 *
 * Copyright 2017, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>

#include "mbr.h"


int mbr_read(virtio_blk_t *bdev, mbr_t *mbr)
{
	ssize_t ret;

	if ((ret = virtio_blk_read(bdev, 0, (char *)mbr, sizeof(mbr_t))) != sizeof(mbr_t))
		return -EIO;

	if (mbr->magic != MBR_MAGIC)
		return -ENOENT;

	return EOK;
}
