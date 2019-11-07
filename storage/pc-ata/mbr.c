/*
 * Phoenix-RTOS
 *
 * ext2
 *
 * mbr.c
 *
 * Copyright 2017 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/msg.h>

#include "mbr.h"


int read_mbr(ata_dev_t *dev, mbr_t *mbr)
{
	int ret = 0;

	if (!mbr || !dev)
		return -EINVAL;

	ret = atadrv_read(dev, 0, (char *)mbr, sizeof(mbr_t));
	if (ret != sizeof(mbr_t)) {
		free(mbr);
		return -EIO;
	}

	if (mbr->boot_sign != MBR_SIGNATURE) {
		free(mbr);
		return -ENOENT;
	}

	return EOK;
}
