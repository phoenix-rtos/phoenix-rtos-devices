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
#include <sys/msg.h>

#include "mbr.h"


mbr_t *alloc_mbr(ata_dev_t *dev)
{
	int ret = 0;
	mbr_t *mbr = malloc(sizeof(mbr_t));

	ret = ata_read(dev, 0, (char *)mbr, sizeof(mbr_t));
	if (ret != sizeof(mbr_t)) {
		free(mbr);
		return NULL;
	}

	if (mbr->boot_sign != MBR_SIGNATURE) {
		free(mbr);
		return NULL;
	}

	return mbr;
}
