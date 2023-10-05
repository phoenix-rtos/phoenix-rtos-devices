/*
 * Phoenix-RTOS
 *
 * pseudo devices
 *
 * Copyright 2023 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/time.h>
#include "pseudodev.h"


static struct {
	unsigned int randSeed;
} pseudo_common;


static int pseudo_open(int id, int flags)
{
	(void)flags;

	switch (id) {
		case pseudo_idNull:
		case pseudo_idZero:
		case pseudo_idFull:
		case pseudo_idRandom:
			return EOK;

		default:
			break;
	}

	return -ENODEV;
}


static int pseudo_close(int id)
{
	switch (id) {
		case pseudo_idNull:
		case pseudo_idZero:
		case pseudo_idFull:
		case pseudo_idRandom:
			return EOK;

		default:
			break;
	}

	return -ENODEV;
}


static ssize_t pseudo_read(int id, void *buf, size_t count)
{
	switch (id) {
		case pseudo_idNull:
			return (ssize_t)0;

		case pseudo_idZero:
		case pseudo_idFull:
			memset(buf, 0, count);
			return (ssize_t)count;

		case pseudo_idRandom:
			for (size_t ofs = 0; ofs < count; ofs += sizeof(pseudo_common.randSeed)) {
				int res = rand_r(&pseudo_common.randSeed);
				memcpy((unsigned char *)buf + ofs, &res,
					((count - ofs) > sizeof(res)) ? sizeof(res) : count - ofs);
			}
			return (ssize_t)count;

		default:
			break;
	}

	return -ENODEV;
}


static ssize_t pseudo_write(int id, const void *buf, size_t count)
{
	(void)buf;

	switch (id) {
		case pseudo_idNull:
		case pseudo_idZero:
		case pseudo_idRandom:
			return (ssize_t)count;

		case pseudo_idFull:
			return -ENOSPC;

		default:
			break;
	}

	return -ENODEV;
}


int pseudo_handleMsg(msg_t *msg, int id)
{
	switch (msg->type) {
		case mtOpen:
			msg->o.io.err = pseudo_open(id, msg->i.openclose.flags);
			return 0;

		case mtClose:
			msg->o.io.err = pseudo_close(id);
			return 0;

		case mtRead:
			msg->o.io.err = pseudo_read(id, msg->o.data, msg->o.size);
			return 0;

		case mtWrite:
			msg->o.io.err = pseudo_write(id, msg->i.data, msg->i.size);
			return 0;

		default:
			break;
	}

	return -ENOSYS;
}


void pseudo_init(void)
{
	time_t now;
	gettime(&now, NULL);

	pseudo_common.randSeed = (unsigned int)now;
}
