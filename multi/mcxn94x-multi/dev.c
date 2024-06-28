/*
 * Phoenix-RTOS
 *
 * Multidriver device manager
 *
 * Copyright 2024 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stddef.h>
#include <errno.h>
#include <sys/msg.h>
#include <posix/utils.h>

#include "common.h"
#include "dev.h"


#define MAJOR_MAX 16


static struct {
	oid_t oid;
	devHandler *devices[MAJOR_MAX];
} dev_common = { .devices = { NULL } };


static void dev_oid2mm(oid_t *oid, unsigned int *major, unsigned int *minor)
{
	*major = (oid->id >> 16) & 0xffff;
	*minor = oid->id & 0xffff;
}


static void dev_mm2oid(oid_t *oid, unsigned int major, unsigned int minor)
{
	oid->id = (major << 16) | minor;
}


/* To be used only in constructors - no lock needed */
int dev_allocMajor(unsigned int *major)
{
	static unsigned int counter = 0;

	if (counter == NELEMS(dev_common.devices)) {
		return -1;
	}

	*major = counter++;

	return 0;
}


int dev_registerFile(const char *fname, unsigned int major, unsigned int minor)
{
	oid_t oid = { .port = dev_common.oid.port };

	dev_mm2oid(&oid, major, minor);

	return create_dev(&oid, fname);
}


int dev_msgReceive(msg_t *msg, msg_rid_t *rid)
{
	return msgRecv(dev_common.oid.port, msg, rid);
}


void dev_handle(msg_t *msg, msg_rid_t rid)
{
	unsigned int major, minor;

	dev_oid2mm(&msg->oid, &major, &minor);

	if ((major < NELEMS(dev_common.devices)) && (dev_common.devices[major] != NULL)) {
		dev_common.devices[major](msg, rid, major, minor);
	}
	else {
		msg->o.err = -ENOSYS;
		msgRespond(dev_common.oid.port, msg, rid);
	}
}


void dev_register(devHandler *handler, unsigned int major)
{
	dev_common.devices[major] = handler;
}


/* Has to be executed first! */
static void __attribute__((constructor(101))) dev_init(void)
{
	portCreate(&dev_common.oid.port);
}
