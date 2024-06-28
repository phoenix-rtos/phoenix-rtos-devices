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

#ifndef MULTI_DEV_H_
#define MULTI_DEV_H_

#include <sys/msg.h>

typedef void devHandler(msg_t *msg, msg_rid_t rid, unsigned int major, unsigned int minor);


int dev_allocMajor(unsigned int *major);


int dev_registerFile(const char *fname, unsigned int major, unsigned int minor);


int dev_msgReceive(msg_t *msg, msg_rid_t *rid);


void dev_handle(msg_t *msg, msg_rid_t rid);


void dev_register(devHandler *handler, unsigned int major);

#endif /* MULTI_DEV_H_ */
