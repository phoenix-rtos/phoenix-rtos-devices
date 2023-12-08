/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MULTI_SPACEWIRE_H_
#define _MULTI_SPACEWIRE_H_


#include <sys/msg.h>


void spw_handleMsg(msg_t *msg, int dev);


int spw_createDevs(oid_t *oid);


int spw_init(void);


#endif
