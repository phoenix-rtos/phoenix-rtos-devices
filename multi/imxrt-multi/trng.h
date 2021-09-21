/*
 * Phoenix-RTOS
 *
 * i.MX RT TRNG driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _TRNG_H_
#define _TRNG_H_

#include <sys/msg.h>

int trng_handleMsg(msg_t *msg);


int trng_init(void);


#endif
