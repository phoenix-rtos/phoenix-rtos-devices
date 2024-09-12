/*
 * Phoenix-RTOS
 *
 * iMX RT RTT communication driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RTT_H_
#define _RTT_H_

#include <stddef.h>
#include <sys/msg.h>

#include <libtty.h>


int rtt_init(void);


int rtt_handleMsg(msg_t *msg, int dev);


void rtt_klogCblk(const char *data, size_t size);


#endif /* _RTT_H_ */
