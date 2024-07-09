/*
 * Phoenix-RTOS
 *
 * i.MX RT integrated PCT2075 driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PCT2075_H_
#define _PCT2075_H_

#include <stdint.h>
#include <stddef.h>
#include <sys/msg.h>


void pct2075_handleMsg(msg_t *msg);


#endif /* _PCT2075_H_ */
