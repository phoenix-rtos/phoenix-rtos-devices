/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Messaging Unit driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef MU_H_
#define MU_H_

#include <stdint.h>
#include <stddef.h>

#define CHANNEL_NO 4


int mu_read(int channel, void *buff, int len);


int mu_write(int channel, const void *buff, int len);


void mu_init(void);


#endif
