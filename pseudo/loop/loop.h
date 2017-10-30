/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Loop driver
 *
 * Copyright 2015 Phoenix Systems
 *
 * Author: Jakub Sejdak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LOOP_H_
#define _LOOP_H_

extern void _loop_controlInit(void);

extern int _loop_init(const char *underlying_name, unsigned long start, unsigned long length);

#endif
