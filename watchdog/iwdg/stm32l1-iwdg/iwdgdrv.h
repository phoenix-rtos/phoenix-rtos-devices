/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 Independent Watchdog driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Adrian Kepka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IWDGDRV_H
#define _IWDGDRV_H


extern int iwdg_ping(void);


extern void iwdg_init(void);


#endif
