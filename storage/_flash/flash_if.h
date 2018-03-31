/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * FLASH memory DRIVE driver
 *
 * Copyright 2012 Phoenix Systems
 *
 * Author: Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASH_IF_H_
#define _FLASH_IF_H_

/* Function brings up FLASH memory DRIVE driver */
extern int _flash_init(void);
extern int flash_attach(dev_t flash, dev_t mtd);

#endif
