/*
 * Phoenix-RTOS
 *
 * USB Mass Storage class driver
 *
 * Common header
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _UMASS_H_
#define _UMASS_H_

#include <stdbool.h>
#include <unistd.h>

#include <usbdriver.h>


typedef struct {
	bool mount_root;
} umass_args_t;


#endif
