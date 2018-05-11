/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 UART driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include ARCH
#include <errno.h>
#include <sys/threads.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>

#include "common.h"
#include "gpio.h"
