/*
 * Phoenix-RTOS
 *
 * STM32L4 TTY driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef UART_H_
#define UART_H_

#include <stddef.h>
#include <stdlib.h> /* FIXME - should be types.h */

ssize_t tty_log(const char *str, size_t len);


void tty_createDev(void);


int tty_init(void);


#endif
