/*
 * Phoenix-RTOS
 *
 * nRF91 TTY driver
 *
 * Copyright 2017, 2018, 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef UART_H_
#define UART_H_

#include <stddef.h>


ssize_t uart_log(const char *str, size_t len, unsigned int mode);


ssize_t uart_read(const char *str, size_t len, unsigned int mode);


void uart_createConsoleDev(void);


int uart_init(void);


#endif
