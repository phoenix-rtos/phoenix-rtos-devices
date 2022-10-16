/*
 * Phoenix-RTOS
 *
 * nRF91 TTY driver
 *
 * Copyright 2017, 2018, 2022 Phoenix Systems
 * Author: Aleksander Kaminski, Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef UART_H_
#define UART_H_


void tty_log(const char *str);


int tty_init(unsigned int *port);


#endif
