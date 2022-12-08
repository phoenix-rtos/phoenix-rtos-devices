/*
 * Phoenix-RTOS
 *
 * iMX RT UART driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _UART_H_
#define _UART_H_


int uart_handleMsg(msg_t *msg, int dev);


void uart_klogCblk(const char *data, size_t size);


int uart_init(void);


#endif
