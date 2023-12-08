/*
 * Phoenix-RTOS
 *
 * GRLIB UART driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MULTI_UART_H_
#define _MULTI_UART_H_


#include <sys/msg.h>


void uart_handleMsg(msg_t *msg, int dev);


void uart_klogClbk(const char *data, size_t size);


int uart_createDevs(oid_t *oid);


int uart_init(void);


#endif
