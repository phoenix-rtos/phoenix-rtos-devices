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


enum { uart1 = 0, uart2, uart3, uart4, uart5, uart6, uart7, uart8 };


int uart_handleMsg(msg_t *msg, int dev);


int uart_init(void);


#endif
