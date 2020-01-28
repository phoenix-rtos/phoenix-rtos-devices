/*
 * Phoenix-RTOS
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

#ifndef _UART_H_
#define _UART_H_


int uart_configure(int uart, char bits, char parity, unsigned int baud, char enable);


int uart_write(int uart, void* buff, unsigned int bufflen);


int uart_read(int uart, void* buff, unsigned int count, char mode, unsigned int timeout);


int uart_init(void);


#endif
