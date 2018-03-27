/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * i.MX 6ULL UART driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _UARTDRV_H_
#define _UARTDRV_H_

enum { urxd = 0, utxd = 16, ucr1 = 32, ucr2, ucr3, ucr4, ufcr, usr1, usr2,
	uesc, utim, ubir, ubmr, ubrc, onems, uts, umcr };


typedef struct {
	volatile u32 *base;

	u8 rx_buff[256];
	int rx_head;
	int rx_tail;
	handle_t rx_cond;

	u8 tx_buff[256];
	int tx_head;
	int tx_tail;
	handle_t tx_cond;

	handle_t cond;
	handle_t lock;
} uart_t;

#endif /* _UARTDRV_H_ */
