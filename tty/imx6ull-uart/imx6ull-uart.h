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

enum { MODE_RAW, MODE_TTY };


enum { FL_COOL = 1, FL_SYNC = 2 };


enum { PAR_NONE, PAR_ODD, PAR_EVEN, PAR_MARK, PAR_SPACE };


typedef struct {
	volatile u32 *base;
	u32 mode;
	u32 baud_rate;
	u8 parity;
	u16 dev_no;
	u32 flags;

	u8 rx_buff[256];
	int rx_head;
	int rx_tail;
	handle_t rx_cond;

	u8 tx_buff[256];
	int tx_head;
	int tx_tail;
	handle_t tx_cond;
	int tx_full;

	handle_t cond;
	handle_t inth;
	handle_t lock;
	int ready;
} uart_t;

#endif /* _UARTDRV_H_ */
