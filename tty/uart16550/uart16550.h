/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Copyright 2012-2015, 2020-2022 Phoenix Systems
 * Copyright 2001, 2005, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_SERIAL_H_
#define _DEV_SERIAL_H_

#define SIZE_SERIALS      2
#define SIZE_SERIAL_CHUNK 256


/* UART registers */
#define REG_RBR 0
#define REG_THR 0
#define REG_IMR 1
#define REG_IIR 2
#define REG_LCR 3
#define REG_MCR 4
#define REG_LSR 5
#define REG_MSR 6
#define REG_ADR 7
#define REG_LSB 0
#define REG_MSB 1
#define REG_FCR 2


/* Register bits */
#define IMR_THRE 0x02
#define IMR_DR   0x01

#define IIR_IRQPEND 0x01 /* no interrupt pending (IIR bit 0 = 1) */
#define IIR_THRE    0x02 /* transmitter holding register empty */
#define IIR_DR      0x04 /* received data available */
#define IIR_RLS     0x06 /* error or break condition flag (receiver line status) */
#define IIR_CTOUT   0x0c /* character timeout */

#define LCR_DLAB 0x80
#define LCR_D8N1 0x03
#define LCR_D8N2 0x07

#define MCR_OUT2 0x08

#define LSR_DR   0x01
#define LSR_THRE 0x20
#define LSR_ERR  0x9f /* line status register error bits mask */

#define FCR_FEN  0x01 /* fifo enable */
#define FCR_RXFR 0x02 /* rx fifo reset */
#define FCR_TXFR 0x04 /* tx fifo reset */


#endif
