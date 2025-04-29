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

#define IIR_IRQPEND   0x01
#define IIR_CODE_MS   0x00 /* Modem Status */
#define IIR_CODE_THRE 0x01 /* Transmitter Holding Register Empty */
#define IIR_CODE_DR   0x02 /* Received Data Ready */
#define IIR_CODE_LS   0x03 /* Receiver Line Status */
#define IIR_CODE_RTO  0x06 /* Reception Timeout */

#define LCR_DLAB 0x80
#define LCR_D8N1 0x03
#define LCR_D8N2 0x07

#define MCR_OUT2 0x08
#define MCR_OUT1 0x04
#define MCR_RTS  0x02
#define MCR_DTR  0x01

#define LSR_DR   0x01
#define LSR_THRE 0x20


#endif
