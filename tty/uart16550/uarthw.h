/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Hardware abstracion layer interface
 *
 * Copyright 2020-2022 Phoenix Systems
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_UART16550_UARTHW_H_
#define _DEV_UART16550_UARTHW_H_

#include <stdio.h>


extern uint8_t uarthw_read(void *hwctx, unsigned int reg);


extern void uarthw_write(void *hwctx, unsigned int reg, uint8_t val);


extern char *uarthw_dump(void *hwctx, char *s, size_t sz);


extern unsigned int uarthw_irq(void *hwctx);


extern int uarthw_init(unsigned int uartn, void *hwctx, size_t hwctxsz, unsigned int *fclk);


#endif
