/*
 * Phoenix-RTOS
 *
 * Operating system kernel
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

enum { UART1_BIT = 1 << 0,
	   UART2_BIT = 1 << 1,
	   UART3_BIT = 1 << 2,
	   UART4_BIT = 1 << 3,
	   UART5_BIT = 1 << 4 };


enum { UARTDRV_DEF = 0, UARTDRV_GET, UARTDRV_ENABLE };


enum { UARTDRV_MNORMAL = 0, UARTDRV_MNBLOCK };


enum { UARTDRV_PARNONE = 0, UARTDRV_PAREVEN, UARTDRV_PARODD };

#endif
