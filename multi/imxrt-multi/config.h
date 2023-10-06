/*
 * Phoenix-RTOS
 *
 * iMX RT multi driver config.
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _CONFIG_H_
#define _CONFIG_H_

#if defined(__CPU_IMXRT105X)
#include "imxrt1050.h"
#elif defined(__CPU_IMXRT106X)
#include "imxrt1060.h"
#elif defined(__CPU_IMXRT117X)
#include "imxrt1170.h"
#else
#warning "No target specified! Falling back to default"
#include "imxrt1060.h"
#endif

#ifndef IMXRT_MULTI_PRIO
#define IMXRT_MULTI_PRIO 2
#endif

/* UART */

#ifndef UART1
#define UART1 1
#endif

#ifndef UART1_HW_FLOWCTRL
#define UART1_HW_FLOWCTRL 0
#endif

#ifndef UART1_BAUDRATE
#define UART1_BAUDRATE 115200
#endif

#ifndef UART2
#define UART2 0
#endif

#ifndef UART2_HW_FLOWCTRL
#define UART2_HW_FLOWCTRL 0
#endif

#ifndef UART2_BAUDRATE
#define UART2_BAUDRATE 115200
#endif

#ifndef UART3
#define UART3 0
#endif

#ifndef UART3_HW_FLOWCTRL
#define UART3_HW_FLOWCTRL 0
#endif

#ifndef UART3_BAUDRATE
#define UART3_BAUDRATE 115200
#endif

#ifndef UART4
#define UART4 0
#endif

#ifndef UART4_HW_FLOWCTRL
#define UART4_HW_FLOWCTRL 0
#endif

#ifndef UART4_BAUDRATE
#define UART4_BAUDRATE 115200
#endif

#ifndef UART5
#define UART5 0
#endif

#ifndef UART5_HW_FLOWCTRL
#define UART5_HW_FLOWCTRL 0
#endif

#ifndef UART5_BAUDRATE
#define UART5_BAUDRATE 115200
#endif

#ifndef UART6
#define UART6 0
#endif

#ifndef UART6_HW_FLOWCTRL
#define UART6_HW_FLOWCTRL 0
#endif

#ifndef UART6_BAUDRATE
#define UART6_BAUDRATE 115200
#endif

#ifndef UART7
#define UART7 0
#endif

#ifndef UART7_HW_FLOWCTRL
#define UART7_HW_FLOWCTRL 0
#endif

#ifndef UART7_BAUDRATE
#define UART7_BAUDRATE 115200
#endif

#ifndef UART8
#define UART8 0
#endif

#ifndef UART8_HW_FLOWCTRL
#define UART8_HW_FLOWCTRL 0
#endif

#ifndef UART8_BAUDRATE
#define UART8_BAUDRATE 115200
#endif

#ifdef __CPU_IMXRT117X

#ifndef UART9
#define UART9 0
#endif

#ifndef UART9_HW_FLOWCTRL
#define UART9_HW_FLOWCTRL 0
#endif

#ifndef UART9_BAUDRATE
#define UART9_BAUDRATE 115200
#endif

#ifndef UART10
#define UART10 0
#endif

#ifndef UART10_HW_FLOWCTRL
#define UART10_HW_FLOWCTRL 0
#endif

#ifndef UART10_BAUDRATE
#define UART10_BAUDRATE 115200
#endif

#ifndef UART11
#define UART11 1
#endif

#ifndef UART11_HW_FLOWCTRL
#define UART11_HW_FLOWCTRL 0
#endif

#ifndef UART11_BAUDRATE
#define UART11_BAUDRATE 115200
#endif

#ifndef UART12
#define UART12 0
#endif

#ifndef UART12_HW_FLOWCTRL
#define UART12_HW_FLOWCTRL 0
#endif

#ifndef UART12_BAUDRATE
#define UART12_BAUDRATE 115200
#endif

#else
#define UART9 0
#define UART9_HW_FLOWCTRL 0
#define UART9_BAUDRATE 0
#define UART10 0
#define UART10_HW_FLOWCTRL 0
#define UART10_BAUDRATE 0
#define UART11 0
#define UART11_HW_FLOWCTRL 0
#define UART11_BAUDRATE 0
#define UART12 0
#define UART12_HW_FLOWCTRL 0
#define UART12_BAUDRATE 0
#endif

#define UART_BAUDRATES \
	UART1_BAUDRATE, \
	UART3_BAUDRATE, \
	UART3_BAUDRATE, \
	UART4_BAUDRATE, \
	UART5_BAUDRATE, \
	UART6_BAUDRATE, \
	UART7_BAUDRATE, \
	UART8_BAUDRATE, \
	UART9_BAUDRATE, \
	UART10_BAUDRATE, \
	UART11_BAUDRATE, \
	UART12_BAUDRATE

#ifndef UART_CONSOLE
#if defined(__CPU_IMXRT105X)
#define UART_CONSOLE 1
#elif defined(__CPU_IMXRT106X)
#define UART_CONSOLE 1
#elif defined(__CPU_IMXRT117X)
#define UART_CONSOLE 11
#else
#define UART_CONSOLE 1
#endif
#endif

/* SPI */

#ifndef SPI1
#define SPI1 1
#endif

#ifndef SPI2
#define SPI2 0
#endif

#ifndef SPI3
#define SPI3 0
#endif

#ifndef SPI4
#define SPI4 0
#endif

#ifndef SPI5
#define SPI5 0
#endif

#ifndef SPI6
#define SPI6 0
#endif


/* I2C */

#ifndef I2C1
#define I2C1 1
#endif

#ifndef I2C2
#define I2C2 0
#endif

#ifndef I2C3
#define I2C3 0
#endif

#ifndef I2C4
#define I2C4 0
#endif

/* TRNG */

#ifndef TRNG
#define TRNG 0
#endif

/* libdummyfs */
#ifndef BUILTIN_DUMMYFS
#define BUILTIN_DUMMYFS 0
#endif

/* Cortex M4 */

#ifdef __CPU_IMXRT117X

#ifndef CM4
#define CM4 1
#endif

#endif

#endif
