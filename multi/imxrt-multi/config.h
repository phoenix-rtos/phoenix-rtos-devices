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

#if defined(TARGET_IMXRT1050)
#include "imxrt1050.h"
#elif defined(TARGET_IMXRT1060)
#include "imxrt1060.h"
#elif defined(TARGET_IMXRT1170)
#include "imxrt1170.h"
#else
#warning "No target specified! Falling back to default"
#include "imxrt1060.h"
#endif

/* UART */

#ifndef UART1
#define UART1 1
#endif

#ifndef UART1_HW_FLOWCTRL
#define UART1_HW_FLOWCTRL 0
#endif

#ifndef UART2
#define UART2 0
#endif

#ifndef UART2_HW_FLOWCTRL
#define UART2_HW_FLOWCTRL 0
#endif

#ifndef UART3
#define UART3 0
#endif

#ifndef UART3_HW_FLOWCTRL
#define UART3_HW_FLOWCTRL 0
#endif

#ifndef UART4
#define UART4 0
#endif

#ifndef UART4_HW_FLOWCTRL
#define UART4_HW_FLOWCTRL 0
#endif

#ifndef UART5
#define UART5 0
#endif

#ifndef UART5_HW_FLOWCTRL
#define UART5_HW_FLOWCTRL 0
#endif

#ifndef UART6
#define UART6 0
#endif

#ifndef UART6_HW_FLOWCTRL
#define UART6_HW_FLOWCTRL 0
#endif

#ifndef UART7
#define UART7 0
#endif

#ifndef UART7_HW_FLOWCTRL
#define UART7_HW_FLOWCTRL 0
#endif

#ifndef UART8
#define UART8 0
#endif

#ifndef UART8_HW_FLOWCTRL
#define UART8_HW_FLOWCTRL 0
#endif

#ifdef TARGET_IMXRT1170

#ifndef UART9
#define UART9 0
#endif

#ifndef UART9_HW_FLOWCTRL
#define UART9_HW_FLOWCTRL 0
#endif

#ifndef UART10
#define UART10 0
#endif

#ifndef UART10_HW_FLOWCTRL
#define UART10_HW_FLOWCTRL 0
#endif

#ifndef UART11
#define UART11 1
#endif

#ifndef UART11_HW_FLOWCTRL
#define UART11_HW_FLOWCTRL 0
#endif

#ifndef UART12
#define UART12 0
#endif

#ifndef UART12_HW_FLOWCTRL
#define UART12_HW_FLOWCTRL 0
#endif

#else
#define UART9 0
#define UART9_HW_FLOWCTRL 0
#define UART10 0
#define UART10_HW_FLOWCTRL 0
#define UART11 0
#define UART11_HW_FLOWCTRL 0
#define UART12 0
#define UART12_HW_FLOWCTRL 0
#endif

#ifndef UART_CONSOLE
#if defined(TARGET_IMXRT1050)
#define UART_CONSOLE 1
#elif defined(TARGET_IMXRT1060)
#define UART_CONSOLE 1
#elif defined(TARGET_IMXRT1170)
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

#endif
