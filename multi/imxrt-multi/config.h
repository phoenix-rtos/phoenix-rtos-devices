/*
 * Phoenix-RTOS
 *
 * iMX RT multi driver config.
 *
 * Copyright 2019-2024 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "helpers.h"

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

#include <board_config.h>


#ifndef IMXRT_MULTI_PRIO
#define IMXRT_MULTI_PRIO 2
#endif

/* UART */

#ifndef UART1
#define UART1          0
#define UART1_BAUDRATE 0
#elif !ISBOOLEAN(UART1)
#error "UART1 must have a value of 0, 1, or be undefined"
#endif

#if UART1

#ifndef UART1_TX_PIN
#warning "[Deprecated] UART1_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART1_TX_PIN UART1_TX_PIN_DEFAULT
#elif ISEMPTY(UART1_TX_PIN)
#error "UART1_TX_PIN must not be empty"
#endif

#ifndef UART1_RX_PIN
#warning "[Deprecated] UART1_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART1_RX_PIN UART1_RX_PIN_DEFAULT
#elif ISEMPTY(UART1_RX_PIN)
#error "UART1_RX_PIN must not be empty"
#endif

#ifdef UART1_HW_FLOWCTRL
#warning "[Deprecated] UART1_HW_FLOWCTRL should not be used. Defines for UART1_RTS_PIN and/or UART1_CTS_PIN are enough"
#if ISTRUTHY(UART1_HW_FLOWCTRL)
#define UART1_RTS_PIN UART1_RTS_PIN_DEFAULT
#define UART1_CTS_PIN UART1_CTS_PIN_DEFAULT
#else
#define UART1_RTS_PIN
#define UART1_CTS_PIN
#endif
#endif

#ifndef UART1_RTS_PIN
#warning "[Deprecated] UART1_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART1_RTS_PIN UART1_RTS_PIN_DEFAULT
#endif

#ifndef UART1_CTS_PIN
#warning "[Deprecated] UART1_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART1_CTS_PIN UART1_CTS_PIN_DEFAULT
#endif

#ifndef UART1_BAUDRATE
#warning "[Deprecated] UART1_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART1_BAUDRATE 115200
#endif

#endif


#ifndef UART2
#define UART2          0
#define UART2_BAUDRATE 0
#elif !ISBOOLEAN(UART2)
#error "UART2 must have a value of 0, 1, or be undefined"
#endif

#if UART2

#ifndef UART2_TX_PIN
#warning "[Deprecated] UART2_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART2_TX_PIN UART2_TX_PIN_DEFAULT
#elif ISEMPTY(UART2_TX_PIN)
#error "UART2_TX_PIN must not be empty"
#endif

#ifndef UART2_RX_PIN
#warning "[Deprecated] UART2_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART2_RX_PIN UART2_RX_PIN_DEFAULT
#elif ISEMPTY(UART2_RX_PIN)
#error "UART2_RX_PIN must not be empty"
#endif

#ifdef UART2_HW_FLOWCTRL
#warning "[Deprecated] UART2_HW_FLOWCTRL should not be used. Defines for UART2_RTS_PIN and/or UART2_CTS_PIN are enough"
#if ISTRUTHY(UART2_HW_FLOWCTRL)
#define UART2_RTS_PIN UART2_RTS_PIN_DEFAULT
#define UART2_CTS_PIN UART2_CTS_PIN_DEFAULT
#else
#define UART2_RTS_PIN
#define UART2_CTS_PIN
#endif
#endif

#ifndef UART2_RTS_PIN
#warning "[Deprecated] UART2_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART2_RTS_PIN UART2_RTS_PIN_DEFAULT
#endif

#ifndef UART2_CTS_PIN
#warning "[Deprecated] UART2_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART2_CTS_PIN UART2_CTS_PIN_DEFAULT
#endif

#ifndef UART2_BAUDRATE
#warning "[Deprecated] UART2_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART2_BAUDRATE 115200
#endif

#endif


#ifndef UART3
#define UART3          0
#define UART3_BAUDRATE 0
#elif !ISBOOLEAN(UART3)
#error "UART3 must have a value of 0, 1, or be undefined"
#endif

#if UART3

#ifndef UART3_TX_PIN
#warning "[Deprecated] UART3_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART3_TX_PIN UART3_TX_PIN_DEFAULT
#elif ISEMPTY(UART3_TX_PIN)
#error "UART3_TX_PIN must not be empty"
#endif

#ifndef UART3_RX_PIN
#warning "[Deprecated] UART3_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART3_RX_PIN UART3_RX_PIN_DEFAULT
#elif ISEMPTY(UART3_RX_PIN)
#error "UART3_RX_PIN must not be empty"
#endif

#ifdef UART3_HW_FLOWCTRL
#warning "[Deprecated] UART3_HW_FLOWCTRL should not be used. Defines for UART3_RTS_PIN and/or UART3_CTS_PIN are enough"
#if ISTRUTHY(UART3_HW_FLOWCTRL)
#define UART3_RTS_PIN UART3_RTS_PIN_DEFAULT
#define UART3_CTS_PIN UART3_CTS_PIN_DEFAULT
#else
#define UART3_RTS_PIN
#define UART3_CTS_PIN
#endif
#endif

#ifndef UART3_RTS_PIN
#warning "[Deprecated] UART3_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART3_RTS_PIN UART3_RTS_PIN_DEFAULT
#endif

#ifndef UART3_CTS_PIN
#warning "[Deprecated] UART3_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART3_CTS_PIN UART3_CTS_PIN_DEFAULT
#endif

#ifndef UART3_BAUDRATE
#warning "[Deprecated] UART3_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART3_BAUDRATE 115200
#endif

#endif


#ifndef UART4
#define UART4          0
#define UART4_BAUDRATE 0
#elif !ISBOOLEAN(UART4)
#error "UART4 must have a value of 0, 1, or be undefined"
#endif

#if UART4

#ifndef UART4_TX_PIN
#warning "[Deprecated] UART4_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART4_TX_PIN UART4_TX_PIN_DEFAULT
#elif ISEMPTY(UART4_TX_PIN)
#error "UART4_TX_PIN must not be empty"
#endif

#ifndef UART4_RX_PIN
#warning "[Deprecated] UART4_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART4_RX_PIN UART4_RX_PIN_DEFAULT
#elif ISEMPTY(UART4_RX_PIN)
#error "UART4_RX_PIN must not be empty"
#endif

#ifdef UART4_HW_FLOWCTRL
#warning "[Deprecated] UART4_HW_FLOWCTRL should not be used. Defines for UART4_RTS_PIN and/or UART4_CTS_PIN are enough"
#if ISTRUTHY(UART4_HW_FLOWCTRL)
#define UART4_RTS_PIN UART4_RTS_PIN_DEFAULT
#define UART4_CTS_PIN UART4_CTS_PIN_DEFAULT
#else
#define UART4_RTS_PIN
#define UART4_CTS_PIN
#endif
#endif

#ifndef UART4_RTS_PIN
#warning "[Deprecated] UART4_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART4_RTS_PIN UART4_RTS_PIN_DEFAULT
#endif

#ifndef UART4_CTS_PIN
#warning "[Deprecated] UART4_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART4_CTS_PIN UART4_CTS_PIN_DEFAULT
#endif

#ifndef UART4_BAUDRATE
#warning "[Deprecated] UART4_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART4_BAUDRATE 115200
#endif

#endif


#ifndef UART5
#define UART5          0
#define UART5_BAUDRATE 0
#elif !ISBOOLEAN(UART5)
#error "UART5 must have a value of 0, 1, or be undefined"
#endif

#if UART5

#ifndef UART5_TX_PIN
#warning "[Deprecated] UART5_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART5_TX_PIN UART5_TX_PIN_DEFAULT
#elif ISEMPTY(UART5_TX_PIN)
#error "UART5_TX_PIN must not be empty"
#endif

#ifndef UART5_RX_PIN
#warning "[Deprecated] UART5_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART5_RX_PIN UART5_RX_PIN_DEFAULT
#elif ISEMPTY(UART5_RX_PIN)
#error "UART5_RX_PIN must not be empty"
#endif

#ifdef UART5_HW_FLOWCTRL
#warning "[Deprecated] UART5_HW_FLOWCTRL should not be used. Defines for UART5_RTS_PIN and/or UART5_CTS_PIN are enough"
#if ISTRUTHY(UART5_HW_FLOWCTRL)
#define UART5_RTS_PIN UART5_RTS_PIN_DEFAULT
#define UART5_CTS_PIN UART5_CTS_PIN_DEFAULT
#else
#define UART5_RTS_PIN
#define UART5_CTS_PIN
#endif
#endif

#ifndef UART5_RTS_PIN
#warning "[Deprecated] UART5_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART5_RTS_PIN UART5_RTS_PIN_DEFAULT
#endif

#ifndef UART5_CTS_PIN
#warning "[Deprecated] UART5_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART5_CTS_PIN UART5_CTS_PIN_DEFAULT
#endif

#ifndef UART5_BAUDRATE
#warning "[Deprecated] UART5_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART5_BAUDRATE 115200
#endif

#endif


#ifndef UART6
#define UART6          0
#define UART6_BAUDRATE 0
#elif !ISBOOLEAN(UART6)
#error "UART6 must have a value of 0, 1, or be undefined"
#endif

#if UART6

#ifndef UART6_TX_PIN
#warning "[Deprecated] UART6_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART6_TX_PIN UART6_TX_PIN_DEFAULT
#elif ISEMPTY(UART6_TX_PIN)
#error "UART6_TX_PIN must not be empty"
#endif

#ifndef UART6_RX_PIN
#warning "[Deprecated] UART6_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART6_RX_PIN UART6_RX_PIN_DEFAULT
#elif ISEMPTY(UART6_RX_PIN)
#error "UART6_RX_PIN must not be empty"
#endif

#ifdef UART6_HW_FLOWCTRL
#warning "[Deprecated] UART6_HW_FLOWCTRL should not be used. Defines for UART6_RTS_PIN and/or UART6_CTS_PIN are enough"
#if ISTRUTHY(UART6_HW_FLOWCTRL)
#define UART6_RTS_PIN UART6_RTS_PIN_DEFAULT
#define UART6_CTS_PIN UART6_CTS_PIN_DEFAULT
#else
#define UART6_RTS_PIN
#define UART6_CTS_PIN
#endif
#endif

#ifndef UART6_RTS_PIN
#warning "[Deprecated] UART6_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART6_RTS_PIN UART6_RTS_PIN_DEFAULT
#endif

#ifndef UART6_CTS_PIN
#warning "[Deprecated] UART6_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART6_CTS_PIN UART6_CTS_PIN_DEFAULT
#endif

#ifndef UART6_BAUDRATE
#warning "[Deprecated] UART6_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART6_BAUDRATE 115200
#endif

#endif


#ifndef UART7
#define UART7          0
#define UART7_BAUDRATE 0
#elif !ISBOOLEAN(UART7)
#error "UART7 must have a value of 0, 1, or be undefined"
#endif

#if UART7

#ifndef UART7_TX_PIN
#warning "[Deprecated] UART7_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART7_TX_PIN UART7_TX_PIN_DEFAULT
#elif ISEMPTY(UART7_TX_PIN)
#error "UART7_TX_PIN must not be empty"
#endif

#ifndef UART7_RX_PIN
#warning "[Deprecated] UART7_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART7_RX_PIN UART7_RX_PIN_DEFAULT
#elif ISEMPTY(UART7_RX_PIN)
#error "UART7_RX_PIN must not be empty"
#endif

#ifdef UART7_HW_FLOWCTRL
#warning "[Deprecated] UART7_HW_FLOWCTRL should not be used. Defines for UART7_RTS_PIN and/or UART7_CTS_PIN are enough"
#if ISTRUTHY(UART7_HW_FLOWCTRL)
#define UART7_RTS_PIN UART7_RTS_PIN_DEFAULT
#define UART7_CTS_PIN UART7_CTS_PIN_DEFAULT
#else
#define UART7_RTS_PIN
#define UART7_CTS_PIN
#endif
#endif

#ifndef UART7_RTS_PIN
#warning "[Deprecated] UART7_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART7_RTS_PIN UART7_RTS_PIN_DEFAULT
#endif

#ifndef UART7_CTS_PIN
#warning "[Deprecated] UART7_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART7_CTS_PIN UART7_CTS_PIN_DEFAULT
#endif

#ifndef UART7_BAUDRATE
#warning "[Deprecated] UART7_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART7_BAUDRATE 115200
#endif

#endif


#ifndef UART8
#define UART8          0
#define UART8_BAUDRATE 0
#elif !ISBOOLEAN(UART8)
#error "UART8 must have a value of 0, 1, or be undefined"
#endif

#if UART8

#ifndef UART8_TX_PIN
#warning "[Deprecated] UART8_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART8_TX_PIN UART8_TX_PIN_DEFAULT
#elif ISEMPTY(UART8_TX_PIN)
#error "UART8_TX_PIN must not be empty"
#endif

#ifndef UART8_RX_PIN
#warning "[Deprecated] UART8_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART8_RX_PIN UART8_RX_PIN_DEFAULT
#elif ISEMPTY(UART8_RX_PIN)
#error "UART8_RX_PIN must not be empty"
#endif

#ifdef UART8_HW_FLOWCTRL
#warning "[Deprecated] UART8_HW_FLOWCTRL should not be used. Defines for UART8_RTS_PIN and/or UART8_CTS_PIN are enough"
#if ISTRUTHY(UART8_HW_FLOWCTRL)
#define UART8_RTS_PIN UART8_RTS_PIN_DEFAULT
#define UART8_CTS_PIN UART8_CTS_PIN_DEFAULT
#else
#define UART8_RTS_PIN
#define UART8_CTS_PIN
#endif
#endif

#ifndef UART8_RTS_PIN
#warning "[Deprecated] UART8_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART8_RTS_PIN UART8_RTS_PIN_DEFAULT
#endif

#ifndef UART8_CTS_PIN
#warning "[Deprecated] UART8_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART8_CTS_PIN UART8_CTS_PIN_DEFAULT
#endif

#ifndef UART8_BAUDRATE
#warning "[Deprecated] UART8_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART8_BAUDRATE 115200
#endif

#endif


#ifdef __CPU_IMXRT117X


#ifndef UART9
#define UART9          0
#define UART9_BAUDRATE 0
#elif !ISBOOLEAN(UART9)
#error "UART9 must have a value of 0, 1, or be undefined"
#endif

#if UART9

#ifndef UART9_TX_PIN
#warning "[Deprecated] UART9_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART9_TX_PIN UART9_TX_PIN_DEFAULT
#elif ISEMPTY(UART9_TX_PIN)
#error "UART9_TX_PIN must not be empty"
#endif

#ifndef UART9_RX_PIN
#warning "[Deprecated] UART9_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART9_RX_PIN UART9_RX_PIN_DEFAULT
#elif ISEMPTY(UART9_RX_PIN)
#error "UART9_RX_PIN must not be empty"
#endif

#ifdef UART9_HW_FLOWCTRL
#warning "[Deprecated] UART9_HW_FLOWCTRL should not be used. Defines for UART9_RTS_PIN and/or UART9_CTS_PIN are enough"
#if ISTRUTHY(UART9_HW_FLOWCTRL)
#define UART9_RTS_PIN UART9_RTS_PIN_DEFAULT
#define UART9_CTS_PIN UART9_CTS_PIN_DEFAULT
#else
#define UART9_RTS_PIN
#define UART9_CTS_PIN
#endif
#endif

#ifndef UART9_RTS_PIN
#warning "[Deprecated] UART9_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART9_RTS_PIN UART9_RTS_PIN_DEFAULT
#endif

#ifndef UART9_CTS_PIN
#warning "[Deprecated] UART9_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART9_CTS_PIN UART9_CTS_PIN_DEFAULT
#endif

#ifndef UART9_BAUDRATE
#warning "[Deprecated] UART9_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART9_BAUDRATE 115200
#endif

#endif


#ifndef UART10
#define UART10          0
#define UART10_BAUDRATE 0
#elif !ISBOOLEAN(UART10)
#error "UART10 must have a value of 0, 1, or be undefined"
#endif

#if UART10

#ifndef UART10_TX_PIN
#warning "[Deprecated] UART10_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART10_TX_PIN UART10_TX_PIN_DEFAULT
#elif ISEMPTY(UART10_TX_PIN)
#error "UART10_TX_PIN must not be empty"
#endif

#ifndef UART10_RX_PIN
#warning "[Deprecated] UART10_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART10_RX_PIN UART10_RX_PIN_DEFAULT
#elif ISEMPTY(UART10_RX_PIN)
#error "UART10_RX_PIN must not be empty"
#endif

#ifdef UART10_HW_FLOWCTRL
#warning "[Deprecated] UART10_HW_FLOWCTRL should not be used. Defines for UART10_RTS_PIN and/or UART10_CTS_PIN are enough"
#if ISTRUTHY(UART10_HW_FLOWCTRL)
#define UART10_RTS_PIN UART10_RTS_PIN_DEFAULT
#define UART10_CTS_PIN UART10_CTS_PIN_DEFAULT
#else
#define UART10_RTS_PIN
#define UART10_CTS_PIN
#endif
#endif

#ifndef UART10_RTS_PIN
#warning "[Deprecated] UART10_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART10_RTS_PIN UART10_RTS_PIN_DEFAULT
#endif

#ifndef UART10_CTS_PIN
#warning "[Deprecated] UART10_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART10_CTS_PIN UART10_CTS_PIN_DEFAULT
#endif

#ifndef UART10_BAUDRATE
#warning "[Deprecated] UART10_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART10_BAUDRATE 115200
#endif

#endif


#ifndef UART11
#define UART11          0
#define UART11_BAUDRATE 0
#elif !ISBOOLEAN(UART11)
#error "UART11 must have a value of 0, 1, or be undefined"
#endif

#if UART11

#ifndef UART11_TX_PIN
#warning "[Deprecated] UART11_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART11_TX_PIN UART11_TX_PIN_DEFAULT
#elif ISEMPTY(UART11_TX_PIN)
#error "UART11_TX_PIN must not be empty"
#endif

#ifndef UART11_RX_PIN
#warning "[Deprecated] UART11_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART11_RX_PIN UART11_RX_PIN_DEFAULT
#elif ISEMPTY(UART11_RX_PIN)
#error "UART11_RX_PIN must not be empty"
#endif

#ifdef UART11_HW_FLOWCTRL
#warning "[Deprecated] UART11_HW_FLOWCTRL should not be used. Defines for UART11_RTS_PIN and/or UART11_CTS_PIN are enough"
#if ISTRUTHY(UART11_HW_FLOWCTRL)
#define UART11_RTS_PIN UART11_RTS_PIN_DEFAULT
#define UART11_CTS_PIN UART11_CTS_PIN_DEFAULT
#else
#define UART11_RTS_PIN
#define UART11_CTS_PIN
#endif
#endif

#ifndef UART11_RTS_PIN
#warning "[Deprecated] UART11_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART11_RTS_PIN UART11_RTS_PIN_DEFAULT
#endif

#ifndef UART11_CTS_PIN
#warning "[Deprecated] UART11_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART11_CTS_PIN UART11_CTS_PIN_DEFAULT
#endif

#ifndef UART11_BAUDRATE
#warning "[Deprecated] UART11_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART11_BAUDRATE 115200
#endif

#endif


#ifndef UART12
#define UART12          0
#define UART12_BAUDRATE 0
#elif !ISBOOLEAN(UART12)
#error "UART12 must have a value of 0, 1, or be undefined"
#endif

#if UART12

#ifndef UART12_TX_PIN
#warning "[Deprecated] UART12_TX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART12_TX_PIN UART12_TX_PIN_DEFAULT
#elif ISEMPTY(UART12_TX_PIN)
#error "UART12_TX_PIN must not be empty"
#endif

#ifndef UART12_RX_PIN
#warning "[Deprecated] UART12_RX_PIN should be explicitly defined in board_config.h. Using the default"
#define UART12_RX_PIN UART12_RX_PIN_DEFAULT
#elif ISEMPTY(UART12_RX_PIN)
#error "UART12_RX_PIN must not be empty"
#endif

#ifdef UART12_HW_FLOWCTRL
#warning "[Deprecated] UART12_HW_FLOWCTRL should not be used. Defines for UART12_RTS_PIN and/or UART12_CTS_PIN are enough"
#if ISTRUTHY(UART12_HW_FLOWCTRL)
#define UART12_RTS_PIN UART12_RTS_PIN_DEFAULT
#define UART12_CTS_PIN UART12_CTS_PIN_DEFAULT
#else
#define UART12_RTS_PIN
#define UART12_CTS_PIN
#endif
#endif

#ifndef UART12_RTS_PIN
#warning "[Deprecated] UART12_RTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART12_RTS_PIN UART12_RTS_PIN_DEFAULT
#endif

#ifndef UART12_CTS_PIN
#warning "[Deprecated] UART12_CTS_PIN should be explicitly defined (may be empty) in board_config.h. Using the default"
#define UART12_CTS_PIN UART12_CTS_PIN_DEFAULT
#endif

#ifndef UART12_BAUDRATE
#warning "[Deprecated] UART12_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART12_BAUDRATE 115200
#endif

#endif


#else

#define UART9           0
#define UART9_BAUDRATE  0
#define UART10          0
#define UART10_BAUDRATE 0
#define UART11          0
#define UART11_BAUDRATE 0
#define UART12          0
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
#define SPI1 0
#elif !ISBOOLEAN(SPI1)
#error "SPI1 must have a value of 0, 1, or be undefined"
#endif

#if SPI1

#ifndef SPI1_SCK
#warning "[Deprecated] SPI1_SCK should be explicitly defined in board_config.h. Using the default"
#define SPI1_SCK SPI1_SCK_DEFAULT
#elif ISEMPTY(SPI1_SCK)
#error "SPI1_SCK must not be empty"
#endif

#ifndef SPI1_SD0
#warning "[Deprecated] SPI1_SD0 should be explicitly defined in board_config.h. Using the default"
#define SPI1_SD0 SPI1_SD0_DEFAULT
#elif ISEMPTY(SPI1_SD0)
#error "SPI1_SD0 must not be empty"
#endif

#ifndef SPI1_SDI
#warning "[Deprecated] SPI1_SDI should be explicitly defined in board_config.h. Using the default"
#define SPI1_SDI SPI1_SDI_DEFAULT
#elif ISEMPTY(SPI1_SDI)
#error "SPI1_SDI must not be empty"
#endif

#ifndef SPI1_PCS0
#warning "[Deprecated] SPI1_PCS0 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI1_PCS0 SPI1_PCS0_DEFAULT
#endif

#ifndef SPI1_PCS1
#warning "[Deprecated] SPI1_PCS1 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI1_PCS1 SPI1_PCS1_DEFAULT
#endif

#ifndef SPI1_PCS2
#warning "[Deprecated] SPI1_PCS2 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI1_PCS2 SPI1_PCS2_DEFAULT
#endif

#ifndef SPI1_PCS3
#warning "[Deprecated] SPI1_PCS3 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI1_PCS3 SPI1_PCS3_DEFAULT
#endif

#endif


#ifndef SPI2
#define SPI2 0
#elif !ISBOOLEAN(SPI2)
#error "SPI2 must have a value of 0, 1, or be undefined"
#endif

#if SPI2

#ifndef SPI2_SCK
#warning "[Deprecated] SPI2_SCK should be explicitly defined in board_config.h. Using the default"
#define SPI2_SCK SPI2_SCK_DEFAULT
#elif ISEMPTY(SPI2_SCK)
#error "SPI2_SCK must not be empty"
#endif

#ifndef SPI2_SD0
#warning "[Deprecated] SPI2_SD0 should be explicitly defined in board_config.h. Using the default"
#define SPI2_SD0 SPI2_SD0_DEFAULT
#elif ISEMPTY(SPI2_SD0)
#error "SPI2_SD0 must not be empty"
#endif

#ifndef SPI2_SDI
#warning "[Deprecated] SPI2_SDI should be explicitly defined in board_config.h. Using the default"
#define SPI2_SDI SPI2_SDI_DEFAULT
#elif ISEMPTY(SPI2_SDI)
#error "SPI2_SDI must not be empty"
#endif

#ifndef SPI2_PCS0
#warning "[Deprecated] SPI2_PCS0 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI2_PCS0 SPI2_PCS0_DEFAULT
#endif

#ifndef SPI2_PCS1
#warning "[Deprecated] SPI2_PCS1 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI2_PCS1 SPI2_PCS1_DEFAULT
#endif

#ifndef SPI2_PCS2
#warning "[Deprecated] SPI2_PCS2 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI2_PCS2 SPI2_PCS2_DEFAULT
#endif

#ifndef SPI2_PCS3
#warning "[Deprecated] SPI2_PCS3 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI2_PCS3 SPI2_PCS3_DEFAULT
#endif

#endif


#ifndef SPI3
#define SPI3 0
#elif !ISBOOLEAN(SPI3)
#error "SPI3 must have a value of 0, 1, or be undefined"
#endif

#if SPI3

#ifndef SPI3_SCK
#warning "[Deprecated] SPI3_SCK should be explicitly defined in board_config.h. Using the default"
#define SPI3_SCK SPI3_SCK_DEFAULT
#elif ISEMPTY(SPI3_SCK)
#error "SPI3_SCK must not be empty"
#endif

#ifndef SPI3_SD0
#warning "[Deprecated] SPI3_SD0 should be explicitly defined in board_config.h. Using the default"
#define SPI3_SD0 SPI3_SD0_DEFAULT
#elif ISEMPTY(SPI3_SD0)
#error "SPI3_SD0 must not be empty"
#endif

#ifndef SPI3_SDI
#warning "[Deprecated] SPI3_SDI should be explicitly defined in board_config.h. Using the default"
#define SPI3_SDI SPI3_SDI_DEFAULT
#elif ISEMPTY(SPI3_SDI)
#error "SPI3_SDI must not be empty"
#endif

#ifndef SPI3_PCS0
#warning "[Deprecated] SPI3_PCS0 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI3_PCS0 SPI3_PCS0_DEFAULT
#endif

#ifndef SPI3_PCS1
#warning "[Deprecated] SPI3_PCS1 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI3_PCS1 SPI3_PCS1_DEFAULT
#endif

#ifndef SPI3_PCS2
#warning "[Deprecated] SPI3_PCS2 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI3_PCS2 SPI3_PCS2_DEFAULT
#endif

#ifndef SPI3_PCS3
#warning "[Deprecated] SPI3_PCS3 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI3_PCS3 SPI3_PCS3_DEFAULT
#endif

#endif


#ifndef SPI4
#define SPI4 0
#elif !ISBOOLEAN(SPI4)
#error "SPI4 must have a value of 0, 1, or be undefined"
#endif

#if SPI4

#ifndef SPI4_SCK
#warning "[Deprecated] SPI4_SCK should be explicitly defined in board_config.h. Using the default"
#define SPI4_SCK SPI4_SCK_DEFAULT
#elif ISEMPTY(SPI4_SCK)
#error "SPI4_SCK must not be empty"
#endif

#ifndef SPI4_SD0
#warning "[Deprecated] SPI4_SD0 should be explicitly defined in board_config.h. Using the default"
#define SPI4_SD0 SPI4_SD0_DEFAULT
#elif ISEMPTY(SPI4_SD0)
#error "SPI4_SD0 must not be empty"
#endif

#ifndef SPI4_SDI
#warning "[Deprecated] SPI4_SDI should be explicitly defined in board_config.h. Using the default"
#define SPI4_SDI SPI4_SDI_DEFAULT
#elif ISEMPTY(SPI4_SDI)
#error "SPI4_SDI must not be empty"
#endif

#ifndef SPI4_PCS0
#warning "[Deprecated] SPI4_PCS0 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI4_PCS0 SPI4_PCS0_DEFAULT
#endif

#ifndef SPI4_PCS1
#warning "[Deprecated] SPI4_PCS1 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI4_PCS1 SPI4_PCS1_DEFAULT
#endif

#ifndef SPI4_PCS2
#warning "[Deprecated] SPI4_PCS2 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI4_PCS2 SPI4_PCS2_DEFAULT
#endif

#ifndef SPI4_PCS3
#warning "[Deprecated] SPI4_PCS3 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI4_PCS3 SPI4_PCS3_DEFAULT
#endif

#endif


#ifdef __CPU_IMXRT117X


#ifndef SPI5
#define SPI5 0
#elif !ISBOOLEAN(SPI5)
#error "SPI5 must have a value of 0, 1, or be undefined"
#endif

#if SPI5

#ifndef SPI5_SCK
#warning "[Deprecated] SPI5_SCK should be explicitly defined in board_config.h. Using the default"
#define SPI5_SCK SPI5_SCK_DEFAULT
#elif ISEMPTY(SPI5_SCK)
#error "SPI5_SCK must not be empty"
#endif

#ifndef SPI5_SD0
#warning "[Deprecated] SPI5_SD0 should be explicitly defined in board_config.h. Using the default"
#define SPI5_SD0 SPI5_SD0_DEFAULT
#elif ISEMPTY(SPI5_SD0)
#error "SPI5_SD0 must not be empty"
#endif

#ifndef SPI5_SDI
#warning "[Deprecated] SPI5_SDI should be explicitly defined in board_config.h. Using the default"
#define SPI5_SDI SPI5_SDI_DEFAULT
#elif ISEMPTY(SPI5_SDI)
#error "SPI5_SDI must not be empty"
#endif

#ifndef SPI5_PCS0
#warning "[Deprecated] SPI5_PCS0 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI5_PCS0 SPI5_PCS0_DEFAULT
#endif

#ifndef SPI5_PCS1
#warning "[Deprecated] SPI5_PCS1 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI5_PCS1 SPI5_PCS1_DEFAULT
#endif

#ifndef SPI5_PCS2
#warning "[Deprecated] SPI5_PCS2 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI5_PCS2 SPI5_PCS2_DEFAULT
#endif

#ifndef SPI5_PCS3
#warning "[Deprecated] SPI5_PCS3 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI5_PCS3 SPI5_PCS3_DEFAULT
#endif

#endif


#ifndef SPI6
#define SPI6 0
#elif !ISBOOLEAN(SPI6)
#error "SPI6 must have a value of 0, 1, or be undefined"
#endif

#if SPI6

#ifndef SPI6_SCK
#warning "[Deprecated] SPI6_SCK should be explicitly defined in board_config.h. Using the default"
#define SPI6_SCK SPI6_SCK_DEFAULT
#elif ISEMPTY(SPI6_SCK)
#error "SPI6_SCK must not be empty"
#endif

#ifndef SPI6_SD0
#warning "[Deprecated] SPI6_SD0 should be explicitly defined in board_config.h. Using the default"
#define SPI6_SD0 SPI6_SD0_DEFAULT
#elif ISEMPTY(SPI6_SD0)
#error "SPI6_SD0 must not be empty"
#endif

#ifndef SPI6_SDI
#warning "[Deprecated] SPI6_SDI should be explicitly defined in board_config.h. Using the default"
#define SPI6_SDI SPI6_SDI_DEFAULT
#elif ISEMPTY(SPI6_SDI)
#error "SPI6_SDI must not be empty"
#endif

#ifndef SPI6_PCS0
#warning "[Deprecated] SPI6_PCS0 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI6_PCS0 SPI6_PCS0_DEFAULT
#endif

#ifndef SPI6_PCS1
#warning "[Deprecated] SPI6_PCS1 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI6_PCS1 SPI6_PCS1_DEFAULT
#endif

#ifndef SPI6_PCS2
#warning "[Deprecated] SPI6_PCS2 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI6_PCS2 SPI6_PCS2_DEFAULT
#endif

#ifndef SPI6_PCS3
#warning "[Deprecated] SPI6_PCS3 should be explicitly defined (may be empty) in board_config.h. Using the default"
#define SPI6_PCS3 SPI6_PCS3_DEFAULT
#endif

#endif


#else

#define SPI5 0
#define SPI6 0

#endif


#ifndef __CPU_IMXRT117X

/* I2C */

#ifndef I2C1
#define I2C1 0
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
