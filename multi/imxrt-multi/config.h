/*
 * Phoenix-RTOS
 *
 * iMX RT multi driver config.
 *
 * Copyright 2019, 2024 Phoenix Systems
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

#ifndef UART_BUFSIZE
#define UART_BUFSIZE 512
#endif


#ifndef UART1
#define UART1 0
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

#if UART1_HW_FLOWCTRL

#ifndef UART1_RTS_PIN
#warning "[Deprecated] UART1_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART1_RTS_PIN UART1_RTS_PIN_DEFAULT
#endif

#ifndef UART1_CTS_PIN
#warning "[Deprecated] UART1_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART1_CTS_PIN UART1_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART1_HW_FLOWCTRL */

#ifndef UART1_BUFSIZE
#define UART1_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART1_BUFSIZE)
#error "UART1_BUFSIZE must not be empty"
#endif

#ifndef UART1_BAUDRATE
#warning "[Deprecated] UART1_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART1_BAUDRATE 115200
#endif

#else
#define UART1_BAUDRATE 0
#define UART1_BUFSIZE  0
#endif /* #if UART1 */


#ifndef UART2
#define UART2 0
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

#if UART2_HW_FLOWCTRL

#ifndef UART2_RTS_PIN
#warning "[Deprecated] UART2_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART2_RTS_PIN UART2_RTS_PIN_DEFAULT
#endif

#ifndef UART2_CTS_PIN
#warning "[Deprecated] UART2_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART2_CTS_PIN UART2_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART2_HW_FLOWCTRL */

#ifndef UART2_BUFSIZE
#define UART2_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART2_BUFSIZE)
#error "UART2_BUFSIZE must not be empty"
#endif

#ifndef UART2_BAUDRATE
#warning "[Deprecated] UART2_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART2_BAUDRATE 115200
#endif

#else
#define UART2_BAUDRATE 0
#define UART2_BUFSIZE  0
#endif /* #if UART2 */


#ifndef UART3
#define UART3 0
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

#if UART3_HW_FLOWCTRL

#ifndef UART3_RTS_PIN
#warning "[Deprecated] UART3_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART3_RTS_PIN UART3_RTS_PIN_DEFAULT
#endif

#ifndef UART3_CTS_PIN
#warning "[Deprecated] UART3_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART3_CTS_PIN UART3_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART3_HW_FLOWCTRL */

#ifndef UART3_BUFSIZE
#define UART3_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART3_BUFSIZE)
#error "UART3_BUFSIZE must not be empty"
#endif

#ifndef UART3_BAUDRATE
#warning "[Deprecated] UART3_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART3_BAUDRATE 115200
#endif

#else
#define UART3_BAUDRATE 0
#define UART3_BUFSIZE  0
#endif /* #if UART3 */


#ifndef UART4
#define UART4 0
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

#if UART4_HW_FLOWCTRL

#ifndef UART4_RTS_PIN
#warning "[Deprecated] UART4_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART4_RTS_PIN UART4_RTS_PIN_DEFAULT
#endif

#ifndef UART4_CTS_PIN
#warning "[Deprecated] UART4_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART4_CTS_PIN UART4_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART4_HW_FLOWCTRL */

#ifndef UART4_BUFSIZE
#define UART4_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART4_BUFSIZE)
#error "UART4_BUFSIZE must not be empty"
#endif

#ifndef UART4_BAUDRATE
#warning "[Deprecated] UART4_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART4_BAUDRATE 115200
#endif

#else
#define UART4_BAUDRATE 0
#define UART4_BUFSIZE  0
#endif /* #if UART4 */


#ifndef UART5
#define UART5 0
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

#if UART5_HW_FLOWCTRL

#ifndef UART5_RTS_PIN
#warning "[Deprecated] UART5_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART5_RTS_PIN UART5_RTS_PIN_DEFAULT
#endif

#ifndef UART5_CTS_PIN
#warning "[Deprecated] UART5_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART5_CTS_PIN UART5_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART5_HW_FLOWCTRL */

#ifndef UART5_BUFSIZE
#define UART5_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART5_BUFSIZE)
#error "UART5_BUFSIZE must not be empty"
#endif

#ifndef UART5_BAUDRATE
#warning "[Deprecated] UART5_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART5_BAUDRATE 115200
#endif

#else
#define UART5_BAUDRATE 0
#define UART5_BUFSIZE  0
#endif /* #if UART5 */


#ifndef UART6
#define UART6 0
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

#if UART6_HW_FLOWCTRL

#ifndef UART6_RTS_PIN
#warning "[Deprecated] UART6_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART6_RTS_PIN UART6_RTS_PIN_DEFAULT
#endif

#ifndef UART6_CTS_PIN
#warning "[Deprecated] UART6_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART6_CTS_PIN UART6_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART6_HW_FLOWCTRL */

#ifndef UART6_BUFSIZE
#define UART6_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART6_BUFSIZE)
#error "UART6_BUFSIZE must not be empty"
#endif

#ifndef UART6_BAUDRATE
#warning "[Deprecated] UART6_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART6_BAUDRATE 115200
#endif

#else
#define UART6_BAUDRATE 0
#define UART6_BUFSIZE  0
#endif /* #if UART6 */


#ifndef UART7
#define UART7 0
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

#if UART7_HW_FLOWCTRL

#ifndef UART7_RTS_PIN
#warning "[Deprecated] UART7_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART7_RTS_PIN UART7_RTS_PIN_DEFAULT
#endif

#ifndef UART7_CTS_PIN
#warning "[Deprecated] UART7_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART7_CTS_PIN UART7_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART7_HW_FLOWCTRL */

#ifndef UART7_BUFSIZE
#define UART7_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART7_BUFSIZE)
#error "UART7_BUFSIZE must not be empty"
#endif

#ifndef UART7_BAUDRATE
#warning "[Deprecated] UART7_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART7_BAUDRATE 115200
#endif

#else
#define UART7_BAUDRATE 0
#define UART7_BUFSIZE  0
#endif /* #if UART7 */


#ifndef UART8
#define UART8 0
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

#if UART8_HW_FLOWCTRL

#ifndef UART8_RTS_PIN
#warning "[Deprecated] UART8_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART8_RTS_PIN UART8_RTS_PIN_DEFAULT
#endif

#ifndef UART8_CTS_PIN
#warning "[Deprecated] UART8_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART8_CTS_PIN UART8_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART8_HW_FLOWCTRL */

#ifndef UART8_BUFSIZE
#define UART8_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART8_BUFSIZE)
#error "UART8_BUFSIZE must not be empty"
#endif

#ifndef UART8_BAUDRATE
#warning "[Deprecated] UART8_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART8_BAUDRATE 115200
#endif

#else
#define UART8_BAUDRATE 0
#define UART8_BUFSIZE  0
#endif /* #if UART8 */


#ifdef __CPU_IMXRT117X


#ifndef UART9
#define UART9 0
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

#if UART9_HW_FLOWCTRL

#ifndef UART9_RTS_PIN
#warning "[Deprecated] UART9_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART9_RTS_PIN UART9_RTS_PIN_DEFAULT
#endif

#ifndef UART9_CTS_PIN
#warning "[Deprecated] UART9_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART9_CTS_PIN UART9_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART9_HW_FLOWCTRL */

#ifndef UART9_BUFSIZE
#define UART9_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART9_BUFSIZE)
#error "UART9_BUFSIZE must not be empty"
#endif

#ifndef UART9_BAUDRATE
#warning "[Deprecated] UART9_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART9_BAUDRATE 115200
#endif

#else
#define UART9_BAUDRATE 0
#define UART9_BUFSIZE  0
#endif /* #if UART9 */


#ifndef UART10
#define UART10 0
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

#if UART10_HW_FLOWCTRL

#ifndef UART10_RTS_PIN
#warning "[Deprecated] UART10_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART10_RTS_PIN UART10_RTS_PIN_DEFAULT
#endif

#ifndef UART10_CTS_PIN
#warning "[Deprecated] UART10_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART10_CTS_PIN UART10_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART10_HW_FLOWCTRL */

#ifndef UART10_BUFSIZE
#define UART10_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART10_BUFSIZE)
#error "UART10_BUFSIZE must not be empty"
#endif

#ifndef UART10_BAUDRATE
#warning "[Deprecated] UART10_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART10_BAUDRATE 115200
#endif

#else
#define UART10_BAUDRATE 0
#define UART10_BUFSIZE  0
#endif /* #if UART10 */


#ifndef UART11
#define UART11 0
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

#if UART11_HW_FLOWCTRL

#ifndef UART11_RTS_PIN
#warning "[Deprecated] UART11_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART11_RTS_PIN UART11_RTS_PIN_DEFAULT
#endif

#ifndef UART11_CTS_PIN
#warning "[Deprecated] UART11_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART11_CTS_PIN UART11_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART11_HW_FLOWCTRL */

#ifndef UART11_BUFSIZE
#define UART11_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART11_BUFSIZE)
#error "UART11_BUFSIZE must not be empty"
#endif

#ifndef UART11_BAUDRATE
#warning "[Deprecated] UART11_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART11_BAUDRATE 115200
#endif

#else
#define UART11_BAUDRATE 0
#define UART11_BUFSIZE  0
#endif /* #if UART11 */


#ifndef UART12
#define UART12 0
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

#if UART12_HW_FLOWCTRL

#ifndef UART12_RTS_PIN
#warning "[Deprecated] UART12_RTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART12_RTS_PIN UART12_RTS_PIN_DEFAULT
#endif

#ifndef UART12_CTS_PIN
#warning "[Deprecated] UART12_CTS_PIN should be explicitly defined in board_config.h. Using the default"
#define UART12_CTS_PIN UART12_CTS_PIN_DEFAULT
#endif

#endif /* #ifdef UART12_HW_FLOWCTRL */

#ifndef UART12_BUFSIZE
#define UART12_BUFSIZE UART_BUFSIZE
#endif

#if ISEMPTY(UART12_BUFSIZE)
#error "UART12_BUFSIZE must not be empty"
#endif

#ifndef UART12_BAUDRATE
#warning "[Deprecated] UART12_BAUDRATE should be explicitly defined in board_config.h. Using the default"
#define UART12_BAUDRATE 115200
#endif

#else
#define UART12_BAUDRATE 0
#define UART12_BUFSIZE  0
#endif /* #if UART12 */


#else

#define UART9           0
#define UART9_BUFSIZE   0
#define UART9_BAUDRATE  0
#define UART10          0
#define UART10_BUFSIZE  0
#define UART10_BAUDRATE 0
#define UART11          0
#define UART11_BUFSIZE  0
#define UART11_BAUDRATE 0
#define UART12          0
#define UART12_BUFSIZE  0
#define UART12_BAUDRATE 0

#endif /* #ifdef __CPU_IMXRT117X */

/* clang-format off */

#define UART_BAUDRATES \
	UART1_BAUDRATE, \
	UART2_BAUDRATE, \
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

#define UART_BUFSIZES \
	UART1_BUFSIZE, \
	UART2_BUFSIZE, \
	UART3_BUFSIZE, \
	UART4_BUFSIZE, \
	UART5_BUFSIZE, \
	UART6_BUFSIZE, \
	UART7_BUFSIZE, \
	UART8_BUFSIZE, \
	UART9_BUFSIZE, \
	UART10_BUFSIZE, \
	UART11_BUFSIZE, \
	UART12_BUFSIZE

/* clang-format on */

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

#if !ISEMPTY(UART_CONSOLE)
#if UART_CONSOLE <= 0
#error "Invalid value for UART_CONSOLE"
#endif
#endif

#if defined(UART_CONSOLE_USER)
#if !ISEMPTY(UART_CONSOLE_USER)
#if (UART_CONSOLE_USER <= 0)
#error "Invalid value for UART_CONSOLE_USER"
#endif
#endif
#else
#define UART_CONSOLE_USER UART_CONSOLE
#endif


/* RTT */

#ifndef RTT_CHANNEL0
#define RTT_CHANNEL0 0
#elif !ISBOOLEAN(RTT_CHANNEL0)
#error "RTT_CHANNEL0 must have a value of 0, 1, or be undefined"
#endif

#ifndef RTT_CHANNEL1
#define RTT_CHANNEL1 0
#elif !ISBOOLEAN(RTT_CHANNEL1)
#error "RTT_CHANNEL1 must have a value of 0, 1, or be undefined"
#endif

#ifndef RTT_CHANNEL0_BLOCKING
#define RTT_CHANNEL0_BLOCKING 0
#elif !ISBOOLEAN(RTT_CHANNEL0_BLOCKING)
#error "RTT_CHANNEL0_BLOCKING must have a value of 0, 1, or be undefined"
#endif

#ifndef RTT_CHANNEL1_BLOCKING
#define RTT_CHANNEL1_BLOCKING 0
#elif !ISBOOLEAN(RTT_CHANNEL1_BLOCKING)
#error "RTT_CHANNEL1_BLOCKING must have a value of 0, 1, or be undefined"
#endif


#ifndef RTT_CHANNEL_CONSOLE
#define RTT_CHANNEL_CONSOLE
#elif !ISEMPTY(RTT_CHANNEL_CONSOLE)
#if RTT_CHANNEL_CONSOLE < 0
#error "Invalid value for RTT_CHANNEL_CONSOLE"
#endif
#endif

#if !ISEMPTY(UART_CONSOLE_USER) && !ISEMPTY(RTT_CHANNEL_CONSOLE)
#error "Console on both UART and RTT not supported"
#elif ISEMPTY(UART_CONSOLE_USER) && ISEMPTY(RTT_CHANNEL_CONSOLE)
#error "Console must be either on UART or RTT"
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

#ifdef SPI1_SD0
#warning "[Deprecated] SPI1_SDO (ending with letter O) should be used instead of SPI1_SD0 (ending with zero)"
#define SPI1_SDO SPI1_SD0
#endif

#ifndef SPI1_SDO
#warning "[Deprecated] SPI1_SDO should be explicitly defined in board_config.h. Using the default"
#define SPI1_SDO SPI1_SDO_DEFAULT
#elif ISEMPTY(SPI1_SDO)
#error "SPI1_SDO must not be empty"
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

#endif /* #if SPI1 */


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

#ifdef SPI2_SD0
#warning "[Deprecated] SPI2_SDO (ending with letter O) should be used instead of SPI2_SD0 (ending with zero)"
#define SPI2_SDO SPI2_SD0
#endif

#ifndef SPI2_SDO
#warning "[Deprecated] SPI2_SDO should be explicitly defined in board_config.h. Using the default"
#define SPI2_SDO SPI2_SDO_DEFAULT
#elif ISEMPTY(SPI2_SDO)
#error "SPI2_SDO must not be empty"
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

#endif /* #if SPI2 */


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

#ifdef SPI3_SD0
#warning "[Deprecated] SPI3_SDO (ending with letter O) should be used instead of SPI3_SD0 (ending with zero)"
#define SPI3_SDO SPI3_SD0
#endif

#ifndef SPI3_SDO
#warning "[Deprecated] SPI3_SDO should be explicitly defined in board_config.h. Using the default"
#define SPI3_SDO SPI3_SDO_DEFAULT
#elif ISEMPTY(SPI3_SDO)
#error "SPI3_SDO must not be empty"
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

#endif /* #if SPI3 */


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

#ifdef SPI4_SD0
#warning "[Deprecated] SPI4_SDO (ending with letter O) should be used instead of SPI4_SD0 (ending with zero)"
#define SPI4_SDO SPI4_SD0
#endif

#ifndef SPI4_SDO
#warning "[Deprecated] SPI4_SDO should be explicitly defined in board_config.h. Using the default"
#define SPI4_SDO SPI4_SDO_DEFAULT
#elif ISEMPTY(SPI4_SDO)
#error "SPI4_SDO must not be empty"
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

#endif /* #if SPI4 */


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

#ifdef SPI5_SD0
#warning "[Deprecated] SPI5_SDO (ending with letter O) should be used instead of SPI5_SD0 (ending with zero)"
#define SPI5_SDO SPI5_SD0
#endif

#ifndef SPI5_SDO
#warning "[Deprecated] SPI5_SDO should be explicitly defined in board_config.h. Using the default"
#define SPI5_SDO SPI5_SDO_DEFAULT
#elif ISEMPTY(SPI5_SDO)
#error "SPI5_SDO must not be empty"
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

#endif /* #if SPI5 */


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

#ifdef SPI6_SD0
#warning "[Deprecated] SPI6_SDO (ending with letter O) should be used instead of SPI6_SD0 (ending with zero)"
#define SPI6_SDO SPI6_SD0
#endif

#ifndef SPI6_SDO
#warning "[Deprecated] SPI6_SDO should be explicitly defined in board_config.h. Using the default"
#define SPI6_SDO SPI6_SDO_DEFAULT
#elif ISEMPTY(SPI6_SDO)
#error "SPI6_SDO must not be empty"
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

#endif /* #if SPI6 */


#else

#define SPI5 0
#define SPI6 0

#endif /* #ifdef __CPU_IMXRT117X */


/* I2C */

#ifndef I2C1
#define I2C1 0
#elif !ISBOOLEAN(I2C1)
#error "I2C1 must have a value of 0, 1, or be undefined"
#endif

#ifndef I2C2
#define I2C2 0
#elif !ISBOOLEAN(I2C2)
#error "I2C2 must have a value of 0, 1, or be undefined"
#endif

#ifndef I2C3
#define I2C3 0
#elif !ISBOOLEAN(I2C3)
#error "I2C3 must have a value of 0, 1, or be undefined"
#endif

#ifndef I2C4
#define I2C4 0
#elif !ISBOOLEAN(I2C4)
#error "I2C4 must have a value of 0, 1, or be undefined"
#endif

#ifdef __CPU_IMXRT117X

#ifndef I2C5
#define I2C5 0
#elif !ISBOOLEAN(I2C5)
#error "I2C5 must have a value of 0, 1, or be undefined"
#endif

#ifndef I2C6
#define I2C6 0
#elif !ISBOOLEAN(I2C6)
#error "I2C6 must have a value of 0, 1, or be undefined"
#endif

#endif /* #ifdef __CPU_IMXRT117X */

#ifndef I2C1_PUSHPULL
#define I2C1_PUSHPULL 0
#endif

#ifndef I2C2_PUSHPULL
#define I2C2_PUSHPULL 0
#endif

#ifndef I2C3_PUSHPULL
#define I2C3_PUSHPULL 0
#endif

#ifndef I2C4_PUSHPULL
#define I2C4_PUSHPULL 0
#endif

#ifndef I2C5_PUSHPULL
#define I2C5_PUSHPULL 0
#endif

#ifndef I2C6_PUSHPULL
#define I2C6_PUSHPULL 0
#endif

#ifndef I2C1_SPEED
#define I2C1_SPEED i2c_speed_slow
#endif

#ifndef I2C2_SPEED
#define I2C2_SPEED i2c_speed_slow
#endif

#ifndef I2C3_SPEED
#define I2C3_SPEED i2c_speed_slow
#endif

#ifndef I2C4_SPEED
#define I2C4_SPEED i2c_speed_slow
#endif

#ifndef I2C5_SPEED
#define I2C5_SPEED i2c_speed_slow
#endif

#ifndef I2C6_SPEED
#define I2C6_SPEED i2c_speed_slow
#endif


/* TRNG */

#ifndef __CPU_IMXRT117X

#ifndef TRNG
#define TRNG 0
#endif

#endif /* #ifndef __CPU_IMXRT117X */

/* Temperature */

#ifndef PCT2075
#define PCT2075 0
#elif !ISBOOLEAN(PCT2075)
#error "PCT2075 must have a value of 0, 1, or be undefined"
#elif !defined(PCT2075_BUS_NUM)
#error "PCT2075_BUS_NUM must be defined"
#elif !defined(PCT2075_DEV_ADDR)
#error "PCT2075_DEV_ADDR must be defined"
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
