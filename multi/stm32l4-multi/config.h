/*
 * Phoenix-RTOS
 *
 * STM32L4 multi driver config.
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * %LICENSE%
 */


#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <board_config.h>

/* True if X is empty (has no value). The result in #if is valid only if defined(X) is true */
#define ISEMPTY(X) ((0 - X - 1) == 1 && (X + 0) != -2)

#ifndef MULTIDRV_INTERFACE_THREADS
#define MULTIDRV_INTERFACE_THREADS 3
#endif

/* UART */
#ifndef UART1
#define UART1 0
#endif

#ifndef UART2
#define UART2 0
#endif

#ifndef UART3
#define UART3 0
#endif

#ifndef UART4
#define UART4 0
#endif

#ifndef UART5
#define UART5 0
#endif

#ifndef UART6
#define UART6 0
#endif

#ifndef UART7
#define UART7 0
#endif

#ifndef UART8
#define UART8 0
#endif

#ifndef UART9
#define UART9 0
#endif

#ifndef UART10
#define UART10 0
#endif

#ifndef UART1_DMA
#define UART1_DMA 0
#endif

#ifndef UART2_DMA
#define UART2_DMA 0
#endif

#ifndef UART3_DMA
#define UART3_DMA 0
#endif

#ifndef UART4_DMA
#define UART4_DMA 0
#endif

#ifndef UART5_DMA
#define UART5_DMA 0
#endif

#ifndef UART6_DMA
#define UART6_DMA 0
#endif

#ifndef UART7_DMA
#define UART7_DMA 0
#endif

#ifndef UART8_DMA
#define UART8_DMA 0
#endif

#ifndef UART9_DMA
#define UART9_DMA 0
#endif

#ifndef UART10_DMA
#define UART10_DMA 0
#endif

#if (UART1_DMA && !UART1) || (UART2_DMA && !UART2) || (UART3_DMA && !UART3) || \
		(UART4_DMA && !UART4) || (UART5_DMA && !UART5) || (UART6_DMA && !UART6) || \
		(UART7_DMA && !UART7) || (UART8_DMA && !UART8) || (UART9_DMA && !UART9) || \
		(UART10_DMA && !UART10)
#error "DMA mode cannot be enabled on a disabled UART!"
#endif

#ifndef TTY1
#define TTY1 0
#endif

#ifndef TTY2
#define TTY2 0
#endif

#ifndef TTY3
#define TTY3 0
#endif

#ifndef TTY4
#define TTY4 0
#endif

#ifndef TTY5
#define TTY5 0
#endif

#ifndef TTY6
#define TTY6 0
#endif

#ifndef TTY7
#define TTY7 0
#endif

#ifndef TTY8
#define TTY8 0
#endif

#ifndef TTY9
#define TTY9 0
#endif

#ifndef TTY10
#define TTY10 0
#endif

#if defined(UART_CONSOLE)
#if !ISEMPTY(UART_CONSOLE)
#if UART_CONSOLE <= 0
#error "Invalid value for UART_CONSOLE"
#endif
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


#ifndef TTY1_DMA
#define TTY1_DMA 0
#endif

#ifndef TTY2_DMA
#define TTY2_DMA 0
#endif

#ifndef TTY3_DMA
#define TTY3_DMA 0
#endif

#ifndef TTY4_DMA
#define TTY4_DMA 0
#endif

#ifndef TTY5_DMA
#define TTY5_DMA 0
#endif

#ifndef TTY6_DMA
#define TTY6_DMA 0
#endif

#ifndef TTY7_DMA
#define TTY7_DMA 0
#endif

#ifndef TTY8_DMA
#define TTY8_DMA 0
#endif

#ifndef TTY9_DMA
#define TTY9_DMA 0
#endif

#ifndef TTY10_DMA
#define TTY10_DMA 0
#endif

#ifndef TTY1_DMA_RXSZ
#define TTY1_DMA_RXSZ 32
#endif

#ifndef TTY2_DMA_RXSZ
#define TTY2_DMA_RXSZ 32
#endif

#ifndef TTY3_DMA_RXSZ
#define TTY3_DMA_RXSZ 32
#endif

#ifndef TTY4_DMA_RXSZ
#define TTY4_DMA_RXSZ 32
#endif

#ifndef TTY5_DMA_RXSZ
#define TTY5_DMA_RXSZ 32
#endif

#ifndef TTY6_DMA_RXSZ
#define TTY6_DMA_RXSZ 32
#endif

#ifndef TTY7_DMA_RXSZ
#define TTY7_DMA_RXSZ 32
#endif

#ifndef TTY8_DMA_RXSZ
#define TTY8_DMA_RXSZ 32
#endif

#ifndef TTY9_DMA_RXSZ
#define TTY9_DMA_RXSZ 32
#endif

#ifndef TTY10_DMA_RXSZ
#define TTY10_DMA_RXSZ 32
#endif

#ifndef TTY1_DMA_RXFIFOSZ
#define TTY1_DMA_RXFIFOSZ 64
#endif

#ifndef TTY2_DMA_RXFIFOSZ
#define TTY2_DMA_RXFIFOSZ 64
#endif

#ifndef TTY3_DMA_RXFIFOSZ
#define TTY3_DMA_RXFIFOSZ 64
#endif

#ifndef TTY4_DMA_RXFIFOSZ
#define TTY4_DMA_RXFIFOSZ 64
#endif

#ifndef TTY5_DMA_RXFIFOSZ
#define TTY5_DMA_RXFIFOSZ 64
#endif

#ifndef TTY6_DMA_RXFIFOSZ
#define TTY6_DMA_RXFIFOSZ 64
#endif

#ifndef TTY7_DMA_RXFIFOSZ
#define TTY7_DMA_RXFIFOSZ 64
#endif

#ifndef TTY8_DMA_RXFIFOSZ
#define TTY8_DMA_RXFIFOSZ 64
#endif

#ifndef TTY9_DMA_RXFIFOSZ
#define TTY9_DMA_RXFIFOSZ 64
#endif

#ifndef TTY10_DMA_RXFIFOSZ
#define TTY10_DMA_RXFIFOSZ 64
#endif

#ifndef TTY1_DMA_TXSZ
#define TTY1_DMA_TXSZ 64
#endif

#ifndef TTY2_DMA_TXSZ
#define TTY2_DMA_TXSZ 64
#endif

#ifndef TTY3_DMA_TXSZ
#define TTY3_DMA_TXSZ 64
#endif

#ifndef TTY4_DMA_TXSZ
#define TTY4_DMA_TXSZ 64
#endif

#ifndef TTY5_DMA_TXSZ
#define TTY5_DMA_TXSZ 64
#endif

#ifndef TTY6_DMA_TXSZ
#define TTY6_DMA_TXSZ 64
#endif

#ifndef TTY7_DMA_TXSZ
#define TTY7_DMA_TXSZ 64
#endif

#ifndef TTY8_DMA_TXSZ
#define TTY8_DMA_TXSZ 64
#endif

#ifndef TTY9_DMA_TXSZ
#define TTY9_DMA_TXSZ 64
#endif

#ifndef TTY10_DMA_TXSZ
#define TTY10_DMA_TXSZ 64
#endif

#ifndef TTY1_LIBTTY_BUFSZ
#define TTY1_LIBTTY_BUFSZ 512
#endif

#ifndef TTY2_LIBTTY_BUFSZ
#define TTY2_LIBTTY_BUFSZ 512
#endif

#ifndef TTY3_LIBTTY_BUFSZ
#define TTY3_LIBTTY_BUFSZ 512
#endif

#ifndef TTY4_LIBTTY_BUFSZ
#define TTY4_LIBTTY_BUFSZ 512
#endif

#ifndef TTY5_LIBTTY_BUFSZ
#define TTY5_LIBTTY_BUFSZ 512
#endif

#ifndef TTY6_LIBTTY_BUFSZ
#define TTY6_LIBTTY_BUFSZ 512
#endif

#ifndef TTY7_LIBTTY_BUFSZ
#define TTY7_LIBTTY_BUFSZ 512
#endif

#ifndef TTY8_LIBTTY_BUFSZ
#define TTY8_LIBTTY_BUFSZ 512
#endif

#ifndef TTY9_LIBTTY_BUFSZ
#define TTY9_LIBTTY_BUFSZ 512
#endif

#ifndef TTY10_LIBTTY_BUFSZ
#define TTY10_LIBTTY_BUFSZ 512
#endif

#ifndef UART1_RXFIFOSZ
#define UART1_RXFIFOSZ 64
#endif

#ifndef UART2_RXFIFOSZ
#define UART2_RXFIFOSZ 64
#endif

#ifndef UART3_RXFIFOSZ
#define UART3_RXFIFOSZ 64
#endif

#ifndef UART4_RXFIFOSZ
#define UART4_RXFIFOSZ 64
#endif

#ifndef UART5_RXFIFOSZ
#define UART5_RXFIFOSZ 64
#endif

#ifndef UART6_RXFIFOSZ
#define UART6_RXFIFOSZ 64
#endif

#ifndef UART7_RXFIFOSZ
#define UART7_RXFIFOSZ 64
#endif

#ifndef UART8_RXFIFOSZ
#define UART8_RXFIFOSZ 64
#endif

#ifndef UART9_RXFIFOSZ
#define UART9_RXFIFOSZ 64
#endif

#ifndef UART10_RXFIFOSZ
#define UART10_RXFIFOSZ 64
#endif


/* SPI */
#ifndef SPI1
#define SPI1 0
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

#ifndef SPI1_USEDMA
#define SPI1_USEDMA 0
#endif

#ifndef SPI2_USEDMA
#define SPI2_USEDMA 0
#endif

#ifndef SPI3_USEDMA
#define SPI3_USEDMA 0
#endif

#ifndef SPI4_USEDMA
#define SPI4_USEDMA 0
#endif

#ifndef SPI5_USEDMA
#define SPI5_USEDMA 0
#endif

#ifndef SPI6_USEDMA
#define SPI6_USEDMA 0
#endif


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


/* dummyfs */
#ifndef BUILTIN_DUMMYFS
#define BUILTIN_DUMMYFS 1
#endif

/* libposixsrv (disabled by default) */
#ifndef BUILTIN_POSIXSRV
#define BUILTIN_POSIXSRV 0
#endif

/* libcoredumpsrv */
#ifndef BUILTIN_COREDUMPSRV
#define BUILTIN_COREDUMPSRV 1
#endif


#endif
