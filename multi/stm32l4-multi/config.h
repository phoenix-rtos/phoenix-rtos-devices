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

#if (UART1_DMA && !UART1) || (UART2_DMA && !UART2) || (UART3_DMA && !UART3) || \
	(UART4_DMA && !UART4) || (UART5_DMA && !UART5)
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

/* UART_CONSOLE has to be specified in board_config.h */

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

#ifndef SPI1_USEDMA
#define SPI1_USEDMA 0
#endif

#ifndef SPI2_USEDMA
#define SPI2_USEDMA 0
#endif

#ifndef SPI3_USEDMA
#define SPI3_USEDMA 0
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

#endif
