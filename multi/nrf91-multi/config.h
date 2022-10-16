/*
 * Phoenix-RTOS
 *
 * NRF91 multi driver config.
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau
 *
 * %LICENSE%
 */


#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifndef UART0
#define UART0 0
#endif

#ifndef UART1
#define UART1 0
#endif

#ifndef UART2
#define UART2 0
#endif

#ifndef UART3
#define UART3 0
#endif

#ifndef TTY0
#define TTY0 1
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

#if (UART0 && TTY0) || (UART1 && TTY1) || (UART2 && TTY2) || (UART3 && TTY3)
#error "Can't use UART as UART and TTY at the same time!"
#endif

#ifndef UART_CONSOLE
#define UART_CONSOLE 0
#endif

#define CONSOLE_IS_TTY ((UART_CONSOLE == 0 && TTY0) || (UART_CONSOLE == 1 && TTY1) || (UART_CONSOLE == 2 && TTY2) || (UART_CONSOLE == 3 && TTY3))

#ifndef BUILTIN_DUMMYFS
#define BUILTIN_DUMMYFS 1
#endif

/* sizes of uart dma memory regions - 1 because of max value of maxcnt */
#define UART_TX_DMA_SIZE 8191
#define UART_RX_DMA_SIZE 8191

/* default uart instance for nrf9160 dk, connected to VCOM0 */
#define UART0_TX 29
#define UART0_RX 28
#define UART0_RTS 27
#define UART0_CTS 26

/* second uart interface on nrf9160 dk called nRF91_UART_2 on the board's schematic*/
#define UART1_TX 1
#define UART1_RX 0
#define UART1_RTS 14
#define UART1_CTS 15

#define UART2_TX UART0_TX
#define UART2_RX UART0_RX
#define UART2_RTS UART0_RTS
#define UART2_CTS UART0_CTS

#define UART3_TX UART1_TX
#define UART3_RX UART1_RX
#define UART3_RTS UART1_RTS
#define UART3_CTS UART1_CTS

#endif
