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
#define TTY4 1
#endif

#ifndef TTY5
#define TTY5 0
#endif

#if (UART1 && TTY1) || (UART2 && TTY2) || (UART3 && TTY3) || \
	(UART4 && TTY4) || (UART5 && TTY5)
#error "Can't use UART as UART and TTY at the same time!"
#endif

#ifndef UART_CONSOLE
#define UART_CONSOLE 4
#endif

#ifndef SPI1
#define SPI1 1
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
#define SPI2_USEDMA 1
#endif

#ifndef SPI3_USEDMA
#define SPI3_USEDMA 0
#endif

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

#define CONSOLE_IS_TTY ((UART_CONSOLE == 1 && TTY1) || (UART_CONSOLE == 2 && TTY2) || (UART_CONSOLE == 3 && TTY3) || (UART_CONSOLE == 4 && TTY4) || (UART_CONSOLE == 5 && TTY5))

#ifndef BUILTIN_DUMMYFS
#define BUILTIN_DUMMYFS 1
#endif

#endif
