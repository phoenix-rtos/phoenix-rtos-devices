/*
 * Phoenix-RTOS
 *
 * STM32L4 UART driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski, Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "lib/libuart.h"

#include "stm32-multi.h"
#include "config.h"
#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "rcc.h"

#define UART1_POS 0
#define UART2_POS (UART1_POS + UART1)
#define UART3_POS (UART2_POS + UART2)
#define UART4_POS (UART3_POS + UART3)
#define UART5_POS (UART4_POS + UART4)

#define UART_CNT (UART1 + UART2 + UART3 + UART4 + UART5)

static libuart_ctx uart_common[UART_CNT];


static const int uartConfig[] = { UART1, UART2, UART3, UART4, UART5 };


static const int uartPos[] = { UART1_POS, UART2_POS, UART3_POS, UART4_POS, UART5_POS };


int uart_configure(int uart, char bits, char parity, unsigned int baud, char enable)
{
	if (uart < usart1 || uart > uart5 || !uartConfig[uart])
		return -EINVAL;

	return libuart_configure(&uart_common[uartPos[uart]], bits, parity, baud, enable);
}


int uart_write(int uart, void* buff, unsigned int bufflen)
{
	if (uart < usart1 || uart > uart5 || !uartConfig[uart])
		return -EINVAL;

	return libuart_write(&uart_common[uartPos[uart]], buff, bufflen);
}


int uart_read(int uart, void* buff, unsigned int count, char mode, unsigned int timeout)
{
	if (uart < usart1 || uart > uart5 || !uartConfig[uart])
		return -EINVAL;

	return libuart_read(&uart_common[uartPos[uart]], buff, count, mode, timeout);
}


int uart_init(void)
{
	int i;
	unsigned int uart;

	for (i = 0, uart = usart1; uart <= uart5; ++uart) {
		if (!uartConfig[uart])
			continue;

		libuart_init(&uart_common[i], uart);

		++i;
	}

	return EOK;
}
