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

#include "libmulti/libuart.h"

#include "stm32l4-multi.h"
#include "common.h"
#include "config.h"
#include "gpio.h"
#include "uart.h"
#include "rcc.h"

#define MAX_UART uart5

#define UART1_POS 0
#define UART2_POS (UART1_POS + UART1)
#define UART3_POS (UART2_POS + UART2)
#define UART4_POS (UART3_POS + UART3)
#define UART5_POS (UART4_POS + UART4)

#define N_ACTIVE_UART (UART1 + UART2 + UART3 + UART4 + UART5)


static libuart_ctx uart_common[N_ACTIVE_UART];


static const int uartEnabled[MAX_UART + 1] = { UART1, UART2, UART3, UART4, UART5 };


static const int uartDMA[MAX_UART + 1] = { UART1_DMA, UART2_DMA, UART3_DMA, UART4_DMA, UART5_DMA };


static const int uartPos[MAX_UART + 1] = { UART1_POS, UART2_POS, UART3_POS, UART4_POS, UART5_POS };


int uart_configure(int uart, char bits, char parity, unsigned int baud, char enable)
{
	if ((N_ACTIVE_UART == 0) || (uart < usart1) || (uart > MAX_UART) || (uartEnabled[uart] == 0)) {
		return -EINVAL;
	}

	return libuart_configure(&uart_common[uartPos[uart]], bits, parity, baud, enable);
}


int uart_write(int uart, const void *buff, unsigned int bufflen)
{
	if ((N_ACTIVE_UART == 0) || (uart < usart1) || (uart > MAX_UART) || (uartEnabled[uart] == 0)) {
		return -EINVAL;
	}

	return libuart_write(&uart_common[uartPos[uart]], buff, bufflen);
}


int uart_read(int uart, void *buff, unsigned int count, char mode, unsigned int timeout)
{
	if ((N_ACTIVE_UART == 0) || (uart < usart1) || (uart > MAX_UART) || (uartEnabled[uart] == 0)) {
		return -EINVAL;
	}

	return libuart_read(&uart_common[uartPos[uart]], buff, count, mode, timeout);
}


int uart_init(void)
{
	if (N_ACTIVE_UART == 0) {
		return EOK;
	}

	for (int uart = usart1; uart <= MAX_UART; ++uart) {
		if (uartEnabled[uart] == 0) {
			continue;
		}

		libuart_init(&uart_common[uartPos[uart]], uart, uartDMA[uart]);
	}

	return EOK;
}
