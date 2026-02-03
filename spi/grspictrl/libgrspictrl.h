/*
 * Phoenix-RTOS
 *
 * GRLIB SPI CTRL driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBGRSPICTRL_H_
#define LIBGRSPICTRL_H_


#include <stdbool.h>

#include <sys/types.h>


/* SPI Pins setup */

#ifndef SPI0_SCK
#define SPI0_SCK -1
#endif

#ifndef SPI0_MOSI
#define SPI0_MOSI -1
#endif

#ifndef SPI0_MISO
#define SPI0_MISO -1
#endif

#ifndef SPI0_CS
#define SPI0_CS -1
#endif


struct spi_ctx {
	struct spictrl_regs *regs;
	int irq;
	uint8_t bitOrder;
	uint8_t fifosz;

	handle_t mutex;
	handle_t irqLock;
	handle_t cond;
	handle_t inth;

	uint32_t ssCount;
};


#endif
