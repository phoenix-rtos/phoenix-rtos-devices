/*
 * Phoenix-RTOS
 *
 * MCP33131/21/11-XX components API
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MCP331_LOW_H
#define MCP331_LOW_H

#include <stdint.h>

int spi_exchange(int devNo, int chan, uint8_t *buff, uint32_t len);


int spi_init(int devNo, int chan);


#endif
