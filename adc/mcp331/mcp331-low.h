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
#include <sys/threads.h>

/* Callback function type for periodic sampling - called from interrupt context */
typedef int (*spi_periodicCallback_t)(const uint8_t *data, size_t len);


int spi_exchange(int devNo, int chan, uint8_t *buff, uint32_t len);


int spi_init(int devNo, int chan);


/* Initialize periodic sampling support - must be called after spi_init */
int spi_initPeriodic(int devNo, handle_t cond, spi_periodicCallback_t callback);


/* Start periodic sampling with specified wait states between transactions */
int spi_startPeriodic(int devNo, int chan, uint16_t waitStates);

#endif
