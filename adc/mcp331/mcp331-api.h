/*
 * Phoenix-RTOS
 *
 * MCP33131/21/11-XX public API
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MCP331_API_H
#define MCP331_API_H

#include <stdint.h>
#include <stddef.h>

#define MCP331_PERIODIC_BUFFER_SIZE 8
// #define MCP331_PERIODIC_BUFFER_SIZE 256


typedef enum {
	mcp331_cmd__recalibrate,
	mcp331_cmd__startPeriodic, /* Start periodic sampling */
	mcp331_cmd__getSamples     /* Get collected samples */
} mcp331_cmd_t;


typedef struct {
	mcp331_cmd_t cmd;
	union {
		struct {
			uint16_t waitStates; /* Number of SPI clock wait states between transactions */
		} startPeriodic;
		struct {
			size_t maxSamples; /* Maximum number of samples to retrieve */
		} getSamples;
	};
} mcp331_i_devctl_t;


#endif /* MCP331_API_H */
