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


typedef enum {
	mcp331_cmd__recalibrate
} mcp331_cmd_t;


typedef struct {
	mcp331_cmd_t cmd;
} mcp331_i_devctl_t;


#endif /* MCP331_API_H */
