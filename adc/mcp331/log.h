/*
 * Phoenix-RTOS
 *
 * logging functions
 *
 * Copyright 2025 Phoenix Systems
 * Author: Adam Greloch
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MCP331_LOG_H
#define MCP331_LOG_H

#include <stdio.h>

/* Logging */
#define COL_RED    "\033[1;31m"
#define COL_CYAN   "\033[1;36m"
#define COL_NORMAL "\033[0m"

#define LOG_TAG "mcp331: "

/* clang-format off */
#ifdef NDEBUG
#define log_debug(fmt, ...)
#else
#define log_debug(fmt, ...) do { printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#endif

#define log_info(fmt, ...) do { printf(LOG_TAG COL_CYAN fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...) do { printf(LOG_TAG COL_RED fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)
/*clang-format on */

#endif
