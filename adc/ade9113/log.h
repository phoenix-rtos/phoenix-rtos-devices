/*
 * Phoenix-RTOS
 *
 * Minimal log macros
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef ADE9113_LOG_H
#define ADE9113_LOG_H

#include <stdio.h>

#ifndef LOG_TAG
#error LOG_TAG is not defined
#endif

#define log_error(fmt, ...) \
	do { \
		fprintf(stdout, LOG_TAG fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define log_info(fmt, ...) log_error(fmt, ##__VA_ARGS__);

#endif /* end of include guard: ADE9113_LOG_H */
