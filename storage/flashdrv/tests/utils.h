/*
 * Phoenix-RTOS
 *
 * GR716 test utilities
 *
 * Copyright 2020, 2023 Phoenix Systems
 * Author: Hubert Buczynski, Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GR716_FLASH_TEST_UTILS_H_
#define _GR716_FLASH_TEST_UTILS_H_

#include <stdio.h>

static int testCounter = 1;
static int categoryCounter = 0;

#define TEST_CATEGORY(category) \
	do { \
		printf("\n\n##        %d. %s :", ++categoryCounter, category); \
		testCounter = 1; \
	} while (0)


#define TEST_CASE(test) \
	do { \
		printf("\nTEST CASE %d.%d: %-20s\t", categoryCounter, testCounter++, #test); \
		if ((test) == EOK) { \
			printf("-- PASSED"); \
		} \
		else { \
			printf("-- FAILED"); \
		} \
		printf("\n"); \
		fflush(stdout); \
	} while (0)


#endif
