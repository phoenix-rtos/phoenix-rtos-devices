/*
 * Phoenix-RTOS
 *
 * i.MX RT utilities
 *
 * Copyright 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>

#define COL_GREY    "\033[1;30m"
#define COL_RED     "\033[1;31m"
#define COL_GREEN   "\033[1;32m"
#define COL_YELLOW  "\033[1;33m"
#define COL_BLUE    "\033[1;34m"
#define COL_CYAN    "\033[1;36m"
#define COL_VIOLET  "\033[1;35m"
#define COL_NORMAL  "\033[0m"

#define LOG_CTG_COLOR       COL_YELLOW
#define LOG_INFO_COLOR      COL_NORMAL
#define LOG_ERR_COLOR       COL_RED
#define LOG_SUCCESS_COLOR   COL_GREEN


static int testCounter = 1;
static int categoryCounter = 0;

#define TEST_CATEGORY(category) \
	do { \
		printf(LOG_CTG_COLOR); \
		printf("\n\n##        %d. %s :", ++categoryCounter, category); \
		testCounter = 1; \
		printf(LOG_INFO_COLOR); \
	} while(0)


#define TEST_CASE(test) \
	do { \
		if ((test) == EOK) { \
			printf("\nTEST CASE %d.%d: %-60s\t", categoryCounter, testCounter++, #test); \
			printf(LOG_SUCCESS_COLOR); \
			printf("-- PASSED"); } \
		else { \
			printf(LOG_ERR_COLOR); \
			printf("\nTEST CASE %d.%d: %-60s\t", categoryCounter, testCounter++, #test); \
			printf("-- FAILED"); \
		} \
		printf(LOG_INFO_COLOR); \
		fflush(stdout); \
	} while (0)
