/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Nandtool tests
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _NANDTOOL_TEST_H_
#define _NANDTOOL_TEST_H_

typedef int (*test_func_t)(void *arg);

test_func_t test_func[16];
extern int test_cnt;

#ifndef CONFIG_NANDTOOL_TEST

#define test_enabled 0
#define init_tests()

#else

#define test_enabled 1
void init_tests(void);

#endif /* CONFIG_NANDTOOL_TESTS */

#endif /* NANDTOOL_TEST_H */
