/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 multi driver main
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include ARCH
										#define NULL 0
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <sys/msg.h>

#include "adc.h"
#include "common.h"
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "rcc.h"
#include "rtc.h"
#include "uart.h"

#ifdef NDEBUG
#define DEBUG(format, ...)
#else
#define DEBUG(format, ...) printf("%s:"format, drvname, __VA_ARGS__)
#endif

#define THREADS_MIN 3
#define THREADS_MAX 7
#define THREADS_PRIORITY 2
#define STACKSZ 512


typedef struct _thread_t {
	struct _thread_t *next;
	void *stack;
} thread_t;


struct {
	thread_t *pool_root;
	thread_t *pool_garbage;
	int pool_size;
	int pool_busy;
	handle_t pool_lock;

	unsigned int port;
} common;


static const char drvname[] = "multidriver";


int spawnThread(void)
{
	thread_t *t, *list;

	if ((stack = malloc(STACKSZ)) == NULL) {
		DEBUG("Failed to alocate thread (pool size %d)\n", common.pool_size);
		return -ENOMEM;
	}

	/* Information about thread is kept just above thread's stack */
	t = (thread_t *)(stack + STACKSZ - sizeof(thread_t));
	t->next = NULL;
	t->stack = stack;

	mutexLock(common.pool_lock);

	if (common.pool_root == NULL) {
		common.pool_root = t;
	}
	else {
		list = common.pool_root;
		while (list->next != NULL)
			list = list->next;

		list->next = t;
	}

	++common.pool_size;

	mutexUnlock(common.pool_lock);

	beginthread(thread, THREADS_PRIORITY, stack, (STACKSZ - sizeof(thread_t) - 8) & ~7, t);

	return EOK;
}


int destroyThread(thread_t *t)
{
	thread_t *prev;

	mutexLock(common.pool_lock);

	if (t == common.pool_root) {
		common.pool_root = t->next;
	}
	else {
		prev = common.pool_root;
		while (prev->next != NULL && prev->next != t)
			prev = prev->next;

		if (prev == NULL) {
			mutexUnlock(common.pool_lock);
			return -ENOENT;
		}

		prev->next = t->next;
	}

	t->next = common.pool_garbage;
	common.pool_garbage = t;

	mutexUnlock(common.pool_lock);

	/* FIXME - racing with garbage collector */
	endthread();
}


void garbageCollector(void)
{
	thread_t *t;

	mutexLock(common.pool_lock);

	if (common.pool_garbage == NULL) {
		mutexUnlock(common.pool_lock);
		return;
	}

	t = common.pool_garbage;
	common.pool_garbage = t->next;

	mutexUnlock(common.pool_lock);

	free(t->stack);
}


void thread(void *arg)
{
	msg_t msg;
	unsigned int rid;

	while (1) {
		while (msgRecv(common.port, &msg, &rid) != EOK)
			;
/*
		mutexLock(common.pool_lock);

		++common.pool_busy;

		if (common.pool_busy == common.pool_size) {
			mutexUnlock(common.pool_lock);

			spawnThread();
		}
		else {
			mutexUnlock(common.pool_lock);
		}
*/


	}
}


int main(void)
{
	int i;

	common.pool_root = NULL;
	common.pool_size = 0;
	common.pool_busy = 0;

	if (portCreate(&common.port) != EOK) {
		DEBUG("Failed to port\n");
		return -1;
	}

	if (mutexCreate(&common.pool_lock) != EOK) {
		DEBUG("Failed to create lock\n");
		portDestroy(common.port);
		return -1;
	}

	rcc_init();
	rtc_init();
	//lcd_init();
	//uart_init();
	//gpio_init();
	//flash_init();
	//adc_init();

	for (i = 0; i < THREADS_MIN; ++i) {
		if (spawnThread() != EOK) {
			DEBUG("Failed to spawn initital thread #%d\n", i);
			--i;
		}
	}
}
