/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32 RTC driver.
 *
 * Copyright 2017 Phoenix Systems
 * Author: Jakub Sejdak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RTCDRV_H_
#define _RTCDRV_H_


enum { MONDAY = 1, TUESDAY, WEDNESDAY, THURSDAY, FRIDAY, SATURDAY, SUNDAY };


typedef struct {
	int hours;
	int minutes;
	int seconds;

	int day;
	int month;
	int year;
	int wday;
} __attribute__((packed)) rtctimestamp_t;


#endif
