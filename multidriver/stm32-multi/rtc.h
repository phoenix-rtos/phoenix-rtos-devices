/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 RTC driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Jakub Sejdak, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RTC_H_
#define _RTC_H_

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


int rtc_get(rtctimestamp_t *timestamp);


int rtc_set(rtctimestamp_t *timestamp);


int rtc_init(void);

#endif
