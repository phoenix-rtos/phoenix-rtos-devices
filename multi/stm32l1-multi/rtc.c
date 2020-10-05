/*
 * Phoenix-RTOS
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


#include <errno.h>
#include <sys/threads.h>
#include <sys/platform.h>

#include "common.h"
#include "rcc.h"
#include "rtc.h"


enum { pwr_cr = 0, pwr_csr };


enum { tr = 0, dr, cr, isr, prer, wutr, calibr, wpr = 9, ssr};


struct {
	volatile unsigned int *base;

	handle_t lock;
} rtc_common;


static char rtc_bcdToBin(char bcd)
{
	return (((bcd & 0xf0) >> 4) * 10) + (bcd & 0xf);
}


static char rtc_binToBcd(char bin)
{
	char bcdhigh = 0;

	while (bin >= 10)  {
		bcdhigh++;
		bin -= 10;
	}

	return (bcdhigh << 4) | bin;
}


static void _rtc_lock(void)
{
	*(rtc_common.base + wpr) = 0xff;
	pwr_lock();

	dataBarier();
}


static void _rtc_unlock(void)
{
	pwr_unlock();
	*(rtc_common.base + wpr) = 0xca;
	*(rtc_common.base + wpr) = 0x53;

	dataBarier();
}


void rtc_setCalib(int value)
{
	unsigned int t;

	mutexLock(rtc_common.lock);
	_rtc_unlock();

	t = *(rtc_common.base + calibr) & ~((1 << 7) | 0x1f);

	value /= 2;

	if (value < 0) {
		t |= 1 << 7;
		value = -value;
	}
	else {
		++value;
		value /= 2;
	}

	t |= value & 0x1f;

	*(rtc_common.base + calibr) = t;

	_rtc_lock();
	mutexUnlock(rtc_common.lock);
}


int rtc_getTime(rtctimestamp_t *timestamp)
{
	unsigned int time, date;

	mutexLock(rtc_common.lock);

	time = *(rtc_common.base + tr);
	date = *(rtc_common.base + dr);

	mutexUnlock(rtc_common.lock);

	timestamp->hours = rtc_bcdToBin((time >> 16) & 0x3f);
	timestamp->minutes = rtc_bcdToBin((time >> 8) & 0x7f);
	timestamp->seconds = rtc_bcdToBin(time & 0x7f);

	timestamp->day = rtc_bcdToBin(date & 0x3f);
	timestamp->month = rtc_bcdToBin((date >> 8) & 0x1f);
	timestamp->year = rtc_bcdToBin((date >> 16) & 0xff);
	timestamp->wday = (date >> 13) & 0x7;

	return EOK;
}


int rtc_setTime(rtctimestamp_t *timestamp)
{
	unsigned int time, date;

	time = 0;
	time |= (rtc_binToBcd(timestamp->hours) & 0x3f) << 16;
	time |= (rtc_binToBcd(timestamp->minutes) & 0x7f) << 8;
	time |= (rtc_binToBcd(timestamp->seconds) & 0x7f);

	date = 0;
	date |= (rtc_binToBcd(timestamp->day) & 0x3f);
	date |= (rtc_binToBcd(timestamp->month) & 0x1f) << 8;
	date |= (rtc_binToBcd(timestamp->year) & 0xff) << 16;
	date |= (timestamp->wday & 0x7) << 13;

	mutexLock(rtc_common.lock);

	_rtc_unlock();

	if (!(*(rtc_common.base + isr) & (1 << 6))) {
		*(rtc_common.base + isr) |= (1 << 7);
		while (!(*(rtc_common.base + isr) & (1 << 6)));
	}

	*(rtc_common.base + tr) = time;
	*(rtc_common.base + dr) = date;

	*(rtc_common.base + isr) &= ~(1 << 7);

	_rtc_lock();

	mutexUnlock(rtc_common.lock);

	return EOK;
}


int rtc_init(void)
{
	rtc_common.base = (void *)0x40002800;

	mutexCreate(&rtc_common.lock);

	pwr_unlock();
	rcc_devClk(pctl_rtc, 1);
	pwr_lock();

	return EOK;
}
