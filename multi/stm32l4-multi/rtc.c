/*
 * Phoenix-RTOS
 *
 * STM32L4 RTC driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Author: Jakub Sejdak, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <sys/threads.h>
#include <sys/platform.h>

#include "common.h"
#include "rcc.h"
#include "exti.h"
#include "rtc.h"


enum { pwr_cr = 0, pwr_csr };


enum { tr = 0, dr, cr, isr, prer, wutr, alrmar = wutr + 2, alrmbr, wpr, ssr, shiftr, tstr, tsdr, tsssr, calr,
	tampcr, alrmassr, alrmbssr, or, bkp0r};


struct {
	volatile unsigned int *base;
	unsigned int prediv_s;

	handle_t lock;
} rtc_common;


static int rtc_alarm_handler(unsigned int n, void *arg)
{
	exti_clear_irq(18);
	return -1;
}


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
	dataBarier();

	*(rtc_common.base + wpr) = 0xff;
	pwr_lock();
}


static void _rtc_unlock(void)
{
	pwr_unlock();
	*(rtc_common.base + wpr) = 0xca;
	*(rtc_common.base + wpr) = 0x53;

	dataBarier();
}


static unsigned int timestampToTime(const rtctimestamp_t *timestamp)
{
	unsigned int time = 0;
	time |= (rtc_binToBcd(timestamp->hours) & 0x3f) << 16;
	time |= (rtc_binToBcd(timestamp->minutes) & 0x7f) << 8;
	time |= (rtc_binToBcd(timestamp->seconds) & 0x7f);
	return time;
}


static unsigned int timestampToDate(const rtctimestamp_t *timestamp)
{
	unsigned int date = 0;
	date |= (rtc_binToBcd(timestamp->day) & 0x3f);
	date |= (rtc_binToBcd(timestamp->month) & 0x1f) << 8;
	date |= (rtc_binToBcd(timestamp->year) & 0xff) << 16;
	date |= (timestamp->wday & 0x7) << 13;
	return date;
}


/* CAUTION - not tested, programmed according to the reference manual */
void rtc_setCalib(int value)
{
	unsigned int t = *(rtc_common.base + calr);

	t &= ~((0x7 << 13) | 0x1ff);

	if (value > 488)
		value = 488;

	if (value < -487)
		value = -487;

	if (value > 0) {
		t |= 1 << 15;
		value = 488 - value;
	}
	else {
		value = -value;
	}

	/* ppm to value */
	value = (100000 * value) / 954;
	value = (value + 55) / 100;

	t |= (unsigned int)value;

	mutexLock(rtc_common.lock);
	_rtc_unlock();
	*(rtc_common.base + calr) = t;
	_rtc_lock();
	mutexUnlock(rtc_common.lock);
}


int rtc_getTime(rtctimestamp_t *timestamp)
{
	unsigned int ssec, time, date, ssec2, time2, date2;

	mutexLock(rtc_common.lock);

	ssec = *(rtc_common.base + ssr);
	time = *(rtc_common.base + tr);
	date = *(rtc_common.base + dr);

	ssec2 = *(rtc_common.base + ssr);
	time2 = *(rtc_common.base + tr);
	date2 = *(rtc_common.base + dr);

	mutexUnlock(rtc_common.lock);

	if (ssec != ssec2 || time != time2) {
		ssec = ssec2;
		time = time2;
		date = date2;
	}

	timestamp->usecs = ((uint64_t) (rtc_common.prediv_s - (ssec & 0xffff)) * 1000 * 1000) / (rtc_common.prediv_s + 1);
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

	time = timestampToTime(timestamp);
	date = timestampToDate(timestamp);

	mutexLock(rtc_common.lock);
	_rtc_unlock();

	if (!(*(rtc_common.base + isr) & (1 << 6))) {
		*(rtc_common.base + isr) |= (1 << 7);
		dataBarier();
		while (!(*(rtc_common.base + isr) & (1 << 6)));
	}

	*(rtc_common.base + tr) = time;
	*(rtc_common.base + dr) = date;

	*(rtc_common.base + isr) &= ~(1 << 7);

	_rtc_lock();
	mutexUnlock(rtc_common.lock);

	return EOK;
}


int rtc_setAlarm(rtctimestamp_t *timestamp)
{
	unsigned int ssec, time;

	time = timestampToTime(timestamp);
	ssec = rtc_common.prediv_s - (((uint64_t) timestamp->usecs * (rtc_common.prediv_s + 1)) / (1000 * 1000));

	mutexLock(rtc_common.lock);
	_rtc_unlock();
	/* Disable the alarm. This clears RTC_ISR.ALR*F flags as a side effect */
	*(rtc_common.base + cr) &= ~(1 << 8);
	dataBarier();
	while (!(*(rtc_common.base + isr) & 0x1));

	/* Only hours, seconds, and subseconds are compared */
	*(rtc_common.base + alrmassr) = (0xf << 24) | (ssec & 0x7fff);
	*(rtc_common.base + alrmar) = (1 << 31) | time;

	dataBarier();
	/* Enable the alarm and its interrupt */
	*(rtc_common.base + cr) |= (1 << 12) | (1 << 8);

	_rtc_lock();
	mutexUnlock(rtc_common.lock);

	return EOK;
}


int rtc_init(void)
{
	rtc_common.base = (void *)0x40002800;

	mutexCreate(&rtc_common.lock);

	pwr_unlock();
	devClk(pctl_rtc, 1);
	pwr_lock();

	rtc_common.prediv_s = *(rtc_common.base + prer) & 0x7fff;

	exti_configure(18, exti_irqevent, exti_rising);

	interrupt(rtc_alarm_irq, rtc_alarm_handler, NULL, 0, NULL);

	return EOK;
}
