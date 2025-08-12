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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/platform.h>

#include "common.h"
#include "rcc.h"
#include "exti.h"
#include "rtc.h"
#include "stm32l4-multi.h"

/* clang-format off */
#if defined(__CPU_STM32L4X6)
enum { tr = 0, dr, cr, isr, prer, wutr, alrmar = wutr + 2, alrmbr, wpr, ssr, shiftr, tstr, tsdr, tsssr, calr,
	tampcr, alrmassr, alrmbssr, or, bkp0r};
#elif defined(__CPU_STM32N6)
enum { tr = 0x0, dr, ssr, icsr, prer, wutr, cr, privcfgr, seccfgr, wpr, calr, shiftr, tstr, tsdr, tsssr,
	alrmar = 0x10, alrmassr, alrmbr, alrmbssr, sr, misr, smisr, scr, alrabinr = 0x1c, alrbbinr };
#endif
/* clang-format on */

#if defined(__CPU_STM32L4X6)
#define RTC_EXTI_LINE 18
#define RTC_INTERRUPT rtc_alarm_irq
#elif defined(__CPU_STM32N6)
/* On STM32N6 there are two RTC interrupts - secure and non-secure */
#define RTC_EXTI_LINE 18
#define RTC_INTERRUPT rtc_irq
#endif

#define BACKUP1_ID_REG    bkp0r
#define BACKUP_PAYLOAD_SZ RTC_BACKUP_SZ
#define BACKUP2_ID_REG    (BACKUP1_ID_REG + (BACKUP_PAYLOAD_SZ / sizeof(uint32_t)) + 1)
#define ID_VALUE(id)      ((id) & 0xffff)
#define ID_IS_VALID(id)   ((((id) >> 16) ^ ((id) & 0xffff)) == 0xffff)


struct {
	volatile unsigned int *base;
	unsigned int prediv_s;

	handle_t lock;
} rtc_common;


static int rtc_alarm_handler(unsigned int n, void *arg)
{
#if defined(__CPU_STM32L4X6)
	exti_clear_irq(RTC_EXTI_LINE);
#elif defined(__CPU_STM32N6)
	uint32_t previous = pwr_unlockFromIRQ();
	*(rtc_common.base + wpr) = 0xca;
	*(rtc_common.base + wpr) = 0x53;
	dataBarier();
	*(rtc_common.base + scr) = *(rtc_common.base + sr); /* Clear any active interrupts */
	dataBarier();
	*(rtc_common.base + wpr) = 0xff;
	pwr_lockFromIRQ(previous);
#endif
	return -1;
}


static char rtc_bcdToBin(char bcd)
{
	return (((bcd & 0xf0) >> 4) * 10) + (bcd & 0xf);
}


static char rtc_binToBcd(char bin)
{
	char bcdhigh = 0;

	while (bin >= 10) {
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

	timestamp->usecs = ((uint64_t)(rtc_common.prediv_s - (ssec & 0xffff)) * 1000 * 1000) / (rtc_common.prediv_s + 1);
	timestamp->hours = rtc_bcdToBin((time >> 16) & 0x3f);
	timestamp->minutes = rtc_bcdToBin((time >> 8) & 0x7f);
	timestamp->seconds = rtc_bcdToBin(time & 0x7f);

	timestamp->day = rtc_bcdToBin(date & 0x3f);
	timestamp->month = rtc_bcdToBin((date >> 8) & 0x1f);
	timestamp->year = rtc_bcdToBin((date >> 16) & 0xff);
	timestamp->wday = (date >> 13) & 0x7;

	return EOK;
}


static void _rtc_initMode(bool enable)
{
#if defined(__CPU_STM32L4X6)
	static const uint32_t reg_offs = isr;
	static const uint32_t bit_initf = 1 << 6;
	static const uint32_t bit_init = 1 << 7;
#elif defined(__CPU_STM32N6)
	static const uint32_t reg_offs = icsr;
	static const uint32_t bit_initf = 1 << 6;
	static const uint32_t bit_init = 1 << 7;
#endif

	if (enable) {
		if (!(*(rtc_common.base + reg_offs) & bit_initf)) {
			*(rtc_common.base + reg_offs) |= bit_init;
			dataBarier();
			while (!(*(rtc_common.base + reg_offs) & bit_initf)) {
				/* Wait for RTC to go into initialization state */
			}
		}
	}
	else {
		*(rtc_common.base + reg_offs) &= ~bit_init;
	}
}


int rtc_setTime(rtctimestamp_t *timestamp)
{
	unsigned int time, date;

	time = timestampToTime(timestamp);
	date = timestampToDate(timestamp);

	mutexLock(rtc_common.lock);
	_rtc_unlock();

	_rtc_initMode(true);
	*(rtc_common.base + tr) = time;
	*(rtc_common.base + dr) = date;
	_rtc_initMode(false);

	_rtc_lock();
	mutexUnlock(rtc_common.lock);

	return EOK;
}


static void _rtc_enableAlarm(bool enable)
{
	if (enable) {
		/* Enable the alarm and its interrupt */
		*(rtc_common.base + cr) |= (1 << 12) | (1 << 8);
	}
	else {
		/* Disable the alarm. This clears RTC_ISR.ALR*F flags as a side effect */
		*(rtc_common.base + cr) &= ~(1 << 8);
		dataBarier();
#if defined(__CPU_STM32L4X6)
		while (!(*(rtc_common.base + isr) & 0x1))
			;
#elif defined(__CPU_STM32N6)
		/* On STM32N6 the ALRAWF flag doesn't exist */
#endif
	}
}


int rtc_setAlarm(rtctimestamp_t *timestamp)
{
	unsigned int ssec, time;

	time = timestampToTime(timestamp);
	ssec = rtc_common.prediv_s - (((uint64_t)timestamp->usecs * (rtc_common.prediv_s + 1)) / (1000 * 1000));

	mutexLock(rtc_common.lock);
	_rtc_unlock();
	_rtc_enableAlarm(false);

	/* Only hours, seconds, and subseconds are compared */
	*(rtc_common.base + alrmassr) = (0xf << 24) | (ssec & 0x7fff);
	*(rtc_common.base + alrmar) = (1 << 31) | time;

	dataBarier();
	_rtc_enableAlarm(true);
	_rtc_lock();
	mutexUnlock(rtc_common.lock);

	return EOK;
}


#if defined(__CPU_STM32L4X6)
static int rtc_getLastStorage(uint32_t *lastID)
{
	uint32_t id[2], valid = 0, num;
	int ret;

	id[0] = *(rtc_common.base + BACKUP1_ID_REG);
	id[1] = *(rtc_common.base + BACKUP2_ID_REG);

	if (ID_IS_VALID(id[0])) {
		valid |= 1;
	}

	if (ID_IS_VALID(id[1])) {
		valid |= 2;
	}

	switch (valid) {
		case 0: /* No valid records */
			ret = -1;
			num = 0;
			break;

		case 1: /* Only record #1 is valid */
			ret = 0;
			num = ID_VALUE(id[0]);
			break;

		case 2: /* Only record #2 is valid */
			ret = 1;
			num = ID_VALUE(id[1]);
			break;

		default: /* Both records are valid */
			if (((ID_VALUE(id[0]) + 1) & 0xffff) == ID_VALUE(id[1])) {
				ret = 1;
				num = ID_VALUE(id[1]);
			}
			else {
				ret = 0;
				num = ID_VALUE(id[0]);
			}
			break;
	}

	if (lastID != NULL) {
		*lastID = num;
	}

	return ret;
}


int rtc_storeBackup(const void *buff, size_t bufflen)
{
	uint32_t idno;
	int index = (rtc_getLastStorage(&idno) == 0) ? 1 : 0;
	int idreg = (index == 0) ? BACKUP1_ID_REG : BACKUP2_ID_REG;
	int retval = -EINVAL;
	size_t i;
	uint32_t tmp[BACKUP_PAYLOAD_SZ / sizeof(uint32_t)] = { 0 };

	if ((buff != NULL) && (bufflen != 0) && (bufflen <= BACKUP_PAYLOAD_SZ)) {
		mutexLock(rtc_common.lock);
		pwr_unlock();
		dataBarier();

		/* Invalidate region */
		*(rtc_common.base + idreg) = 0;

		memcpy(tmp, buff, bufflen);

		for (i = 0; i < sizeof(tmp) / sizeof(tmp[0]); ++i) {
			*(rtc_common.base + idreg + 1 + i) = tmp[i];
		}

		/* Validate and store ID */
		idno = (idno + 1) & 0xffff;
		idno |= (~idno) << 16;
		*(rtc_common.base + idreg) = idno;

		dataBarier();
		pwr_lock();
		mutexUnlock(rtc_common.lock);

		retval = bufflen;
	}

	return retval;
}


int rtc_recallBackup(void *buff, size_t bufflen)
{
	int index = rtc_getLastStorage(NULL);
	int idreg = (index == 0) ? BACKUP1_ID_REG : BACKUP2_ID_REG;
	int retval;
	size_t i;
	uint32_t tmp[BACKUP_PAYLOAD_SZ / sizeof(uint32_t)] = { 0 };

	if (index < 0) {
		retval = -ENOENT;
	}
	else if ((buff == NULL) || (bufflen == 0) || (bufflen > BACKUP_PAYLOAD_SZ)) {
		retval = -EINVAL;
	}
	else {
		mutexLock(rtc_common.lock);
		for (i = 0; i < sizeof(tmp) / sizeof(tmp[0]); ++i) {
			tmp[i] = *(rtc_common.base + idreg + 1 + i);
		}
		mutexUnlock(rtc_common.lock);

		memcpy(buff, tmp, bufflen);
		retval = bufflen;
	}

	return retval;
}
#elif defined(__CPU_STM32N6)
int rtc_storeBackup(const void *buff, size_t bufflen)
{
	/* TODO: On STM32N6 backup memory is accessed in a different manner */
	return -ENOSYS;
}


int rtc_recallBackup(void *buff, size_t bufflen)
{
	/* TODO: On STM32N6 backup memory is accessed in a different manner */
	return -ENOSYS;
}
#endif


int rtc_init(void)
{
	rtc_common.base = RTC_BASE;

	mutexCreate(&rtc_common.lock);

	pwr_unlock();
	devClk(pctl_rtc, 1);
	pwr_lock();

	rtc_common.prediv_s = *(rtc_common.base + prer) & 0x7fff;

#if defined(__CPU_STM32L4X6)
	exti_configure(RTC_EXTI_LINE, exti_irqevent, exti_rising);
#elif defined(__CPU_STM32N6)
	_rtc_unlock();
	*(rtc_common.base + cr) &= ~(0xf << 12);            /* Disable all interrupts for now */
	*(rtc_common.base + scr) = *(rtc_common.base + sr); /* Clear any active interrupts */
	_rtc_lock();
#endif

	interrupt(RTC_INTERRUPT, rtc_alarm_handler, NULL, 0, NULL);

	return EOK;
}
