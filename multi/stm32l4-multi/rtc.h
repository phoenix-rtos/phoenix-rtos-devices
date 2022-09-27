/*
 * Phoenix-RTOS
 *
 * STM32L4 RTC driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Jakub Sejdak, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef RTC_H_
#define RTC_H_


#include "stm32l4-multi.h"


void rtc_setCalib(int value);


int rtc_getTime(rtctimestamp_t *timestamp);


int rtc_setTime(rtctimestamp_t *timestamp);


int rtc_setAlarm(rtctimestamp_t *timestamp);


int rtc_storeBackup(const void *buff, size_t bufflen);


int rtc_recallBackup(void *buff, size_t bufflen);


int rtc_init(void);

#endif
