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

#ifndef _RTC_H_
#define _RTC_H_


#include "stm32l1-multi.h"


void rtc_setCalib(int value);


int rtc_getTime(rtctimestamp_t *timestamp);


int rtc_setTime(rtctimestamp_t *timestamp);


int rtc_init(void);

#endif
