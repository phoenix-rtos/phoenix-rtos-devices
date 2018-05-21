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


#include "stm32-multi.h"


int rtc_get(rtctimestamp_t *timestamp);


int rtc_set(rtctimestamp_t *timestamp);


int rtc_init(void);

#endif
