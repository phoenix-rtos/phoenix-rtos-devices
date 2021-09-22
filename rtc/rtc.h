/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL RTC driver API
 *
 * Copyright 2018 Phoenix Systems
 * Author: Krystian Wasik
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef RTC_API_H
#define RTC_API_H

#include <stdint.h>

#include <sys/ioctl.h>

#include <arch.h>

struct rtc_time {
	int tm_sec;
	int tm_min;
	int tm_hour;
	int tm_mday;
	int tm_mon;
	int tm_year;
	int tm_wday;
	int tm_yday;
	int tm_isdst;
};

#define RTC_RD_TIME  _IOR('p', 0x09, struct rtc_time) /* Read RTC time   */
#define RTC_SET_TIME _IOW('p', 0x0a, struct rtc_time) /* Set RTC time    */

#endif /* RTC_API_H */
