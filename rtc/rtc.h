/*
 * %LICENSE%
 * 
 * Copyright 2012 Phoenix Systems
 *
 */

#ifndef DEV_RTC__H_
#define DEV_RTC__H_

#include <hal/if.h> // needed for "time_t"
#include <include/time.h>

#include <lib/timeconv.h>

extern int rtc_getlocaltime(struct tm *time);
extern int rtc_setlocaltime(struct tm *time);
//TODO - add support for alarms

extern int rtc_gettimeofday(struct timespec *t);

extern int _rtc_init(void);

#endif
