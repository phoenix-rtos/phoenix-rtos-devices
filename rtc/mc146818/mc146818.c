#include <lib/if.h>
#include <proc/if.h>
#include <main/if.h>


static int rtc_in(int reg);

uint8_t const bcd2bin_data[] = {
         0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 0, 0, 0, 0, 0, 0,
        10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 0, 0, 0, 0, 0, 0,
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 0, 0, 0, 0, 0, 0,
        30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 0, 0, 0, 0, 0, 0,
        40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 0, 0, 0, 0, 0, 0,
        50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 0, 0, 0, 0, 0, 0,
        60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 0, 0, 0, 0, 0, 0,
        70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 0, 0, 0, 0, 0, 0,
        80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 0, 0, 0, 0, 0, 0,
        90, 91, 92, 93, 94, 95, 96, 97, 98, 99
};

#define bcd2bin(b) (bcd2bin_data[b])

#define IO_RTC ((void*)0x070)
#define RTC_LOCK
#define RTC_UNLOCK

#define RTC_SEC         0x00
#define RTC_MIN         0x02
#define RTC_HRS         0x04
#define RTC_DAY         0x07
#define RTC_MONTH       0x08
#define RTC_YEAR        0x09


int static rtc_in(int reg)
{
	uint8_t val;
	int rtc_reg = -1;

	RTC_LOCK;
	if (rtc_reg != reg) {
		hal_inb((void*)0x84);
		hal_outb(IO_RTC, reg);
		rtc_reg = reg;
		hal_inb((void*)0x84);
	}
	val = hal_inb(IO_RTC + 1);
	RTC_UNLOCK;
	return (val);
}


int rtc_gettimeofday(struct timespec *t)
{
	int year = 2000 + bcd2bin(rtc_in(RTC_YEAR));
	int month = bcd2bin(rtc_in(RTC_MONTH));
	int day = bcd2bin(rtc_in(RTC_DAY));
	int hrs = bcd2bin(rtc_in(RTC_HRS));
	int min = bcd2bin(rtc_in(RTC_MIN));
	int sec = bcd2bin(rtc_in(RTC_SEC));
	int days;

	month -=2;
	if (month <= 0){
		year --;
		month += 12;
	}

	days = (year/4 - year/100 + year/400 + 367*month/12 + day) + year * 365 - 719499;
	t->tv_sec = ((days * 24 + hrs )*60 + min )*60 + sec;    
	t->tv_nsec = 0;

	return EOK;

}

int _rtc_init(void)
{
	return EOK;
}
