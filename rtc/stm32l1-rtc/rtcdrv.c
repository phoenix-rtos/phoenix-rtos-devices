/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32 RTC driver.
 *
 * Copyright 2017 Phoenix Systems
 * Author: Jakub Sejdak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>

#include <sys/threads.h>
#include <sys/msg.h>

#include "rtcdrv.h"


static const struct {
	volatile unsigned int *pwr;
	volatile unsigned int *rcc;
	volatile unsigned int *rtc;
} rtc_common = {
	(void *) 0x40007000,
	(void *) 0x40023800,
	(void *) 0x40002800,
};


enum { pwr_cr = 0, pwr_csr };


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbenr = 7, rcc_apb2enr, rcc_apb1enr, rcc_csr = 13};


enum { rtc_tr = 0, rtc_dr, rtc_cr, rtc_isr, rtc_prer, rtc_wutr, rtc_wpr = 9, rtc_ssr};


static void rtc_lockRegs(void)
{
	/* Lock RTC */
	*(rtc_common.rtc + rtc_wpr) = 0xff;

	/* Reset DBP bit */
	*(rtc_common.pwr + pwr_cr) &= ~(1 << 8);

	__asm__ volatile ("dmb");
}


static void rtc_unlockRegs(void)
{
	/* Set DBP bit */
	*(rtc_common.pwr + pwr_cr) |= 1 << 8;

	/* Unlock RTC */
	*(rtc_common.rtc + rtc_wpr) = 0xca;
	*(rtc_common.rtc + rtc_wpr) = 0x53;

	__asm__ volatile ("dmb");
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


static void rtc_setInitMode(int enable)
{
	if (enable) {
		/* Check if the Initialization mode is set */
		if (*(rtc_common.rtc + rtc_isr) & (1 << 6))
			return;

		*(rtc_common.rtc + rtc_isr) |= (1 << 7);
		while ((*(rtc_common.rtc + rtc_isr) & (1 << 6)) == 0);
	}
	else
		*(rtc_common.rtc + rtc_isr) &= ~(1 << 7);
}


static void rtc_getTimestamp(rtctimestamp_t *timestamp)
{
	unsigned int time, date;

	time = *(rtc_common.rtc + rtc_tr);
	date = *(rtc_common.rtc + rtc_dr);

	timestamp->hours = rtc_bcdToBin((time >> 16) & 0x3f);
	timestamp->minutes = rtc_bcdToBin((time >> 8) & 0x7f);
	timestamp->seconds = rtc_bcdToBin(time & 0x7f);

	timestamp->day = rtc_bcdToBin(date & 0x3f);
	timestamp->month = rtc_bcdToBin((date >> 8) & 0x1f);
	timestamp->year = rtc_bcdToBin((date >> 16) & 0xff);
	timestamp->wday = (date >> 13) & 0x7;
}


static void rtc_setTimestamp(rtctimestamp_t *timestamp)
{
	unsigned int time, date;

	time = 0;
	time |= (rtc_binToBcd(timestamp->hours) & 0x3f) << 16;
	time |= (rtc_binToBcd(timestamp->minutes) & 0x7f) << 8;
	time |= (rtc_binToBcd(timestamp->seconds) & 0x3f);

	date = 0;
	date |= (rtc_binToBcd(timestamp->day) & 0x3f);
	date |= (rtc_binToBcd(timestamp->month) & 0x1f) << 8;
	date |= (rtc_binToBcd(timestamp->year) & 0xff) << 16;
	date |= (timestamp->wday & 0x7) << 13;

	rtc_unlockRegs();
	rtc_setInitMode(1);

	*(rtc_common.rtc + rtc_tr) = time;
	*(rtc_common.rtc + rtc_dr) = date;

	rtc_setInitMode(0);
	rtc_lockRegs();
}


void main(void)
{
	rtctimestamp_t timestamp;
	msghdr_t hdr;
	unsigned int port;

	rtc_unlockRegs();

	/* Select LSE as clock source for RTC and LCD. */
	*(rtc_common.rcc + rcc_csr) |= 1 << 16;

	/* Enable RTC clock. */
	*(rtc_common.rcc + rcc_csr) |= (1 << 22);

	__asm__ volatile ("dmb");

	rtc_lockRegs();

	portCreate(&port);
	portRegister(port, "/rtcdrv");

	for (;;) {
		recv(port, &timestamp, sizeof(timestamp), &hdr);
		if (hdr.type == NOTIFY)
			continue;

		if (hdr.op == READ) {
			rtc_getTimestamp(&timestamp);
			respond(port, EOK, &timestamp, sizeof(timestamp));
		}
		else if (hdr.op == WRITE) {
			rtc_setTimestamp(&timestamp);
			respond(port, EOK, NULL, 0);
		}
	}
}
