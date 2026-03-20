/*
 * Phoenix-RTOS
 *
 * imx6ull RTC driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Rafał Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/msg.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <posix/utils.h>
#include <errno.h>
#include <phoenix/arch/armv7a/imx6ull/imx6ull.h>

#include "imx6ull-rtc.h"


struct {
	volatile uint32_t *rtcReg;
	uint32_t port;
} common;


static uint64_t rtc_readCounterReg(void)
{
	uint32_t lprstcmr, lprstcl;

	lprstcmr = common.rtcReg[SNVS_LPSRTCMR];
	lprstcl = common.rtcReg[SNVS_LPSRTCLR];

	return (uint64_t)(((uint64_t)lprstcmr << 32) | lprstcl);
}


static int rtc_readTime(uint32_t *seconds)
{
	uint64_t read1, read2, diff;
	uint8_t timeout = RTC_READ_TIMEOUT; /* 3 consecutive read cycles should guarantee proper value, we set limit to 10 */

	do {
		read1 = rtc_readCounterReg();
		read2 = rtc_readCounterReg();
		diff = read2 - read1;
		timeout--;

	} while ((diff > MAX_READOUT_DIFF) && (timeout > 0U));

	if (diff > MAX_READOUT_DIFF) {
		return -EIO;
	}

	/* divide by 32,768 to return time in seconds */
	*seconds = (uint32_t)(read2 >> 15);
	return EOK;
}


static int rtc_setState(uint8_t state)
{
	uint32_t timeout = RTC_SETSTATE_TIMEOUT;
	uint32_t enable;

	if (state == RTC_DISABLE) {
		enable = 0UL;
		common.rtcReg[SNVS_LPCR] &= ~(1UL << SNVS_LPCR_EN);
	}
	else {
		enable = 1UL;
		common.rtcReg[SNVS_LPCR] |= (1UL << SNVS_LPCR_EN);
	}

	enable = (enable + 1U) & 1U;

	while (((common.rtcReg[SNVS_LPCR] >> SNVS_LPCR_EN) & 1UL) == enable) {
		if (--timeout == 0UL) {
			return -1;
		}
	}

	return 0;
}


static int rtc_getTime(struct tm *timeUnix)
{
	time_t timeSec;
	uint32_t seconds;

	/* read rtc counter */
	if (rtc_readTime(&seconds) < 0) {
		return -EIO;
	}

	timeSec = (time_t)seconds;

	/* convert seconds to struct tm */
	(void)gmtime_r(&timeSec, timeUnix);

	return 0;
}


static int rtc_setTime(struct tm *timeUnix)
{
	time_t seconds;
	int ret;

	/* convert unix time to seconds */
	seconds = timegm(timeUnix);
	if (seconds < 0) {
		return -EINVAL;
	}

	/* disable RTC */
	ret = rtc_setState(RTC_DISABLE);

	if (ret < 0) {
		return -EIO;
	}

	/* set RTC registers */
	common.rtcReg[SNVS_LPSRTCLR] = (uint32_t)((uint64_t)seconds << 15);
	common.rtcReg[SNVS_LPSRTCMR] = (uint32_t)((uint64_t)seconds >> 17);

	/* enable RTC */
	ret = rtc_setState(RTC_ENABLE);

	if (ret < 0) {
		return -EIO;
	}

	return EOK;
}


static void parseDevctl(msg_t *msg)
{
	/* parse ioctl */
	int err;
	unsigned long req;
	id_t id;
	const void *data = NULL;
	struct tm timeUnix;

	data = ioctl_unpack(msg, &req, &id);

	switch (req) {
		case SNVS_RD_TIME:
			err = rtc_getTime(&timeUnix);
			ioctl_setResponse(msg, req, err, (void *)&timeUnix);
			break;
		case SNVS_SET_TIME:
			if (data == NULL) {
				err = -EINVAL;
			}
			else {
				memcpy((&timeUnix), data, sizeof(struct tm));
				err = rtc_setTime(&timeUnix);
			}
			ioctl_setResponse(msg, req, err, NULL);
			break;
		default:
			ioctl_setResponse(msg, req, -EINVAL, NULL);
			break;
	}
}


static int rtc_snvcInit(void)
{
	int ret;

	/* clear interrupts */
	common.rtcReg[SNVS_LPSR] = UINT32_MAX;

	/* enable RTC */
	ret = rtc_setState(RTC_ENABLE);

	if (ret < 0) {
		return -EIO;
	}

	return EOK;
}


static int rtc_init(void)
{
	int res;
	oid_t dev;

	common.rtcReg = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_PHYSMEM | MAP_DEVICE | MAP_UNCACHED | MAP_ANONYMOUS, -1, SNVS_BASE);
	if (common.rtcReg == MAP_FAILED) {
		fprintf(stderr, "[RTC] Memory allocation failed\n");
		return -ENOMEM;
	}

	res = rtc_snvcInit();
	if (res != EOK) {
		fprintf(stderr, "[RTC] Peripheral init failed\n");
		(void)munmap((void *)common.rtcReg, _PAGE_SIZE);
		return -EIO;
	}

	res = portCreate(&common.port);
	if (res != EOK) {
		fprintf(stderr, "[RTC] Port registration failed\n");
		(void)munmap((void *)common.rtcReg, _PAGE_SIZE);
		return -ENOMEM;
	}

	dev.port = common.port;
	res = create_dev(&dev, "rtc");
	if (res != EOK) {
		fprintf(stderr, "[RTC] Device registration failed\n");
		(void)portDestroy(common.port);
		(void)munmap((void *)common.rtcReg, _PAGE_SIZE);
		return -ENOMEM;
	}

	return EOK;
}


static void rtc_thread(void)
{
	msg_t msg;
	msg_rid_t rid;
	int res;

	for (;;) {
		res = msgRecv(common.port, &msg, &rid);
		if (res < 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;
			case mtDevCtl:
				parseDevctl(&msg);
				break;
			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(common.port, &msg, rid);
	}
}


int main(int argc, char **argv)
{
	int res;
	res = rtc_init();
	if (res < 0) {
		return -res;
	}

	rtc_thread();

	return 0;
}
