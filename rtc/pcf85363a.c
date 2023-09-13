/*
 * Phoenix-RTOS
 *
 * PCF85363A i2c RTC driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

#include <sys/msg.h>
#include <sys/ioctl.h>
#include <posix/utils.h>

#include <i2c.h>
#include <rtc.h>

/* NOTEs:
 *  - PCF85363A auto-increments register address post-read, so we can read multiple consecutive registers with single read
 *  - we configure / use TSR3 register to hold Last Timestamp the RTC switched to battery operation (last "POWERFAIL" event)
 *  - we configure / use TSR2 register to hold First Timestamp the RTC switched to battery operation (real "POWERFAIL" event?)
 *  - we use battery-backed generic RTC RAM memory to check if battery supply has failed in the meantime
 *  - RTC alarm generation with interrupt pin is not used as we're not planning on putting main CPU to sleep
 */

static const uint8_t DEV_ADDR = 0x51u;             /* PCF85363A */
static const uint8_t RTC_TIME_REG_ADDR = 0x01;     /* RTC time: seconds register */
static const uint8_t RTC_TSR2_REG_ADDR = 0x17;     /* RTC event (2): seconds register */
static const uint8_t RTC_TSR3_REG_ADDR = 0x1d;     /* RTC event (3): seconds register */
static const uint8_t RTC_TSR_MODE_REG_ADDR = 0x23; /* RTC trigger events configuration register */
static const uint8_t RTC_FLAGS_REG_ADDR = 0x2b;    /* RTC status flags */
static const uint8_t RTC_RAM_BYTE_REG_ADDR = 0x2c; /* single byte of battery-backed RAM */

/* clang-format off */
enum { dev_id_rtc, dev_id_first_powerfail, dev_id_last_powerfail, dev_id_batt_powerfail };
/* clang-format on */

static struct {
	unsigned int battery_failed;
} common;


static unsigned int from_bcd(uint8_t byte)
{
	return (byte >> 4) * 10 + (byte & 0xf);
}


static uint8_t to_bcd(uint8_t val)
{
	return ((val / 10) << 4) | (val % 10);
}


static int rtc_get_time(struct rtc_time *rtc_time, uint8_t reg_addr, int no_weekday)
{
	uint8_t timestamp[7] = { 0 };
	uint32_t data_len = sizeof(timestamp) - (no_weekday ? 1 : 0);

	memset(rtc_time, 0, sizeof(*rtc_time));
	if (i2c_regRead(DEV_ADDR, reg_addr, timestamp, data_len) < 0)
		return -EIO;

	rtc_time->tm_sec = from_bcd(timestamp[0] & 0x7f);
	rtc_time->tm_min = from_bcd(timestamp[1] & 0x7f);
	rtc_time->tm_hour = from_bcd(timestamp[2]);


	if (no_weekday) {
		/* no weekdays in alarm / event time registers */
		rtc_time->tm_mday = from_bcd(timestamp[3]);       /* 1-31 */
		rtc_time->tm_mon = from_bcd(timestamp[4]) - 1;    /* 0-11; RTC holds 1-12 */
		rtc_time->tm_year = 100 + from_bcd(timestamp[5]); /* year since 1900, RTC holds only last 2 digids - assume 20XX */
	}
	else {
		rtc_time->tm_mday = from_bcd(timestamp[3]);       /* 1-31 */
		rtc_time->tm_wday = from_bcd(timestamp[4]);       /* weekday 0-6; 0: sunday (RTC has the same semantics) */
		rtc_time->tm_mon = from_bcd(timestamp[5]) - 1;    /* 0-11; RTC holds 1-12 */
		rtc_time->tm_year = 100 + from_bcd(timestamp[6]); /* year since 1900, RTC holds only last 2 digids - assume 20XX */
	}


	rtc_time->tm_yday = 0;  /* FIXME?: yday not stored in RTC */
	rtc_time->tm_isdst = 0; /* RTC holds UTC time, no DST */

	return 0;
}


static int rtc_set_time(const struct rtc_time *rtc_time)
{
	if (rtc_time == NULL)
		return -EINVAL;

	uint8_t data[] = {
		RTC_TIME_REG_ADDR,
		to_bcd(rtc_time->tm_sec),
		to_bcd(rtc_time->tm_min),
		to_bcd(rtc_time->tm_hour),
		to_bcd(rtc_time->tm_mday),
		to_bcd(rtc_time->tm_wday),
		to_bcd(rtc_time->tm_mon + 1),    /* 0-11 -> 1-12 */
		to_bcd(rtc_time->tm_year % 100), /* since 1900, assume 20XX and hold only last 2 digits */
	};


	if (i2c_busWrite(DEV_ADDR, data, sizeof(data)) < 0)
		return -EIO;

	return EOK;
}


static void dev_ctl(msg_t *msg)
{
	unsigned long request;
	int res;
	id_t id;
	const void *data = ioctl_unpack(msg, &request, &id);
	struct rtc_time time;

	if (id != dev_id_rtc) {
		printf("rtc: this device does not support ioctls: id=%llu\n", id);
		ioctl_setResponse(msg, request, -ENOSYS, NULL);
		return;
	}

	switch (request) {
		case RTC_RD_TIME:
			res = rtc_get_time(&time, RTC_TIME_REG_ADDR, 0);
			ioctl_setResponse(msg, request, res, &time);
			break;

		case RTC_SET_TIME:
			memcpy(&time, data, sizeof(struct rtc_time));
			res = rtc_set_time(&time);
			ioctl_setResponse(msg, request, res, NULL);
			break;

		default:
			printf("rtc: unsupported ioctl (cmd=0x%lx, group=0x%lx, type=0x%lx, len=%lu)",
				request & 0xff, IOCGROUP(request), request & IOC_DIRMASK, IOCPARM_LEN(request));
			ioctl_setResponse(msg, request, -EINVAL, NULL);
			break;
	}
}


static time_t rtc_time_to_unix(const struct rtc_time *rtc_time)
{
	struct tm t;

	t.tm_sec = rtc_time->tm_sec;
	t.tm_min = rtc_time->tm_min;
	t.tm_hour = rtc_time->tm_hour;
	t.tm_mday = rtc_time->tm_mday;
	t.tm_mon = rtc_time->tm_mon;
	t.tm_year = rtc_time->tm_year;
	t.tm_wday = rtc_time->tm_wday;
	t.tm_yday = rtc_time->tm_yday;
	t.tm_isdst = rtc_time->tm_isdst;

	return mktime(&t);
}


static int dev_read(oid_t *oid, offs_t offs, size_t len, void *data)
{
	char buf[32];
	struct rtc_time rtc_time;
	time_t unix_time;
	ssize_t available;
	int res;

	if (oid->id >= dev_id_rtc && oid->id <= dev_id_last_powerfail) {
		if (oid->id == dev_id_rtc)
			res = rtc_get_time(&rtc_time, RTC_TIME_REG_ADDR, 0);
		else if (oid->id == dev_id_first_powerfail)
			res = rtc_get_time(&rtc_time, RTC_TSR2_REG_ADDR, 1);
		else if (oid->id == dev_id_last_powerfail)
			res = rtc_get_time(&rtc_time, RTC_TSR3_REG_ADDR, 1);
		else
			return -EINVAL;

		if (res < 0)
			return res;


		unix_time = rtc_time_to_unix(&rtc_time);
		snprintf(buf, sizeof(buf), "%llu\n", unix_time);
	}
	else if (oid->id == dev_id_batt_powerfail) {
		snprintf(buf, sizeof(buf), "%u\n", common.battery_failed);
	}
	else {
		return -EINVAL;
	}

	available = (ssize_t)strlen(buf) - offs;
	if (available <= 0)
		return 0;
	if (available < (ssize_t)len)
		len = available;

	memcpy(data, &buf[offs], len);
	return len;
}


static void thread(void *arg)
{
	uint32_t port = (uint32_t)arg;
	msg_t msg;
	msg_rid_t rid;
	int err;

	for (;;) {
		if ((err = msgRecv(port, &msg, &rid)) < 0) {
			printf("rtc: msgRecv returned error: %s\n", strerror(-err));
			if (err == -EINTR)
				continue;
			else
				break; /* EINVAL (invalid/closed port) or ENOMEM - fatal error */
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;
			case mtWrite:
				msg.o.io.err = -EINVAL;
				break;
			case mtRead:
				msg.o.io.err = dev_read(&msg.i.io.oid, msg.i.io.offs, msg.o.size, msg.o.data);
				break;
			case mtDevCtl:
				dev_ctl(&msg);
				break;
			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static int create_devs(uint32_t port)
{
	oid_t dev;

	dev.port = port;
	dev.id = dev_id_rtc;

	if (create_dev(&dev, "rtc0") < 0) {
		printf("rtc: could not create device rtc0\n");
		return -1;
	}

	/* unix timestamp of the first time since last full bootup we've switched to battery supply */
	dev.id = dev_id_first_powerfail;
	if (create_dev(&dev, "rtc_first_powerfail") < 0) {
		printf("rtc: could not create device rtc_first_powerfail\n");
		return -1;
	}

	/* unix timestamp of the last time since we've switched to battery supply */
	dev.id = dev_id_last_powerfail;
	if (create_dev(&dev, "rtc_last_powerfail") < 0) {
		printf("rtc: could not create device rtc_last_powerfail\n");
		return -1;
	}

	/* 0/1 state if RTC battery supply failed (rtc contents is invalid) */
	dev.id = dev_id_batt_powerfail;
	if (create_dev(&dev, "rtc_batt_powerfail") < 0) {
		printf("rtc: could not create device rtc_batt_powerfail\n");
		return -1;
	}

	return 0;
}


/* assume errors during setup are not fatal as they control periperial RTC functions, not the main one */
static void setup_rtc(void)
{
	const uint8_t battery_check_magic = 0x5a;
	const uint8_t event_config[] = {
		RTC_TSR_MODE_REG_ADDR,
		(2 << 6) | (1 << 2), /* TSR3 = LB, TSR2 = FB */
	};
	const uint8_t ram_config[] = {
		RTC_RAM_BYTE_REG_ADDR,
		battery_check_magic,
	};
	const uint8_t reset_flags[] = {
		RTC_FLAGS_REG_ADDR,
		0,
	};

	uint8_t buf[1] = { 0 };
	int res;

	if ((res = i2c_busWrite(DEV_ADDR, event_config, sizeof(event_config))) < 0)
		printf("rtc: failed to setup the events configuration: %s\n", strerror(res));

	if ((res = i2c_regRead(DEV_ADDR, RTC_RAM_BYTE_REG_ADDR, buf, sizeof(buf))) < 0)
		printf("rtc: failed to read RTC RAM memory: %s\n", strerror(res));

	common.battery_failed = 0;
	if (buf[0] != battery_check_magic) {
		printf("rtc: battery failed while device was powered off\n");
		common.battery_failed = 1;
	}

	/* re-set the RAM contents */
	if ((res = i2c_busWrite(DEV_ADDR, ram_config, sizeof(ram_config))) < 0)
		printf("rtc: failed to setup RAM contents: %s\n", strerror(res));

	/* reset RTC flags to catch First Battery Switch event correctly next time */
	if ((res = i2c_busWrite(DEV_ADDR, reset_flags, sizeof(reset_flags))) < 0)
		printf("rtc: failed to clear RTC flags: %s\n", strerror(res));
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [i2c_bus_no <1,4>]\n", progname);
}


int main(int argc, char **argv)
{
	unsigned int i2c_bus_no;
	uint32_t port;

	if (argc != 2) {
		print_usage(argv[0]);
		return 1;
	}

	i2c_bus_no = atoi(argv[1]);
	if (i2c_init(i2c_bus_no) < 0) {
		printf("rtc: i2c initialization failed\n");
		return 1;
	}

	if (portCreate(&port) != EOK) {
		printf("rtc: port allocation failed\n");
		return 2;
	}

	if (create_devs(port) < 0)
		return 3;

	setup_rtc();

	puts("rtc: initialized");
	thread((void *)port);

	printf("rtc: exiting\n");
	return 0;
}
