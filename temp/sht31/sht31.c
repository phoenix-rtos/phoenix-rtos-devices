/*
 * Phoenix-RTOS
 *
 * NCT75 i2c Temperature Sensor driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>

#include <sys/msg.h>
#include <posix/utils.h>

#include <i2c.h>

#include "sht31-api.h"

#define RAW_STATUS_DATA_LEN 3
#define RAW_MEAS_DATA_LEN   6

#define CMD_INVALID 0xffff

#define DEV_ADDR 0x44


static int setCmd(uint16_t cmd)
{
	if (cmd == CMD_INVALID) {
		return -EINVAL;
	}
	return i2c_busWrite(DEV_ADDR, (uint8_t[]) { cmd >> 8, cmd }, sizeof(cmd));
}


static int regRead(uint16_t cmd, uint8_t *data_out, uint32_t len)
{
	if (cmd == CMD_INVALID) {
		return -EINVAL;
	}
	int ret = i2c_busWrite(DEV_ADDR, (uint8_t[]) { cmd >> 8, cmd }, sizeof(cmd));
	if (ret < 0) {
		return ret;
	}

	return i2c_busRead(DEV_ADDR, data_out, len);
}


static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0xFF;  // Initialization value

	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 8; bit > 0; bit--) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;
			}
			else {
				crc = (crc << 1);
			}
		}
	}
	return crc;
}


static int getStatus(uint16_t *status)
{
	uint8_t data[RAW_STATUS_DATA_LEN];
	int ret = regRead(sht31_cmdGetStatus, data, sizeof(data));
	if (ret < 0) {
		return ret;
	}

	if (crc8(data, sizeof(data) - 1) != data[sizeof(data) - 1]) {
		return -EINVAL;
	}
	*status = data[0] << 8 | data[1];

	return 0;
}


static uint16_t periodic2cmd(uint8_t repeatability, uint8_t freq)
{
	uint16_t cmd = CMD_INVALID;

	switch (freq) {
		case sht31_freq0_5Hz:
			cmd = 0x20 << 8;
			switch (repeatability) {
				case sht31_repeatabilityHigh:
					cmd |= 0x32;
					break;
				case sht31_repeatabilityMed:
					cmd |= 0x24;
					break;
				case sht31_repeatabilityLow:
					cmd |= 0x2f;
					break;
				default:
					cmd = CMD_INVALID;
					break;
			}
			break;
		case sht31_freq1Hz:
			cmd = 0x21 << 8;
			switch (repeatability) {
				case sht31_repeatabilityHigh:
					cmd |= 0x30;
					break;
				case sht31_repeatabilityMed:
					cmd |= 0x26;
					break;
				case sht31_repeatabilityLow:
					cmd |= 0x2d;
					break;
				default:
					cmd = CMD_INVALID;
					break;
			}
			break;
		case sht31_freq2Hz:
			cmd = 0x22 << 8;
			switch (repeatability) {
				case sht31_repeatabilityHigh:
					cmd |= 0x36;
					break;
				case sht31_repeatabilityMed:
					cmd |= 0x20;
					break;
				case sht31_repeatabilityLow:
					cmd |= 0x2b;
					break;
				default:
					cmd = CMD_INVALID;
					break;
			}
			break;
		case sht31_freq4Hz:
			cmd = 0x23 << 8;
			switch (repeatability) {
				case sht31_repeatabilityHigh:
					cmd |= 0x34;
					break;
				case sht31_repeatabilityMed:
					cmd |= 0x22;
					break;
				case sht31_repeatabilityLow:
					cmd |= 0x29;
					break;
				default:
					cmd = CMD_INVALID;
					break;
			}
			break;
		case sht31_freq10Hz:
			cmd = 0x27 << 8;
			switch (repeatability) {
				case sht31_repeatabilityHigh:
					cmd |= 0x37;
					break;
				case sht31_repeatabilityMed:
					cmd |= 0x21;
					break;
				case sht31_repeatabilityLow:
					cmd |= 0x2a;
					break;
				default:
					cmd = CMD_INVALID;
					break;
			}
			break;
		default:
			cmd = CMD_INVALID;
			break;
	}

	return cmd;
}


/* returns temperature in mC and relative humidity in m% */
static int getMeasurement(int *temp, int *rh)
{
	uint8_t raw[RAW_MEAS_DATA_LEN];

	int ret = regRead(sht31_cmdFetchData, raw, sizeof(raw));
	if (ret < 0) {
		return ret;
	}

	if (crc8(&raw[0], 2) != raw[2] || crc8(&raw[3], 2) != raw[5]) {
		printf("crc error\n");
		return -EINVAL;
	}

	const int PRECISION = 10;
	const int SCALER = 1000 / PRECISION;

	int tmp = raw[0] << 8 | raw[1];
	tmp = PRECISION * tmp * 175 / UINT16_MAX - PRECISION * 45;
	*temp = SCALER * tmp;

	tmp = raw[3] << 8 | raw[4];
	tmp = PRECISION * tmp * 100 / UINT16_MAX;
	*rh = SCALER * tmp;

	return 0;
}


static int devctl(msg_t *msg)
{
	sht31_i_devctl_t data;

	memcpy(&data, msg->i.raw, sizeof(sht31_i_devctl_t));

	int ret = 0;
	switch (data.type) {
		case sht31_typeMeasConfig: {
			uint16_t cmd = periodic2cmd(data.measConfig.repeatability, data.measConfig.freq);
			if (cmd == CMD_INVALID) {
				ret = -ENOTSUP;
				break;
			}

			ret = setCmd(cmd);
		} break;

		case sht31_typeCmd:
			if (data.cmd == sht31_cmdGetStatus) {
				ret = regRead(data.cmd, msg->o.data, RAW_STATUS_DATA_LEN);
			}
			else if (data.cmd == sht31_cmdFetchData) {
				ret = regRead(data.cmd, msg->o.data, RAW_MEAS_DATA_LEN);
			}
			else {
				ret = setCmd(data.cmd);
			}
			break;

		default:
			ret = -ENOTSUP;
			break;
	}

	return ret;
}


static void thread(void *arg)
{
	uint32_t port = (uint32_t)arg;
	msg_t msg;
	msg_rid_t rid;
	int err;

	for (;;) {
		if ((err = msgRecv(port, &msg, &rid)) < 0) {
			printf("sht31: msgRecv returned error: %s\n", strerror(-err));
			if (err == -EINTR)
				continue;
			else
				break; /* EINVAL (invalid/closed port) or ENOMEM - fatal error */
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;
			case mtWrite:
				msg.o.err = -EBUSY;
				break;
			case mtRead:
				/* don't support partial reads, signal EOF */
				if (msg.i.io.offs > 0) {
					msg.o.err = 0; /* EOF */
				}
				else {
					int temp = 0, rh = 0;
					int ret = getMeasurement(&temp, &rh);
					if (ret < 0) {
						msg.o.err = ret;
						break;
					}

					ret = snprintf(msg.o.data, msg.o.size, "%d,%d\n", temp, rh);
					if (ret < 0) {
						msg.o.err = ret;
					}

					msg.o.err = 0;
				}
				break;
			case mtDevCtl:
				msg.o.err = devctl(&msg);
				break;
			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [i2c_bus_no <1,4>] [temp_device_no]\n", progname);
}


int main(int argc, char **argv)
{
	unsigned int i2c_bus_no, dev_no;
	uint32_t port;
	oid_t dev;
	char devname[sizeof("sht31-x")];

	if (argc != 3) {
		print_usage(argv[0]);
		return 1;
	}

	i2c_bus_no = atoi(argv[1]);
	if (i2c_init(i2c_bus_no) < 0) {
		printf("sht31: i2c initialization failed\n");
		return 1;
	}

	dev_no = atoi(argv[2]);

	if (portCreate(&port) != EOK)
		return 2;

	dev.port = port;
	dev.id = 0;

	snprintf(devname, sizeof(devname), "sht31-%u", dev_no % 10);
	if (create_dev(&dev, devname) < 0) {
		printf("sht31: could not create device\n");
		return 3;
	}

	/* TODO: add optional address arg */

	uint16_t status;
	int ret = getStatus(&status);
	if (ret < 0) {
		printf("sht31: status readout error %d\n", ret);
		return 4;
	}

	ret = setCmd(periodic2cmd(sht31_repeatabilityLow, sht31_freq1Hz));
	if (ret < 0) {
		printf("sht31: measurement setup error %d\n", ret);
		return 5;
	}

	printf("sht31: initialized, status: %2x\n", status);
	thread((void *)port);

	printf("sht31: exiting\n");
	return 0;
}
