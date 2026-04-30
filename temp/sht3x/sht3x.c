/*
 * Phoenix-RTOS
 *
 * SHT3x driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <sys/msg.h>
#include <posix/utils.h>

#include <i2c.h>

#include "sht3x-api.h"

#define RAW_STATUS_DATA_LEN 3
#define RAW_MEAS_DATA_LEN   6

#define CMD_INVALID 0xffff

#define DEFAULT_DEV_ADDR 0x44


static struct {
	int devAddr;
} common;


static int setCmd(uint16_t cmd)
{
	if (cmd == CMD_INVALID) {
		return -EINVAL;
	}
	return i2c_busWrite(common.devAddr, (uint8_t[]) { cmd >> 8, cmd }, sizeof(cmd));
}


static int regRead(uint16_t cmd, uint8_t *data_out, uint32_t len)
{
	if (cmd == CMD_INVALID) {
		return -EINVAL;
	}

	int ret = i2c_busWrite(common.devAddr, (uint8_t[]) { cmd >> 8, cmd }, sizeof(cmd));
	if (ret < 0) {
		return ret;
	}

	ret = i2c_busRead(common.devAddr, data_out, len);
	if (ret == -EIO) {
		return -EAGAIN; /* Interpret register read nack as register value not ready */
	}

	return ret;
}


static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc = 0xFF;

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


static uint16_t periodic2cmd(uint8_t repId, uint8_t freqId)
{
	if (freqId >= sht3x_freqCnt || repId >= sht3x_repeatabilityCnt) {
		return CMD_INVALID;
	}

	static const struct {
		uint8_t msb;
		uint8_t lsb[sht3x_repeatabilityCnt];
	} params2cmd[sht3x_freqCnt] = {
		[sht3x_freq0_5Hz] = { 0x20, { 0x32, 0x24, 0x2f } },
		[sht3x_freq1Hz] = { 0x21, { 0x30, 0x26, 0x2d } },
		[sht3x_freq2Hz] = { 0x22, { 0x36, 0x20, 0x2b } },
		[sht3x_freq4Hz] = { 0x23, { 0x34, 0x22, 0x29 } },
		[sht3x_freq10Hz] = { 0x27, { 0x37, 0x21, 0x2a } }
	};

	return (uint16_t)params2cmd[freqId].msb << 8 | params2cmd[freqId].lsb[repId];
}


static int raw2temp(uint16_t raw)
{
	const int PRECISION = 10;
	const int SCALER = 1000 / PRECISION;

	int res = (int64_t)PRECISION * raw * 175 / UINT16_MAX - PRECISION * 45;
	return SCALER * res;
}


static int raw2rh(uint16_t raw)
{
	const int PRECISION = 10;
	const int SCALER = 1000 / PRECISION;

	int res = (int64_t)PRECISION * raw * 100 / UINT16_MAX;
	return SCALER * res;
}


/* returns temperature in mC and relative humidity in m% */
static int getMeasurement(int *temp, int *rh)
{
	uint8_t raw[RAW_MEAS_DATA_LEN];

	int ret = regRead(sht3x_cmdFetchData, raw, sizeof(raw));
	if (ret < 0) {
		return ret;
	}

	if (crc8(&raw[0], 2) != raw[2] || crc8(&raw[3], 2) != raw[5]) {
		printf("sht3x: crc error\n");
		return -EINVAL;
	}

	*temp = raw2temp(raw[0] << 8 | raw[1]);
	*rh = raw2rh(raw[3] << 8 | raw[4]);

	return 0;
}


static int getStatus(uint16_t *status)
{
	uint8_t data[RAW_STATUS_DATA_LEN];
	int ret = regRead(sht3x_cmdGetStatus, data, sizeof(data));
	if (ret < 0) {
		return ret;
	}

	if (crc8(data, sizeof(data) - 1) != data[sizeof(data) - 1]) {
		return -EINVAL;
	}
	*status = data[0] << 8 | data[1];

	return 0;
}


static int devctl(msg_t *msg)
{
	sht3x_i_devctl_t data;

	memcpy(&data, msg->i.raw, sizeof(sht3x_i_devctl_t));

	int ret = 0;
	switch (data.type) {
		case sht3x_typeMeasConfig: {
			uint16_t cmd = periodic2cmd(data.measConfig.repeatability, data.measConfig.freq);
			if (cmd == CMD_INVALID) {
				ret = -ENOTSUP;
				break;
			}

			ret = setCmd(cmd);
		} break;

		case sht3x_typeCmd:
			if (data.cmd == sht3x_cmdGetStatus) {
				ret = regRead(data.cmd, msg->o.data, msg->o.size < RAW_STATUS_DATA_LEN ? msg->o.size : RAW_STATUS_DATA_LEN);
			}
			else if (data.cmd == sht3x_cmdFetchData) {
				ret = regRead(data.cmd, msg->o.data, msg->o.size < RAW_MEAS_DATA_LEN ? msg->o.size : RAW_MEAS_DATA_LEN);
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
	static int temp = 0, rh = 0;

	uint32_t port = (uint32_t)arg;
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		int err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			printf("sht3x: msgRecv returned error: %s\n", strerror(-err));
			if (err != -EINTR) {
				break; /* EINVAL (invalid/closed port) or ENOMEM - fatal error */
			}
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtWrite:
				msg.o.err = -EBUSY;
				break;

			case mtRead: {
				int ret = getMeasurement(&temp, &rh);
				if (ret < 0 && ret != -EAGAIN) { /* If new measurement is not ready, then send previous value */
					msg.o.err = ret;
					break;
				}

				ret = snprintf(msg.o.data, msg.o.size, "%d,%d\n", temp, rh);
				if (ret < 0) {
					msg.o.err = ret;
					break;
				}

				msg.o.err = 0; /* FIXME: read should return number of bytes read */
			} break;

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


static void usage(const char *progname)
{
	printf("Usage: %s i2c_bus_no temp_device_no [dev_addr]\n", progname);
}


int main(int argc, char **argv)
{
	if (argc != 3 && argc != 4) {
		usage(argv[0]);
		return 1;
	}

	int i2cBusNo = strtoul(argv[1], NULL, 0);
	if (i2c_init(i2cBusNo) < 0) {
		printf("sht3x: i2c initialization failed\n");
		return 1;
	}

	int devNo = strtoul(argv[2], NULL, 0);
	char devname[sizeof("sht3x-x")];
	snprintf(devname, sizeof(devname), "sht3x-%u", devNo % 10);

	uint32_t port;
	if (portCreate(&port) != EOK) {
		return 2;
	}

	oid_t dev = {
		.port = port,
		.id = 0,
	};
	if (create_dev(&dev, devname) < 0) {
		printf("sht3x: could not create device\n");
		return 3;
	}

	common.devAddr = DEFAULT_DEV_ADDR;
	if (argc == 4) {
		common.devAddr = strtoul(argv[3], NULL, 0);
	}

	uint16_t status;
	int ret = getStatus(&status);
	if (ret < 0) {
		printf("sht3x: status readout error %d\n", ret);
		return 4;
	}

	ret = setCmd(periodic2cmd(sht3x_repeatabilityLow, sht3x_freq1Hz));
	if (ret < 0) {
		printf("sht3x: measurement setup error %d\n", ret);
		return 5;
	}

	printf("sht3x: initialized, status: %2x\n", status);
	thread((void *)port);

	/* Should never happen */
	printf("sht3x: exiting\n");
	return 0;
}
