/*
 * Phoenix-RTOS
 *
 * ALS31300 i2c Magnetometer Sensor driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sys/minmax.h>

#include <sys/msg.h>
#include <posix/utils.h>

#include <i2c.h>

#define LOG_TAG       "mag-als31300: "
#define log(fmt, ...) printf(LOG_TAG fmt "\n", ##__VA_ARGS__)


#define I2C_DEV_ADDR 0x60u
#define CFG_REG1     0x02u
#define DATA_REG1    0x28u
#define DATA_REG2    0x29u

#define CFG_REG1_I2C_ADDR_MASK    0x0001fc00u
#define CFG_REG1_I2C_THRESH       0x00000200u
#define CFG_REG1_ENABLE_Z         0x00000100u
#define CFG_REG1_ENABLE_Y         0x00000080u
#define CFG_REG1_ENABLE_X         0x00000040u
#define CFG_REG1_CUSTOMER_EE_MASK 0x0000001fu

/* 12-bit signed data */
#define DATA_SIGN_BIT           0x0800u
#define DATA_GAUSS_MULTIPLIER_X 1 /* ALS31300EEJASR-JOY: 1 LSB/G */
#define DATA_GAUSS_MULTIPLIER_Y 1 /* ALS31300EEJASR-JOY: 1 LSB/G */
#define DATA_GAUSS_MULTIPLIER_Z 4 /* ALS31300EEJASR-JOY: 0.25 LSB/G */
#define DATA_TEMP_MULTIPLIER    302
#define DATA_TEMP_SUBTRACTOR    1702

#define GAUSS_TO_MILITESLA_DIVIDER 10 /* 1 G = 0.1 mT */


/* handling device file requests */
#define DEV_ID_X    0
#define DEV_ID_Y    1
#define DEV_ID_Z    2
#define DEV_ID_TEMP 3

static const struct {
	const char *dev;
	int id;
} DEVICES[] = {
	{ "magX", DEV_ID_X },
	{ "magY", DEV_ID_Y },
	{ "magZ", DEV_ID_Z },
	{ "magTemp", DEV_ID_TEMP },
};


struct mag_data_s {
	/* values in microTesla (0.001 milliTesla) */
	int32_t x;
	int32_t y;
	int32_t z;

	/* value in milliCelsius */
	int32_t temp;
};


static int read32(uint8_t addr, uint32_t *valOut)
{
	uint8_t val[4] = {};
	if (i2c_regRead(I2C_DEV_ADDR, addr, val, sizeof(val)) < 0) {
		log("failed to read register 0x%02x", addr);
		return -1;
	}

	*valOut = (val[0] << 24) | (val[1] << 16) | (val[2] << 8) | val[3];
	return 0;
}


static int read64(uint8_t addr, uint32_t *valOut)
{
	uint8_t val[8] = {};
	if (i2c_regRead(I2C_DEV_ADDR, addr, val, sizeof(val)) < 0) {
		log("failed to read register 0x%02x", addr);
		return -1;
	}

	valOut[0] = (val[0] << 24) | (val[1] << 16) | (val[2] << 8) | val[3];
	valOut[1] = (val[4] << 24) | (val[5] << 16) | (val[6] << 8) | val[7];
	return 0;
}


static int getData(struct mag_data_s *data)
{
	int16_t rawX, rawY, rawZ, rawTemp;
	uint32_t reg[2];

	if (read64(DATA_REG1, reg) < 0) {
		return -1;
	}

	rawX = (reg[0] >> 20) & 0x0FF0;
	rawY = (reg[0] >> 12) & 0x0FF0;
	rawZ = (reg[0] >> 4) & 0x0FF0;
	rawTemp = (reg[0] & 0x3F) << 6;

	rawX |= ((reg[1] >> 16) & 0x0F);
	rawY |= ((reg[1] >> 12) & 0x0F);
	rawZ |= ((reg[1] >> 8) & 0x0F);
	rawTemp |= (reg[1] & 0x3F);

	rawX = (rawX ^ DATA_SIGN_BIT) - DATA_SIGN_BIT;
	rawY = (rawY ^ DATA_SIGN_BIT) - DATA_SIGN_BIT;
	rawZ = (rawZ ^ DATA_SIGN_BIT) - DATA_SIGN_BIT;

	/* EM field in microTesla */
	data->x = 1000 * (int32_t)rawX * DATA_GAUSS_MULTIPLIER_X / GAUSS_TO_MILITESLA_DIVIDER;
	data->y = 1000 * (int32_t)rawY * DATA_GAUSS_MULTIPLIER_Y / GAUSS_TO_MILITESLA_DIVIDER;
	data->z = 1000 * (int32_t)rawZ * DATA_GAUSS_MULTIPLIER_Z / GAUSS_TO_MILITESLA_DIVIDER;

	/* temperature in milliCelsius */
	data->temp = 1000 * DATA_TEMP_MULTIPLIER * (rawTemp - DATA_TEMP_SUBTRACTOR) / 4096;

	return 0;
}


/* verify CFG1 contents to ensure we have ALS31300 under I2C_DEV_ADDR */
static int verifyCfgReg(uint32_t val)
{
	if ((val & CFG_REG1_CUSTOMER_EE_MASK) != 0) {
		return -1; /* we don't store anything in customer EEPROM - should be 0 */
	}

	if ((val & CFG_REG1_I2C_ADDR_MASK) != 0) {
		return -1; /* factory configuration: eeprom-based i2c address should be 0x0 */
	}

	if (((val & CFG_REG1_ENABLE_X) == 0) || ((val & CFG_REG1_ENABLE_Y) == 0) || ((val & CFG_REG1_ENABLE_Z) == 0)) {
		return -1; /* all channels should be enabled */
	}

	if ((val & CFG_REG1_I2C_THRESH) == 0) {
		return -1; /* factory configuration: 1.8V i2c compatible mode enabled */
	}

	return 0;
}


/* probe for ALS31300 - return 0 if found, negative value otherwise */
static int probe(void)
{
	uint32_t reg;
	if (read32(CFG_REG1, &reg) < 0) {
		/* no device with I2C_DEV_ADDR present on the i2c bus */
		return -1;
	}

	if (verifyCfgReg(reg) < 0) {
		return -1;
	}

	return 0;
}


static int handleRead(unsigned int devId, char *buf, size_t size)
{
	struct mag_data_s data;
	int len;

	if (getData(&data) < 0) {
		return -EIO;
	}

	switch (devId) {
		case DEV_ID_X:
			len = snprintf(buf, size, "%d\n", data.x);
			return min(len, (int)size);
		case DEV_ID_Y:
			len = snprintf(buf, size, "%d\n", data.y);
			return min(len, (int)size);
		case DEV_ID_Z:
			len = snprintf(buf, size, "%d\n", data.z);
			return min(len, (int)size);
		case DEV_ID_TEMP:
			len = snprintf(buf, size, "%d\n", data.temp);
			return min(len, (int)size);
		default:
			return -ENODEV;
	}

	return 0;
}


static void handleDumpMode(void)
{
	struct mag_data_s data;
	struct mag_data_s dataAbsMax = { 0 };

	while (true) {
		getData(&data);

		dataAbsMax.x = (dataAbsMax.x < abs(data.x)) ? abs(data.x) : dataAbsMax.x;
		dataAbsMax.y = (dataAbsMax.y < abs(data.y)) ? abs(data.y) : dataAbsMax.y;
		dataAbsMax.z = (dataAbsMax.z < abs(data.z)) ? abs(data.z) : dataAbsMax.z;
		dataAbsMax.temp = (dataAbsMax.temp < abs(data.temp)) ? abs(data.temp) : dataAbsMax.temp;

		log("current: x: %6.2f mT, y: %6.2f mT, z: %6.2f mT", (float)data.x / 1000, (float)data.y / 1000, (float)data.z / 1000);
		log("absMax : x: %6.2f mT, y: %6.2f mT, z: %6.2f mT", (float)dataAbsMax.x / 1000, (float)dataAbsMax.y / 1000, (float)dataAbsMax.z / 1000);
		log("temp: %6.3f C (absMax: %6.3f)", (float)data.temp / 1000, (float)dataAbsMax.temp / 1000);

		sleep(1);
	}
}


static void msgLoop(uint32_t port)
{
	msg_t msg;
	msg_rid_t rid;
	bool running = true;

	while (running) {
		int err = msgRecv(port, &msg, &rid);
		if (err < 0) {
			log("msgRecv returned error: %s", strerror(-err));
			if (err == -EINTR) {
				continue;
			}
			else {
				break; /* EINVAL (invalid/closed port) or ENOMEM - fatal error */
			}
		}

		switch (msg.type) {
			case mtOpen: /* fallthrough */
			case mtClose:
				msg.o.io.err = EOK;
				break;
			case mtWrite:
				/* TODO: support setting interrupt thresholds? */
				msg.o.io.err = -EINVAL;
				break;
			case mtRead:
				/* don't support partial reads, signal EOF */
				if (msg.i.io.offs > 0) {
					msg.o.io.err = 0; /* EOF */
				}
				else {
					msg.o.io.err = handleRead(msg.i.io.oid.id, msg.o.data, msg.o.size);
				}
				break;

			case mtUnlink:
				/* somebody is deleting our device files - exit */
				msg.o.io.err = EOK;
				running = false;
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static int createDevs(uint32_t port)
{
	oid_t dev;
	dev.port = port;

	for (unsigned int idx = 0; idx < (sizeof(DEVICES) / sizeof(DEVICES[0])); ++idx) {
		dev.id = DEVICES[idx].id;
		if (create_dev(&dev, DEVICES[idx].dev) < 0) {
			log("failed to create dev: %s", DEVICES[idx].dev);
			return -1;
		}
	}

	return 0;
}


static void removeDevs(void)
{
	char path[255];

	for (unsigned int idx = 0; idx < (sizeof(DEVICES) / sizeof(DEVICES[0])); ++idx) {
		snprintf(path, sizeof(path), "/dev/%s", DEVICES[idx].dev);
		unlink(path);
	}
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [OPTIONS] [i2c_bus_no <1,4>]\n", progname);
	printf("\t-d       Dump mode (don't create devices, write readings to stdout)\n");
	printf("\t-D       Daemonize after probing\n");
	printf("\t-h       This help message\n");
}


static void signal_exit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	unsigned int i2cBusNo;
	uint32_t port;
	int dumpMode = 0;
	int daemonize = 0;

	while (true) {
		int c = getopt(argc, argv, "Ddh");
		if (c == -1) {
			break;
		}

		switch (c) {
			case 'D':
				daemonize = 1;
				break;
			case 'd':
				dumpMode = 1;
				break;
			case 'h':
				print_usage(argv[0]);
				return 0;
			default:
				print_usage(argv[0]);
				return 1;
		}
	}

	if (optind >= argc) {
		log("missing i2c bus number");
		print_usage(argv[0]);
		return 1;
	}

	i2cBusNo = atoi(argv[optind]);
	if (i2c_init(i2cBusNo) < 0) {
		log("i2c initialization failed");
		return 1;
	}

	if (probe() < 0) {
		log("device not found");
		return 2;
	}

	if (dumpMode == 1) {
		/* don't create devices, just obtain and print data */
		handleDumpMode();
		return 0;
	}

	/* Daemonizing first to make all initialization in child process.
	 * Otherwise the port will be destroyed when parent exits. */
	if (daemonize) {
		pid_t pid;

		/* set exit handler */
		signal(SIGUSR1, signal_exit);
		/* Fork off the parent process */
		pid = fork();
		if (pid < 0) {
			log("fork failed: [%d] -> %s", errno, strerror(errno));
			exit(EXIT_FAILURE);
		}

		if (pid > 0) {
			/* PARENT: wait for initialization to finish and then exit */
			sleep(10);

			log("failed to communicate with child");
			exit(EXIT_FAILURE);
		}
		/* CHILD: set default handler back */
		signal(SIGUSR1, SIG_DFL);
	}

	/* TODO: configure thresholds and interrupt? */

	if (portCreate(&port) != EOK) {
		return 3;
	}

	if (createDevs(port) < 0) {
		/* NOTE: portDestroy needs to be first, otherwise removing devices would hang */
		portDestroy(port);
		removeDevs();
		return 4;
	}

	if (daemonize) {
		/* init completed - wake parent */
		kill(getppid(), SIGUSR1);
	}
	log("initialized");
	msgLoop(port);

	log("exiting");
	portDestroy(port);
	removeDevs();
	return 0;
}
