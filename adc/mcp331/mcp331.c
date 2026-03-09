/*
 * Phoenix-RTOS
 *
 * MCP33131/21/11-XX driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/msg.h>
#include <posix/utils.h>

#include "mcp331-low.h"
#include "mcp331-api.h"
#include "log.h"


#define READ_SIZE          (sizeof(uint16_t))
#define DEFAULT_RESOLUTION 16

static struct {
	uint32_t port;
	uint8_t dev;
	uint8_t res;
	struct {
		uint8_t bus;
		uint8_t chan;
	} spi;
} common;


static int devInit(void)
{
	oid_t dev;

	int res = portCreate(&common.port);
	if (res < 0) {
		log_error("Could not create port: %d", res);
		return -1;
	}

	dev.id = 0;
	dev.port = common.port;

	char devname[sizeof("mcp331-x")];
	snprintf(devname, sizeof(devname), "mcp331-%u", common.dev % 10);

	if (create_dev(&dev, devname) < 0) {
		log_error("Could not create device mcp331");
		return -1;
	}

	return 0;
}


static int devCtl(msg_t *msg)
{
	mcp331_i_devctl_t data;

	memcpy(&data, msg->i.raw, sizeof(mcp331_i_devctl_t));

	switch (data.cmd) {
		case mcp331_cmd__recalibrate:
			return -ENOTSUP; /* TODO: Implement */

		default:
			return -ENOSYS;
	}
}


static int devRead(uint8_t *buff, uint32_t len)
{
	if (len < READ_SIZE) {
		return -EINVAL;
	}

	uint8_t data[READ_SIZE];
	int ret = spi_exchange(common.spi.bus, common.spi.chan, data, len);
	if (ret < 0) {
		return ret;
	}

	assert(ret == READ_SIZE);

	uint16_t result = (((uint16_t)data[0] << 8) | (data[1])) >> (16 - common.res);
	data[0] = result >> 8;
	data[1] = result & 0xff;

	memcpy(buff, data, READ_SIZE);
	return READ_SIZE;
}


static void msgLoop(void)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		int err = msgRecv(common.port, &msg, &rid);
		if (err < 0) {
			log_error("msgRecv returned error: %s\n", strerror(-err));
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
				msg.o.err = -ENOSYS;
				break;

			case mtRead: {
				int ret = devRead(msg.o.data, msg.o.size);
				msg.o.err = ret > 0 ? 0 : ret; /* FIXME: [RTOS-1280] Read should return number of bytes read */
			} break;

			case mtDevCtl:
				msg.o.err = devCtl(&msg);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(common.port, &msg, rid);
	}
}


static int init(void)
{
	if (spi_init(common.spi.bus, common.spi.chan) < 0) {
		log_error("Failed to initialize spi");
		return -1;
	}

	if (devInit() < 0) {
		log_error("Device initialization failed");
		return -1;
	}

	return 0;
}


static void usage(const char *progname)
{
	printf("Usage: %s spi_no spi_ch dev_no resolution\n", progname);
}


static int parseArgs(int argc, char *argv[])
{
	if (argc != 5) {
		return -1;
	}

	common.spi.bus = strtoul(argv[1], NULL, 0);
	if (common.spi.bus > 3) {
		log_error("SPI bus must be in range <0,3>\n");
		return -1;
	}

	common.spi.chan = strtoul(argv[2], NULL, 0);
	if (common.spi.chan > 3) {
		log_error("SPI channel must be in range <0,3>\n");
		return -1;
	}

	common.dev = strtoul(argv[3], NULL, 0);
	if (common.dev > 9) {
		log_error("Dev no must be in range <0,9>\n");
		return -1;
	}

	common.res = strtoul(argv[4], NULL, 0);
	if (common.res != 12 && common.res != 14 && common.res != 16) {
		log_error("Resolution must be chosen from {12,14,16}\n");
		return -1;
	}

	return 0;
}


int main(int argc, char *argv[])
{
	oid_t root;

	if (parseArgs(argc, argv) < 0) {
		usage(argv[0]);
		return 1;
	}

	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0) {
		usleep(10000);
	}

	if (init() < 0) {
		log_error("Could not init driver.");
		return 2;
	}

	log_info("Initialized");

	msgLoop();

	/* Should never be reached */
	log_error("Exiting!");
	return 0;
}
