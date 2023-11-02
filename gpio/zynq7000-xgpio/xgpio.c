/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq 7000 XGPIO driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <posix/idtree.h>
#include <posix/utils.h>
#include <sys/mman.h>

#define ROOTFS_WAIT 1
#define PRIORITY    2

/* Registers */
#define GPIO1_DATA 0
#define GPIO1_DIR  1
#define GPIO2_DATA 2
#define GPIO2_DIR  3


typedef struct {
	int channel;
	int isDir;
	int isPin;
	int pin;
} gpio_info_t;


static struct {
	oid_t oid;
	volatile uint32_t *base;
} gpio_common;


static void gpio_encodeOid(const gpio_info_t *info, oid_t *oid)
{
	oid->id = info->channel << 7;

	if (info->isPin != 0) {
		oid->id |= (1 << 6) | (info->pin & 0x1f);
	}

	if (info->isDir != 0) {
		oid->id |= 1 << 5;
	}
}


static void gpio_decodeOid(gpio_info_t *info, const oid_t *oid)
{
	info->channel = (oid->id >> 7) & 1;
	info->isPin = (oid->id >> 6) & 1;
	info->isDir = (oid->id >> 5) & 1;
	info->pin = oid->id & 0x1f;
}


static int gpio_writePort(int channel, uint32_t val, uint32_t mask)
{
	int ret = 0;
	uint32_t t;

	if (channel == 0) {
		t = *(gpio_common.base + GPIO1_DATA);
		*(gpio_common.base + GPIO1_DATA) = (val & mask) | (t & ~mask);
	}
	else if (channel == 1) {
		t = *(gpio_common.base + GPIO2_DATA);
		*(gpio_common.base + GPIO2_DATA) = (val & mask) | (t & ~mask);
	}
	else {
		ret = -EINVAL;
	}

	return ret;
}


static int gpio_readPort(int channel, uint32_t *val)
{
	int ret = 0;

	if (channel == 0) {
		*val = *(gpio_common.base + GPIO1_DATA);
	}
	else if (channel == 1) {
		*val = *(gpio_common.base + GPIO2_DATA);
	}
	else {
		ret = -EINVAL;
	}

	return ret;
}


static int gpio_writeDir(int channel, uint32_t dir)
{
	int ret = 0;

	if (channel == 0) {
		*(gpio_common.base + GPIO1_DIR) = ~dir;
	}
	else if (channel == 1) {
		*(gpio_common.base + GPIO2_DIR) = ~dir;
	}
	else {
		ret = -EINVAL;
	}

	return ret;
}


static int gpio_readDir(int channel, uint32_t *dir)
{
	int ret = 0;

	if (channel == 0) {
		*dir = ~(*(gpio_common.base + GPIO1_DIR));
	}
	else if (channel == 1) {
		*dir = ~(*(gpio_common.base + GPIO2_DIR));
	}
	else {
		ret = -EINVAL;
	}

	return ret;
}


static int gpio_writePin(int channel, int pin, uint32_t val)
{
	int ret = -EINVAL;

	if (channel >= 0 && channel <= 1 && pin >= 0 && pin <= 31) {
		ret = gpio_writePort(channel, val << pin, 1U << pin);
	}

	return ret;
}


static int gpio_readPin(int channel, int pin, uint32_t *val)
{
	int ret = -EINVAL;
	uint32_t t;

	if (channel >= 0 && channel <= 1 && pin >= 0 && pin <= 31) {
		ret = gpio_readPort(channel, &t);
		*val = (t & (1U << pin)) == 0 ? 0 : 1;
	}

	return ret;
}


static void gpio_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	uint32_t val;
	int ret;
	char buff[16];
	gpio_info_t info;

	(void)arg;

	for (;;) {
		if (msgRecv(gpio_common.oid.port, &msg, &rid) < 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = 0;
				break;

			case mtRead:
				gpio_decodeOid(&info, &msg.i.io.oid);

				if (info.isDir != 0) {
					ret = gpio_readDir(info.channel, &val);
				}
				else if (info.isPin != 0) {
					ret = gpio_readPin(info.channel, info.pin, &val);
				}
				else {
					ret = gpio_readPort(info.channel, &val);
				}

				if (ret < 0) {
					msg.o.io.err = ret;
				}
				else if (msg.o.data == NULL || msg.o.size == 0) {
					msg.o.io.err = 0;
				}
				else {
					ret = sprintf(buff, "%u\n", val);
					++ret;
					if (ret <= msg.i.io.offs) {
						/* EOF */
						msg.o.io.err = 0;
					}
					else {
						strncpy(msg.o.data, buff + msg.i.io.offs, msg.o.size);
						((char *)msg.o.data)[msg.o.size - 1] = '\0';
						ret -= msg.i.io.offs;
						msg.o.io.err = (ret < (int)msg.o.size) ? ret : msg.o.size;
					}
				}
				break;

			case mtWrite:
				if (msg.i.data == NULL || msg.i.size == 0) {
					msg.o.io.err = 0;
				}
				else {
					strncpy(buff, msg.i.data, msg.i.size < sizeof(buff) ? msg.i.size : sizeof(buff));
					buff[sizeof(buff) - 1] = '\0';
					val = (uint32_t)strtoul(buff, NULL, 0);

					gpio_decodeOid(&info, &msg.i.io.oid);

					if (info.isDir != 0) {
						ret = gpio_writeDir(info.channel, val);
					}
					else if (info.isPin != 0) {
						ret = gpio_writePin(info.channel, info.pin, val);
					}
					else {
						ret = gpio_writePort(info.channel, val, 0xffffffffUL);
					}

					if (ret < 0) {
						msg.o.io.err = ret;
					}
					else {
						msg.o.io.err = (int)msg.i.size;
					}
				}
				break;

			case mtSetAttr:
			case mtGetAttr:
				msg.o.attr.err = -ENOSYS;
				break;

			default:
				msg.o.io.err = -ENOSYS;
				break;
		}

		msgRespond(gpio_common.oid.port, &msg, rid);
	}
}


static void gpio_usage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("Options:\n");
	printf("\t-b base_addr - base address of XGPIO IP\n");
	printf("\t-p priority  - server thread priority (default %d)\n", PRIORITY);
	printf("\t-h           - this message\n");
}


int main(int argc, char *argv[])
{
	int opt;
	uintptr_t baseaddr = 0;
	char dev[16];
	oid_t oid;
	gpio_info_t info;
	int prio = PRIORITY;

	if (ROOTFS_WAIT != 0) {
		oid_t rootfs;
		int cnt = 0;

		while (lookup("/", &rootfs, NULL) < 0) {
			usleep(10 * 1000);
			++cnt;

			if (cnt > 10000) {
				fprintf(stderr, "xgpio: rootfs wait timeout\n");
				return EXIT_FAILURE;
			}
		}
	}

	while ((opt = getopt(argc, argv, "b:p:h")) >= 0) {
		switch (opt) {
			case 'b':
				baseaddr = (uintptr_t)strtoul(optarg, NULL, 0);
				if (baseaddr == 0) {
					/* NULL is not a valid baseaddr anyway */
					fprintf(stderr, "xgpio: failed to parse base address %s\n", optarg);
					return EXIT_FAILURE;
				}
				break;

			case 'p':
				/* Ignore conversion errors */
				prio = (int)strtol(optarg, NULL, 0);
				break;

			case 'h':
				gpio_usage(argv[0]);
				return EXIT_SUCCESS;

			default:
				fprintf(stderr, "xgpio: invalid option %c\n", opt);
				gpio_usage(argv[0]);
				return EXIT_FAILURE;
		}
	}

	if (baseaddr == 0) {
		fprintf(stderr, "xgpio: no XGPIO IP base address specified\n");
		return EXIT_FAILURE;
	}

	gpio_common.base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, baseaddr);

	if (gpio_common.base == MAP_FAILED) {
		fprintf(stderr, "xgpio: failed to mmap device\n");
		return EXIT_FAILURE;
	}

	if (portCreate(&oid.port) < 0) {
		fprintf(stderr, "xgpio: failed to create port\n");
		return EXIT_FAILURE;
	}

	gpio_common.oid.port = oid.port;

	for (info.channel = 0; info.channel < 2; ++info.channel) {
		info.isPin = 0;
		info.isDir = 0;
		info.pin = 0;

		gpio_encodeOid(&info, &oid);
		snprintf(dev, sizeof(dev), "xgpio%d/port", info.channel);
		if (create_dev(&oid, dev) < 0) {
			fprintf(stderr, "xgpio: failed to register device %s\n", dev);
			return EXIT_FAILURE;
		}

		info.isDir = 1;
		gpio_encodeOid(&info, &oid);
		snprintf(dev, sizeof(dev), "xgpio%d/dir", info.channel);
		if (create_dev(&oid, dev) < 0) {
			fprintf(stderr, "xgpio: failed to register device %s\n", dev);
			return EXIT_FAILURE;
		}

		info.isDir = 0;
		info.isPin = 1;

		for (info.pin = 0; info.pin < 32; ++info.pin) {
			gpio_encodeOid(&info, &oid);
			snprintf(dev, sizeof(dev), "xgpio%d/pin%d", info.channel, info.pin);
			if (create_dev(&oid, dev) < 0) {
				fprintf(stderr, "xgpio: failed to register device /dev/%s\n", dev);
				return EXIT_FAILURE;
			}
		}
	}

	printf("xgpio: initialized\n");

	priority(prio);
	gpio_thread(NULL);

	return EXIT_FAILURE;
}
