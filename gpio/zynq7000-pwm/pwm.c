/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq 7000 PWM driver
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

#include "zynq7000-pwm-priv.h"

#define ROOTFS_WAIT 1
#define PRIORITY    2

#define RELOAD_DEFAULT 100000
#define PWM_NCHAN      8
#define PWM_RELOAD     PWM_NCHAN


typedef struct {
	oid_t oid;
	unsigned int reg;
} pwm_chan_t;


static struct {
	volatile uint32_t *base;
	pwm_chan_t channel[PWM_NCHAN];
} pwm_common;


static void pwm_read(pwm_chan_t *chan, uint32_t *val)
{
	*val = *(pwm_common.base + chan->reg);
}


static void pwm_write(pwm_chan_t *chan, uint32_t val)
{
	uint32_t reload = *(pwm_common.base + PWM_RELOAD);

	if (reload < val) {
		val = reload;
	}

	*(pwm_common.base + chan->reg) = val;
}


static void pwm_setReload(uint32_t reload)
{
	*(pwm_common.base + PWM_RELOAD) = reload;
}


static void pwm_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	uint32_t val;
	int ret;
	char buff[16];
	zynq7000pwm_imsg_t *iptr = (zynq7000pwm_imsg_t *)msg.i.raw;
	zynq7000pwm_omsg_t *optr = (zynq7000pwm_omsg_t *)msg.o.raw;
	size_t i;

	(void)arg;

	for (;;) {
		if (msgRecv(pwm_common.channel[0].oid.port, &msg, &rid) < 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				if (msg.i.io.oid.id >= PWM_NCHAN) {
					msg.o.io.err = -ENOENT;
				}
				else {
					msg.o.io.err = 0;
				}
				break;

			case mtRead:
				if (msg.i.io.oid.id >= PWM_NCHAN) {
					msg.o.io.err = -ENOENT;
				}
				else {
					pwm_read(&pwm_common.channel[msg.i.io.oid.id], &val);
					ret = sprintf(buff, "%u\n", val);

					if (ret < msg.i.io.offs) {
						/* EOF */
						msg.o.io.err = 0;
					}
					else {
						strncpy(msg.o.data, buff + msg.i.io.offs, msg.o.size);
						((char *)msg.o.data)[msg.o.size - 1] = '\0';
						msg.o.io.err = ((ret + 1) < (int)msg.o.size) ? ret + 1 : msg.o.size;
					}
				}
				break;

			case mtWrite:
				if (msg.i.io.oid.id >= PWM_NCHAN) {
					msg.o.io.err = -ENOENT;
				}
				else if (msg.i.data == NULL || msg.i.size == 0) {
					msg.o.io.err = -EINVAL;
				}
				else {
					strncpy(buff, msg.i.data, msg.i.size < sizeof(buff) ? msg.i.size : sizeof(buff));
					buff[sizeof(buff) - 1] = '\0';
					val = (uint32_t)strtoul(buff, NULL, 0);
					pwm_write(&pwm_common.channel[msg.i.io.oid.id], val);
					msg.o.io.err = (int)msg.i.size;
				}
				break;

			case mtSetAttr:
				msg.o.attr.err = -ENOSYS;
				break;


			case mtGetAttr:
				msg.o.attr.val = sizeof(buff);
				msg.o.attr.err = EOK;
				break;

			case mtDevCtl:
				if (iptr->type == zynq7000pwm_msgGet) {
					for (i = 0; i < sizeof(optr->compval) / sizeof(*optr->compval); ++i) {
						pwm_read(&pwm_common.channel[i], &optr->compval[i]);
					}
					ret = i;
				}
				else if (iptr->type == zynq7000pwm_msgSet) {
					ret = 0;
					for (i = 0; i < sizeof(iptr->compval) / sizeof(*iptr->compval); ++i) {
						if ((iptr->mask & (1 << i)) != 0) {
							pwm_write(&pwm_common.channel[i], iptr->compval[i]);
							++ret;
						}
					}
				}
				else {
					ret = -EINVAL;
				}

				optr->err = ret;
				break;

			default:
				msg.o.io.err = -ENOSYS;
				break;
		}

		msgRespond(pwm_common.channel[0].oid.port, &msg, rid);
	}
}


static void pwm_usage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("Options:\n");
	printf("\t-b base_addr - base address of PWM8X IP\n");
	printf("\t-r reload    - reload value (0-0xffffffff, default 0x%x)\n", RELOAD_DEFAULT);
	printf("\t-p priority  - server thread priority (default %d)\n", PRIORITY);
	printf("\t-h           - this message\n");
}


int main(int argc, char *argv[])
{
	int opt;
	uintptr_t baseaddr = 0;
	char dev[16];
	oid_t oid;
	uint32_t reload = RELOAD_DEFAULT;
	unsigned int i;
	int prio = PRIORITY;

	if (ROOTFS_WAIT != 0) {
		oid_t rootfs;
		int cnt = 0;

		while (lookup("/", &rootfs, NULL) < 0) {
			usleep(10 * 1000);
			++cnt;

			if (cnt > 10000) {
				fprintf(stderr, "pwm: rootfs wait timeout\n");
				return EXIT_FAILURE;
			}
		}
	}

	while ((opt = getopt(argc, argv, "b:r:h")) >= 0) {
		switch (opt) {
			case 'b':
				baseaddr = (uintptr_t)strtoul(optarg, NULL, 0);
				if (baseaddr == 0) {
					/* NULL is not a valid baseaddr anyway */
					fprintf(stderr, "pwm: failed to parse base address %s\n", optarg);
					return EXIT_FAILURE;
				}
				break;

			case 'r':
				/* Ignore conversion error */
				reload = (uint32_t)strtoul(optarg, NULL, 0);
				break;

			case 'p':
				/* Ignore conversion error */
				prio = (int)strtol(optarg, NULL, 0);
				break;

			case 'h':
				pwm_usage(argv[0]);
				return EXIT_SUCCESS;

			default:
				fprintf(stderr, "pwm: invalid option %c\n", opt);
				pwm_usage(argv[0]);
				return EXIT_FAILURE;
		}
	}

	if (baseaddr == 0) {
		fprintf(stderr, "pwm: no PWM IP base address specified\n");
		return EXIT_FAILURE;
	}

	pwm_common.base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, baseaddr);

	if (pwm_common.base == MAP_FAILED) {
		fprintf(stderr, "pwm: failed to mmap device\n");
		return EXIT_FAILURE;
	}

	pwm_setReload(reload);

	if (portCreate(&oid.port) < 0) {
		fprintf(stderr, "pwm: failed to create port\n");
		return EXIT_FAILURE;
	}

	for (i = 0; i < PWM_NCHAN; ++i) {
		pwm_common.channel[i].oid.port = oid.port;
		pwm_common.channel[i].oid.id = i;
		pwm_common.channel[i].reg = i;

		snprintf(dev, sizeof(dev), "pwm%u", i);
		if (create_dev(&pwm_common.channel[i].oid, dev) < 0) {
			fprintf(stderr, "pwm: failed to register device /dev/%s\n", dev);
			return EXIT_FAILURE;
		}
	}

	printf("pwm: initialized\n");

	priority(prio);
	pwm_thread(NULL);

	return EXIT_FAILURE;
}
