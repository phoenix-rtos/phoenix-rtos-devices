/*
 * Phoenix-RTOS
 *
 * i.MX RT1064 AD7779 driver.
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Krystian Wasik, Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/threads.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <posix/utils.h>
#include <board_config.h>

#include <phoenix/arch/imxrt.h>

#include "ad7779.h"
#include "adc-api-ad7779.h"

#define MAX_INIT_TRIES              (4)

#ifndef AD7779_PRIO
#define AD7779_PRIO 4
#endif


static struct {
	uint32_t port;
	int enabled;
	addr_t buffer_paddr;
} ad7779_common;


static int dev_init(void)
{
	int res;
	oid_t dev;

	res = portCreate(&ad7779_common.port);
	if (res != EOK) {
		log_error("could not create port: %d", res);
		return -1;
	}

	dev.id = 0;
	dev.port = ad7779_common.port;

	res = create_dev(&dev, "ad7779");
	if (res != EOK) {
		log_error("could not create device ad7779");
		return -1;
	}

	return 0;
}

static int dev_open(oid_t *oid, int flags)
{
	(void)oid;
	(void)flags;

	return EOK;
}

static int dev_close(oid_t *oid, int flags)
{
	(void)oid;
	(void)flags;

	return EOK;
}

static int dev_read(void *data, size_t size)
{
	return dma_read(data, size);
}

static int dev_ctl(msg_t *msg)
{
	int res;
	adc_dev_ctl_t dev_ctl;

	memcpy(&dev_ctl, msg->o.raw, sizeof(adc_dev_ctl_t));

	switch (dev_ctl.type) {
		case adc_dev_ctl__enable:
			ad7779_common.enabled = 1;
			sai_rx_enable();
			dma_enable();
			return EOK;

		case adc_dev_ctl__disable:
			dma_disable();
			sai_rx_disable();
			ad7779_common.enabled = 0;
			return EOK;

		case adc_dev_ctl__reset:
			return ad7779_init(dev_ctl.reset_hard);

		case adc_dev_ctl__status:
			res = ad7779_get_status((uint8_t *)&dev_ctl.status);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__set_adc_mux:
			res = ad7779_set_adc_mux((dev_ctl.mux >> 6),
				(dev_ctl.mux >> 2) & 0b1111);
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__set_config:
			if (ad7779_common.enabled)
				return -EBUSY;
			res = ad7779_set_sampling_rate(dev_ctl.config.sampling_rate);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			res = ad7779_set_enabled_channels(dev_ctl.config.enabled_ch);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__get_config:
			res = ad7779_get_sampling_rate(&dev_ctl.config.sampling_rate);
			if (res != AD7779_OK)
				return -EIO;
			res = ad7779_get_enabled_channels(&dev_ctl.config.enabled_ch);
			dev_ctl.config.enabled_ch = ~dev_ctl.config.enabled_ch;
			if (res != AD7779_OK)
				return -EIO;
			dev_ctl.config.channels = AD7779_NUM_OF_CHANNELS;
			dev_ctl.config.bits = AD7779_NUM_OF_BITS;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__get_buffers:
			dev_ctl.buffers.paddr = ad7779_common.buffer_paddr;
			dev_ctl.buffers.num = AD7779_BUFFER_CNT;
			dev_ctl.buffers.size = AD7779_BUFFER_SIZE * AD7779_BUFFER_CNT;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__set_channel_config:
		{
			res = ad7779_set_channel_gain(dev_ctl.ch_config.channel,
				dev_ctl.ch_config.gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;

			ad7779_chmode_t mode = dev_ctl.ch_config.ref_monitor_mode +
				dev_ctl.ch_config.meter_rx_mode;

			res = ad7779_set_channel_mode(dev_ctl.ch_config.channel, mode);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;
		}

		case adc_dev_ctl__get_channel_config:
		{
			uint8_t gain;
			res = ad7779_get_channel_gain(dev_ctl.ch_config.channel, &gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;

			dev_ctl.ch_config.gain = gain;

			ad7779_chmode_t mode;
			res = ad7779_get_channel_mode(dev_ctl.ch_config.channel, &mode);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;

			dev_ctl.ch_config.meter_rx_mode = 0;
			dev_ctl.ch_config.ref_monitor_mode = 0;
			if (mode == ad7779_chmode__ref_monitor)
				dev_ctl.ch_config.ref_monitor_mode = 1;
			else if (mode == ad7779_chmode__meter_rx)
				dev_ctl.ch_config.meter_rx_mode = 1;

			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;
		}

		case adc_dev_ctl__set_channel_gain:
			res = ad7779_set_channel_gain(dev_ctl.gain.channel, dev_ctl.gain.val);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__get_channel_gain:
			res = ad7779_get_channel_gain(dev_ctl.gain.channel, &dev_ctl.gain.val);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__set_channel_calib:
			res = ad7779_set_channel_offset(dev_ctl.calib.channel, dev_ctl.calib.offset);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			res = ad7779_set_channel_gain_correction(dev_ctl.calib.channel, dev_ctl.calib.gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__get_channel_calib:
			res = ad7779_get_channel_offset(dev_ctl.calib.channel, &dev_ctl.calib.offset);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			res = ad7779_get_channel_gain_correction(dev_ctl.calib.channel, &dev_ctl.calib.gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		default:
			log_error("dev_ctl: unknown type (%d)", dev_ctl.type);
			return -ENOSYS;
	}

	return EOK;
}

static void msg_loop(void)
{
	msg_t msg;
	unsigned long rid;

	while (1) {
		if (msgRecv(ad7779_common.port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
			case mtOpen:
				msg.o.io.err = dev_open(&msg.i.openclose.oid, msg.i.openclose.flags);
				break;

			case mtClose:
				msg.o.io.err = dev_close(&msg.i.openclose.oid, msg.i.openclose.flags);
				break;

			case mtRead:
				msg.o.io.err = dev_read(msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = -ENOSYS;
				break;

			case mtDevCtl:
				msg.o.io.err = dev_ctl(&msg);
				break;
		}

		msgRespond(ad7779_common.port, &msg, rid);
	}
}

static int init(void)
{
	int res;

	ad7779_common.enabled = 0;

	if ((res = sai_init()) < 0) {
		log_error("failed to initialize sai");
		return res;
	}

	if ((res = dma_init(AD7779_BUFFER_SIZE, AD7779_BUFFER_CNT, &ad7779_common.buffer_paddr)) != 0) {
		log_error("failed to initialize dma");
		sai_free();
		return res;
	}

	if ((res = spi_init()) < 0) {
		log_error("failed to initialize spi");
		sai_free();
		dma_free();
		return res;
	}

	if ((res = ad7779_gpio_init()) < 0) {
		log_error("failed to initialize gpio");
		sai_free();
		dma_free();
		return res;
	}

	if ((res = ad7779_init(0)) < 0) {
		log_error("failed to initialize ad7779 (%d)", res);
		sai_free();
		dma_free();
		return res;
	}

	if ((res = dev_init()) < 0) {
		log_error("device initialization failed");
		sai_free();
		dma_free();
	}

	return res;
}

static int parse_args(int argc, char *argv[])
{
	int opt;
	unsigned long prio;
	char *endptr;

	while ((opt = getopt(argc, argv, "p:")) != -1) {
		switch (opt) {
			case 'p':
				prio = strtoul(optarg, &endptr, 0);
				if (*endptr != '\0') {
					printf("%s: incorrect priority value (%s)\n", argv[0], optarg);
					return -1;
				}

				if (prio > 7) {
					printf("%s: incorrect priority value (%lu). It must be in [0,7] range\n",
						argv[0], prio);
					return -1;
				}

				priority(prio);
				break;

			default:
				printf("AD7779 driver. Usage:\n");
				printf("%s [-p priority]\n", argv[0]);
				printf("\t-p priority\t\tSelect priority for the AD7779 driver\n");
				return -1;
		}
	}

	return 0;
}

int main(int argc, char *argv[])
{
	int i;
	oid_t root;

	priority(AD7779_PRIO);

	if (parse_args(argc, argv) < 0)
		return 1;

	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0)
		usleep(10000);

	for (i = 0; i < MAX_INIT_TRIES; ++i) {
		if (!init())
			break;

		usleep(100000);
	}

	if (i == MAX_INIT_TRIES) {
		log_error("could not init driver.");
		return -EIO;
	}

	log_info("initialized");

	msg_loop();

	/* Should never be reached */
	log_error("Exiting!");
	return 0;
}
