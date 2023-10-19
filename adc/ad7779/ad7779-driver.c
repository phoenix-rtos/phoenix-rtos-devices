/*
 * Phoenix-RTOS
 *
 * i.MX RT1064 / i.MX 6ULL AD7779 driver.
 *
 * Copyright 2018, 2020, 2023 Phoenix Systems
 * Author: Krystian Wasik, Marcin Baran, Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

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


/* mtRead message is the only blocking one, handling it asynchronously makes it possible
 * to support multiple clients (readers).
 * To be defined in board_config.h if needed */
/* #define AD7779_SUPPORT_ASYNC_REQS */

#define MAX_PENDING_REQS 4
#define READ_SIZE        (sizeof(uint32_t))


enum asyncState {
	stUnused = 0,
	stUsed,
	stPendingRead
};


typedef struct {
	volatile sig_atomic_t state; /* one of asyncState, access is thread-safe */
	msg_t msg;
	msg_rid_t rid;
} async_req_t;


static struct {
	uint32_t port;
	volatile sig_atomic_t enabled;
	unsigned long prio;
	addr_t buffer_paddr;
#ifdef AD7779_SUPPORT_ASYNC_REQS
	struct {
		handle_t lock;
		handle_t cond;
		async_req_t pending[MAX_PENDING_REQS];
		uint8_t stack[2048] __attribute__((aligned(8)));
	} async;
#endif
} ad7779_common;


static async_req_t *allocReq(void)
{
	async_req_t *ret = NULL;

#ifdef AD7779_SUPPORT_ASYNC_REQS
	for (int i = 0; i < MAX_PENDING_REQS; ++i) {
		/* NOTE: no need for mutex (allocReq called only by one thread), access to `state` is thread-safe */
		if (ad7779_common.async.pending[i].state == stUnused) {
			ad7779_common.async.pending[i].state = stUsed;
			ret = &ad7779_common.async.pending[i];
			break;
		}
	}
#else
	/* all requests are blocking/synchronous - use single req */
	static async_req_t req_storage;
	ret = &req_storage;
#endif /* AD7779_SUPPORT_ASYNC_REQS */

	return ret;
}


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


static int dev_read(async_req_t *req, int *respond)
{
#ifdef AD7779_SUPPORT_ASYNC_REQS
	*respond = 1;

	if (ad7779_common.enabled != 1) {
		log_error("trying to read when not enabled");
		return -EIO;
	}

	if ((req->msg.o.data != NULL) && (req->msg.o.size != READ_SIZE)) {
		return -EIO;
	}

	/* NOTE: mutex not needed for data races, but ensures no reader would be added while servicing single read */
	mutexLock(ad7779_common.async.lock);

	/* schedule async read */
	req->state = stPendingRead;
	*respond = 0;

	condSignal(ad7779_common.async.cond);
	mutexUnlock(ad7779_common.async.lock);

	return EOK;
#else
	return dma_read(req->msg.o.data, req->msg.o.size);
#endif /* AD7779_SUPPORT_ASYNC_REQS */
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
			/* NOTE: it's not guaranteed that dma_read() would ever finish (read_thr may be blocked until next enable or timeout) */
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

#ifdef AD7779_SUPPORT_ASYNC_REQS
static void read_thr(void *arg)
{
	uint32_t data;

	mutexLock(ad7779_common.async.lock);
	for (;;) {
		if (condWait(ad7779_common.async.cond, ad7779_common.async.lock, 0) != 0) {
			continue;
		}

		if (ad7779_common.enabled != 1) {
			continue;
		}

		mutexUnlock(ad7779_common.async.lock);
		int ret = dma_read(&data, READ_SIZE);

		/* NOTE: mutex not needed for data races, but ensures no reader would be added while servicing single read
		 * prevents interrupt double-read by the same process */
		mutexLock(ad7779_common.async.lock);

		for (int i = 0; i < MAX_PENDING_REQS; ++i) {
			async_req_t *req = &ad7779_common.async.pending[i];
			if (req->state == stPendingRead) {
				req->msg.o.io.err = ret;
				if ((req->msg.o.data != NULL) && (ret == EOK)) {
					memcpy(req->msg.o.data, &data, READ_SIZE);
				}

				msgRespond(ad7779_common.port, &req->msg, req->rid);
				req->state = stUnused;
			}
		}
	}

	endthread();
}
#endif

static void msg_loop(void)
{
	async_req_t *req = NULL;

	while (1) {
		if (req == NULL) {
			req = allocReq();
			if (req == NULL) {
				log_error("no memory for handling new request");
				usleep(100); /* assume there are some pending read requests which will finish soon */
				continue;
			}
		}

		/* NOTE: it's forbidden to memcpy msg_t (eg o.data may point to o.raw), we need to keep full message for async requests to work */
		msg_t *msg = &req->msg;
		if (msgRecv(ad7779_common.port, msg, &req->rid) < 0) {
			continue;
		}

		int respond = 1;

		switch (msg->type) {
			case mtOpen:
				msg->o.io.err = dev_open(&msg->i.openclose.oid, msg->i.openclose.flags);
				break;

			case mtClose:
				msg->o.io.err = dev_close(&msg->i.openclose.oid, msg->i.openclose.flags);
				break;

			case mtRead:
				/* FIXME: read should return number of bytes read (now returns 0 on success */
				msg->o.io.err = dev_read(req, &respond);
				break;

			case mtWrite:
				msg->o.io.err = -ENOSYS;
				break;

			case mtDevCtl:
				msg->o.io.err = dev_ctl(msg);
				break;

			default:
				msg->o.io.err = -ENOSYS;
				break;
		}

		if (respond == 1) {
			msgRespond(ad7779_common.port, msg, req->rid);
			/* safe to reuse current `req` for next request */
		}
		else {
			/* current req is pending - we need to alloc new req */
			req = NULL;
		}
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
	ad7779_common.prio = AD7779_PRIO;
	char *endptr;

	while ((opt = getopt(argc, argv, "p:")) != -1) {
		switch (opt) {
			case 'p':
				ad7779_common.prio = strtoul(optarg, &endptr, 0);
				if (*endptr != '\0') {
					printf("%s: incorrect priority value (%s)\n", argv[0], optarg);
					return -1;
				}

				if (ad7779_common.prio > 7) {
					printf("%s: incorrect priority value (%lu). It must be in [0,7] range\n",
						argv[0], ad7779_common.prio);
					return -1;
				}

				priority(ad7779_common.prio);
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
		return 1;
	}

#ifdef AD7779_SUPPORT_ASYNC_REQS
	if (mutexCreate(&ad7779_common.async.lock) != EOK) {
		log_error("mutex resource creation failed");
		return 1;
	}
	if (condCreate(&ad7779_common.async.cond) != EOK) {
		log_error("conditional resource creation failed");
		return 1;
	}

	if (beginthread(read_thr, ad7779_common.prio, ad7779_common.async.stack, sizeof(ad7779_common.async.stack), NULL) < 0) {
		log_error("read_thr startup failed");
		return 1;
	}
#endif

	log_info("initialized");

	msg_loop();

	/* Should never be reached */
	log_error("Exiting!");
	return 0;
}
