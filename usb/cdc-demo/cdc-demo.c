/*
 * Phoenix-RTOS
 *
 * cdc demo - Example of using: USB Communication Device Class
 *
 * Copyright 2019, 2021 Phoenix Systems
 * Author: Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <cdc_client.h>


#define TIME_ONESECOND         1000000000ULL
#define TIME_REALTIME_OVERFLOW (TIME_ONESECOND / 5)

#define USB_BUF_SIZE 0x1000

#define LOG(str, ...) \
	do { \
		if (1) \
			fprintf(stderr, "cdc-demo: " str "\n", ##__VA_ARGS__); \
	} while (0)
#define LOG_ERROR(str, ...) \
	do { \
		fprintf(stderr, __FILE__ ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); \
	} while (0)


static const char animHourGlass[] = { '/', '|', '\\', '-' };


typedef struct {
	struct {
		void *ptr;
		size_t size;
	} buf;

	struct {
		unsigned int steps;
		uint64_t current;
		uint64_t schedule;
		uint64_t start;
	} time;

	struct {
		size_t totaltransferred;
	} stat;

	struct {
		uint8_t isQuit : 1; /* TODO: attach to SIGQUIT signal  */
		uint8_t isConnected : 1;
		uint8_t canPushData : 1;
		uint8_t isDrop : 1;
	} flags;
} cdc_demo_ctx_t;


static uint64_t getTimeStamp(void)
{
	struct timespec t1;
	clock_gettime(CLOCK_MONOTONIC, &t1);

	return (uint64_t)t1.tv_sec * TIME_ONESECOND + t1.tv_nsec;
}


static void cdc_demoEventNotify(int evType, void *ctxUser)
{
	cdc_demo_ctx_t *ctx = (cdc_demo_ctx_t *)ctxUser;

	switch (evType) {
		case CDC_EV_RESET:
		case CDC_EV_DISCONNECT:
			ctx->flags.canPushData = 0;
			ctx->flags.isConnected = 0;
			break;

		case CDC_EV_CONNECT:
			ctx->flags.isConnected = 1;
			break;

		case CDC_EV_CARRIER_DEACTIVATE:
			ctx->flags.canPushData = 0;
			break;

		case CDC_EV_CARRIER_ACTIVATE:
			if (ctx->flags.isConnected) {
				ctx->stat.totaltransferred = 0;
				ctx->flags.isDrop = 1;
				ctx->flags.canPushData = 1;
			}
			break;
	}
}


static void cdc_demoProgress(cdc_demo_ctx_t *ctx)
{

	if (ctx->flags.canPushData) {
		uint32_t delta = ctx->time.current > ctx->time.start ? (ctx->time.current - ctx->time.start) / TIME_ONESECOND : ctx->stat.totaltransferred;

		ctx->time.schedule += TIME_ONESECOND;

		printf("\rTotal %u bytes transferred at %u kB/s\033[0J", ctx->stat.totaltransferred, ctx->stat.totaltransferred / (delta * 1000));
	}
	else {
		ctx->time.schedule += TIME_ONESECOND / 4;

		printf("\rIdle %c (%s host)\033[0J",
			animHourGlass[ctx->time.steps % sizeof(animHourGlass)], ctx->flags.isConnected ? "connected to" : "disconnected from");
	}

	fflush(stdout);
}


static void cdc_demoPepareData(cdc_demo_ctx_t *ctx)
{
	unsigned int n;
	uint8_t *buf = (uint8_t *)ctx->buf.ptr;

	if (ctx->flags.isDrop) {
		ctx->flags.isDrop = 0;
		/* send block of zeros - mark dropped real time data */
		bzero(ctx->buf.ptr, ctx->buf.size);
	}
	else {
		for (n = 0; n < ctx->buf.size; n++)
			*buf++ = ' ' + n % ('~' - ' ');
	}
}


static void cdc_demoSend(cdc_demo_ctx_t *ctx)
{
	int res = cdc_send(CDC_ENDPT_BULK, ctx->buf.ptr, ctx->buf.size);
	uint64_t delayed;

	if (res < 0) {
		LOG("cdc_send(): failed");
		ctx->flags.isDrop = 1;
		return;
	}

	delayed = getTimeStamp() - ctx->time.current;
	if (delayed > TIME_REALTIME_OVERFLOW) {
		LOG("cdc_send(): blocked for too long, dropping all realtime data!");
		ctx->flags.isDrop = 1;
		return;
	}

	ctx->stat.totaltransferred += res;
}


static void cdc_demoRecv(cdc_demo_ctx_t *ctx)
{
	int res = cdc_recv(CDC_ENDPT_BULK, ctx->buf.ptr, ctx->buf.size);

	if (res < 0) {
		LOG("cdc_recv(): failed");
		ctx->flags.isDrop = 1;
		return;
	}

	ctx->stat.totaltransferred += res;
}


int main(int argc, char **argv)
{
	cdc_demo_ctx_t ctx = { .buf = { .size = USB_BUF_SIZE } };
	int modeRecv = argc > 1 && (*argv[1] | 0x20) == 'r';

	/*
	 * pass r or R in program args to start receiving mode
	 */

	sleep(1);

	LOG("started.");

	if ((ctx.buf.ptr = malloc(ctx.buf.size)) == NULL) {
		LOG_ERROR("Out of memory.");
		return -1;
	}

	if (cdc_init(cdc_demoEventNotify, &ctx)) {
		free(ctx.buf.ptr);
		LOG_ERROR("couldn't initialize CDC transport.");
		return -2;
	}

	if (modeRecv)
		LOG("initialized in receiving mode");
	else
		LOG("initialized in sending mode");

	ctx.time.start = getTimeStamp();

	while (ctx.flags.isQuit == 0) {
		ctx.time.current = getTimeStamp();

		if (ctx.time.schedule < ctx.time.current) {
			ctx.time.steps++;
			cdc_demoProgress(&ctx);
		}

		if (ctx.flags.canPushData == 0) {
			/* idle */
			ctx.time.start = ctx.time.current;
			usleep(200 * 1000);
			continue;
		}

		if (modeRecv) {
			cdc_demoRecv(&ctx);
		}
		else {
			cdc_demoPepareData(&ctx);
			cdc_demoSend(&ctx);
		}

		/*
		 * the below usleep/yield is not necessary when speed of 22MB/s (~180Mbit/s)
		 * is required but it may flood hub and other devices like mouse, keyboard
		 * may freeze :), when usleep(1) is used data transfer drops to 6MB/s (~50Mbit),
		 * test it yourself (50MB/s ~400Mbit/s may be achieved by removing any hub
		 * in between device and PC, und usleep(1) below.
		 */
		usleep(1);
	}

	cdc_destroy();
	free(ctx.buf.ptr);

	return 0;
}
