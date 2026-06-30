/*
 * Phoenix-RTOS
 *
 * ADE9113 driver DMA for IMX6ULL
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <signal.h>
#include <assert.h>

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/types.h>
#include <sys/interrupt.h>

#include <phoenix/arch/armv7a/imx6ull/imx6ull.h>
#include <sdma.h>

#define LOG_TAG "ADE9113(DMA): "

#include "log.h"

#define SDMA_FILE_CH_TX "/dev/sdma/ch07"
#define SDMA_FILE_CH_RX "/dev/sdma/ch08"

#define IMX6ULL_SDMA_EVENT_ECSPI1_RX (3)
#define IMX6ULL_SDMA_EVENT_ECSPI1_TX (4)
#define IMX6ULL_ECSPI1_BASE          (0x02008000U)

#define SDMA_ECSPI_BASE IMX6ULL_ECSPI1_BASE
#define SDMA_EVENT_RX   IMX6ULL_SDMA_EVENT_ECSPI1_RX
#define SDMA_EVENT_TX   IMX6ULL_SDMA_EVENT_ECSPI1_TX

#define IRQ_BASE (32)
#define IRQ_SDMA (IRQ_BASE + 2)

#define ADE9113_CHAIN_LEN   4
#define ADE9113_REQ_SIZE    16
#define SDMA_TX_DESCRIPTORS 2

struct sdmaBuffer {
	size_t size;
	size_t count;
	addr_t phys_addr;
	void *ptr;
};


struct sdmaChannel {
	sdma_t sdma;
	struct sdmaBuffer buffer;
	struct sdmaBuffer bd;
};


struct sdmaCtx {
	volatile sig_atomic_t enabled;
	struct sdmaChannel tx;
	struct sdmaChannel rx;
	handle_t rxCond;
	handle_t rxLock;
	handle_t rxThreadId;
	bool (*rxCb)(const uint8_t *data, size_t size);
};


struct sdmaCtx sdma_common;


static int sdmaBufferAlloc(sdma_t *sdma, struct sdmaBuffer *buffer, size_t size, size_t count)
{
	*buffer = (struct sdmaBuffer) {
		.size = size,
		.count = count,
		.phys_addr = 0,
		.ptr = NULL
	};
	buffer->ptr = sdma_alloc_uncached(sdma, size * count, &buffer->phys_addr, 1);
	return (buffer->ptr != NULL) ? 0 : -1;
}


static void sdmaBufferFree(sdma_t *sdma, struct sdmaBuffer *buffer)
{
	if (buffer->ptr == NULL) {
		return;
	}
	sdma_free_uncached(buffer->ptr, buffer->count * buffer->size);
	*buffer = (struct sdmaBuffer) {
		.size = 0,
		.count = 0,
		.phys_addr = 0,
		.ptr = NULL
	};
}


static void resetDescriptor(struct sdmaChannel *ch, size_t idx)
{
	if (idx >= ch->bd.count) {
		return;
	}
	bool isLast = idx + 1 == ch->bd.count;
	volatile sdma_buffer_desc_t *bd = (sdma_buffer_desc_t *)ch->bd.ptr;
	/* TODO: check this line can be skipped when handingIrq */
	bd[idx] = (struct sdma_buffer_desc_s) {
		.count = ch->buffer.size,
		.flags = SDMA_BD_CONT | SDMA_BD_INTR | (isLast ? SDMA_BD_WRAP : 0),
		.command = SDMA_CMD_MODE_32_BIT,
		/* buf->count is a special case for tx that reuses single buffer */
		.buffer_addr = ch->buffer.phys_addr + (ch->buffer.count > 1 ? ch->buffer.size * idx : 0)
	};
	bd[idx].flags |= SDMA_BD_DONE;
}


static int handleIrq(unsigned int irqNum, void *voidCtx)
{
	struct sdmaCtx *ctx = (struct sdmaCtx *)voidCtx;
	bool wakeupThread = false;
	if (ctx->enabled != 0) {
		struct sdmaChannel *ch = &ctx->tx;
		volatile sdma_buffer_desc_t *bd = ch->bd.ptr;
		for (size_t i = 0; i < ch->bd.count; ++i) {
			if ((bd[i].flags & SDMA_BD_DONE) != 0) {
				continue; /* descriptor still owned by SDMA */
			}
			/* TX data is fixed so we can immediately give BD owenership back to SDMA (set BD_DONE flag) */
			resetDescriptor(ch, i);
		}
	}

	if (ctx->enabled != 0) {
		struct sdmaChannel *ch = &ctx->rx;
		volatile sdma_buffer_desc_t *bd = ch->bd.ptr;
		for (size_t i = 0; i < ch->bd.count; ++i) {
			if ((bd[i].flags & SDMA_BD_DONE) != 0) {
				continue; /* descriptor still owned by SDMA */
			}
			/* thread will consume data and set BD_DONE flag */
			wakeupThread = true;
			break;
		}
	}
	return wakeupThread ? 0 : -1;
}


static void handleThread(void *voidCtx)
{
	struct sdmaCtx *ctx = (struct sdmaCtx *)voidCtx;
	mutexLock(ctx->rxLock);
	while (ctx->enabled != 0) {
		/* TODO: add order check (should alternate between both descriptors) */
		struct sdmaChannel *ch = &ctx->rx;
		volatile sdma_buffer_desc_t *bd = ch->bd.ptr;
		for (size_t i = 0; i < ch->bd.count; ++i) {
			if ((bd[i].flags & SDMA_BD_DONE) != 0) {
				continue; /* descriptor still owned by SDMA */
			}
			bool keepGoing = true;
			if (ctx->rxCb != NULL) {
				const uint8_t *data = (const uint8_t *)ch->buffer.ptr + (ch->buffer.size * i);
				keepGoing = ctx->rxCb(data, ch->buffer.size);
			}
			if (keepGoing) {
				resetDescriptor(ch, i);
			}
		}
		condWait(ctx->rxCond, ctx->rxLock, 0);
	}
	mutexUnlock(ctx->rxLock);
	endthread();
}


struct channelConfig {
	sdma_script_t script;
	uint8_t event;
	unsigned priority;
	addr_t fifoAddr;
	unsigned fifoWatermark;
};


static void configureChannel(struct sdmaChannel *ch, const struct channelConfig *cfg)
{
	for (int i = 0; i < ch->bd.count; ++i) {
		resetDescriptor(ch, i);
	}
	sdma_context_t sdma_context;
	sdma_context_init(&sdma_context);
	sdma_context_set_pc(&sdma_context, cfg->script);
	sdma_context.gr[0] = (cfg->event < 32) ? 0 : 1 << (cfg->event - 32); /* Event2_mask */
	sdma_context.gr[1] = (cfg->event < 32) ? (1 << cfg->event) : 0;
	sdma_context.gr[6] = cfg->fifoAddr;
	sdma_context.gr[7] = cfg->fifoWatermark;
	sdma_context_set(&ch->sdma, &sdma_context);

	sdma_channel_configure(
			&ch->sdma,
			&(sdma_channel_config_t) {
				.bd_paddr = ch->bd.phys_addr,
				.bd_cnt = ch->bd.count,
				.trig = sdma_trig__event,
				.event = cfg->event,
				.priority = cfg->priority,
				.options = 0 });
}


static void sdmaConfigure(struct sdmaCtx *ctx)
{
	struct sdmaChannel *ch = &ctx->tx;
	{
		/*
		 * Fill request buffer with identical read requests. Long responses always include sample data so we can read
		 * any register. Using SCRATCH register to allow to differentiate responses from each device.
		 *
		 * Order of bytes is reversed in 4 byte words. DMA/SPI will send this request in following order:
		 * req[3], req[2], req[1], req[0], req[7], req[6], ...
		 */
		const uint8_t adcRequest[ADE9113_REQ_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x00, 0x13, 0xc0 };
		for (size_t offs = 0; offs + ADE9113_REQ_SIZE <= ch->buffer.size; offs += ADE9113_REQ_SIZE) {
			memcpy((uint8_t *)ch->buffer.ptr + offs, adcRequest, ADE9113_REQ_SIZE);
		}
	}
	/*
	 * NOTE: "AP" scripts are used here as there were issues observed with "SHP" scripts.
	 *
	 * When shp_2_mcu was used and main processor was interacting with other SPBA peripherals (ie. UART) random RX
	 * words were lost. It looked like some read instructions (SDMA `ld` instruction) on RX FIFO were executed twice
	 * resulting in one FIFO element being silently lost. Similar TX issues were observed with mcu_2_shp.
	 *
	 * It is not clear at this point if this is caused by configuration error (ie. clock setup) or if this is inherent
	 * platform / hw issue. "AP" scripts work good enough for this use-case.
	 *
	 * This post describes similar issue:
	 * https://community.nxp.com/t5/i-MX-Processors/SDMA-data-corrupt-under-CPU-load/td-p/1571766
	 */
	configureChannel(
			&ctx->rx,
			&(const struct channelConfig) {
				.event = SDMA_EVENT_RX,
				.script = sdma_script__ap_2_mcu,
				.fifoAddr = SDMA_ECSPI_BASE + 0,                       /* eCSPI->RXDATA */
				.fifoWatermark = ADE9113_REQ_SIZE * ADE9113_CHAIN_LEN, /* The WML is always given in bytes */
				.priority = SDMA_CHANNEL_PRIORITY_MIN + 1 });
	configureChannel(
			&ctx->tx,
			&(const struct channelConfig) {
				.event = SDMA_EVENT_TX,
				.script = sdma_script__mcu_2_ap,
				.fifoAddr = SDMA_ECSPI_BASE + 4,                       /* eCSPI->TXDATA */
				.fifoWatermark = ADE9113_REQ_SIZE * ADE9113_CHAIN_LEN, /* The WML is always given in bytes */
				.priority = SDMA_CHANNEL_PRIORITY_MIN + 1 });
}


static int sdmaInit(struct sdmaCtx *const ctx, size_t size, size_t count)
{
	if ((size % _PAGE_SIZE) != 0) {
		log_error("buffer size is not aligned to %d", _PAGE_SIZE);
		return -1;
	}

	interrupt(IRQ_SDMA, handleIrq, ctx, ctx->rxCond, 0);

	const char *files[] = { SDMA_FILE_CH_RX, SDMA_FILE_CH_TX };
	struct sdmaChannel *channels[] = { &ctx->rx, &ctx->tx };
	for (int i = 0; i < 2; ++i) {
		unsigned tries = 25;
		while (sdma_open(&channels[i]->sdma, files[i]) < 0) {
			usleep(100 * 1000);
			if (--tries == 0) {
				log_error("failed to open SDMA device file (%s)", files[i]);
				return -1;
			}
		}
	}

	/*
	 * NOTE: OCRAM usage could be optimized here by using single allocation for descriptors of both channels.
	 * It is would be little bit hacky in current API as allocations are on per channel bases.
	 */

	if (sdmaBufferAlloc(&ctx->tx.sdma, &ctx->tx.buffer, _PAGE_SIZE * 4, 1) < 0) {
		return -1;
	}
	if (sdmaBufferAlloc(&ctx->tx.sdma, &ctx->tx.bd, sizeof(sdma_buffer_desc_t), 2) < 0) {
		return -1;
	}

	if (sdmaBufferAlloc(&ctx->rx.sdma, &ctx->rx.buffer, size, count) < 0) {
		return -1;
	}
	if (sdmaBufferAlloc(&ctx->rx.sdma, &ctx->rx.bd, sizeof(sdma_buffer_desc_t), count) < 0) {
		return -1;
	}

	sdmaConfigure(ctx);
	return 0;
}


static int sdmaReset(struct sdmaCtx *ctx)
{
	sdmaConfigure(ctx);
	return 0;
}


static void sdmaFree(struct sdmaCtx *const ctx)
{
	sdmaBufferFree(&ctx->tx.sdma, &ctx->tx.buffer);
	sdmaBufferFree(&ctx->tx.sdma, &ctx->tx.bd);
	sdmaBufferFree(&ctx->rx.sdma, &ctx->rx.buffer);
	sdmaBufferFree(&ctx->rx.sdma, &ctx->rx.bd);
}


void dma_enable(void)
{
	struct sdmaCtx *const ctx = &sdma_common;

	ctx->enabled = 1;
	static uint8_t handleStack[_PAGE_SIZE] __attribute__((aligned(8)));
	beginthreadex(handleThread, 0, handleStack, sizeof(handleStack), ctx, &ctx->rxThreadId);

	sdma_enable(&ctx->rx.sdma);
	sdma_enable(&ctx->tx.sdma);
}


void dma_disable(void)
{
	struct sdmaCtx *const ctx = &sdma_common;

	mutexLock(ctx->rxLock);
	ctx->enabled = 0;
	condBroadcast(ctx->rxCond);
	mutexUnlock(ctx->rxLock);

	threadJoin(ctx->rxThreadId, 0);

	usleep(50 * 1000);

	sdma_disable(&ctx->tx.sdma);
	sdma_disable(&ctx->rx.sdma);
}


int dma_init(size_t size, size_t count, bool (*rxCb)(const uint8_t *data, size_t size))
{
	struct sdmaCtx *const ctx = &sdma_common;

	condCreate(&ctx->rxCond);
	mutexCreate(&ctx->rxLock);
	ctx->rxCb = rxCb;

	if (sdmaInit(ctx, size, count) < 0) {
		sdmaFree(ctx);
		log_error("dma_init failed\n");
		return -EIO;
	}
	return EOK;
}


int dma_reset(void)
{
	if (sdmaReset(&sdma_common) < 0) {
		return -EIO;
	}
	return EOK;
}
