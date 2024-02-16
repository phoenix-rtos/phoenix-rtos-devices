/*
 * Phoenix-RTOS
 *
 * GRLIB DMA Controller driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>

#include <sys/mman.h>
#include <sys/platform.h>

#include <phoenix/arch/sparcv8leon3/sparcv8leon3.h>

#include "grdmac2.h"

/* GRDMAC2 Registers */
#define GRDMAC2_CTRL  0 /* 0x00 Control */
#define GRDMAC2_STS   1 /* 0x04 Status */
#define GRDMAC2_TIMER 2 /* 0x08 Timer reset value */
#define GRDMAC2_CAPAB 3 /* 0x0c Capability */
#define GRDMAC2_FIRST 4 /* 0x10 First descriptor pointer */


/* GRDMAC2_CTRL bits */
#define CTRL_EN   (1 << 0) /* Enable */
#define CTRL_RST  (1 << 1) /* Reset */
#define CTRL_KICK (1 << 2) /* Read the next descriptor pointer */
#define CTRL_RT   (1 << 3) /* Restart current descriptor queue */
#define CTRL_IE   (1 << 4) /* Interrupt enable */
#define CTRL_IM   (1 << 5) /* Mask irq on descriptor completion */
#define CTRL_IER  (1 << 6) /* Interrupt enable on error */
#define CTRL_TE   (1 << 7) /* Timeout check */


void grdma_setup(grdma_ctx_t *ctx, void *first)
{
	ctx->base[GRDMAC2_CTRL] = CTRL_RST;
	ctx->base[GRDMAC2_TIMER] = 0xffffffff;
	ctx->base[GRDMAC2_FIRST] = (uintptr_t)first;
}


void grdma_start(grdma_ctx_t *ctx)
{
	ctx->base[GRDMAC2_CTRL] = CTRL_EN | CTRL_IE;
}


void grdma_restart(grdma_ctx_t *ctx)
{
	ctx->base[GRDMAC2_CTRL] |= CTRL_RT;
}


bool grdma_finished(grdma_ctx_t *ctx)
{
	return (ctx->base[GRDMAC2_STS] & 0x1) != 0;
}


int grdma_descrAlloc(grdma_ctx_t *ctx, size_t size)
{
	size = (size + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);
	void *descr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
	if (descr == MAP_FAILED) {
		return -ENOMEM;
	}

	ctx->descr = descr;
	ctx->descrSize = size;

	return EOK;
}


void grdma_destroy(grdma_ctx_t *ctx)
{
	if (ctx != NULL) {
		if (ctx->descr != NULL) {
			(void)munmap(ctx->descr, ctx->descrSize);
		}
		if (ctx->base != NULL) {
			(void)munmap((void *)ctx->base, _PAGE_SIZE);
		}
	}
	free(ctx);
}


grdma_ctx_t *grdma_init(unsigned int instance)
{
	/* Get DMA core info */
	ambapp_dev_t dev = { .devId = CORE_ID_GRDMAC2 };
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_ambapp,
		.ambapp = {
			.dev = &dev,
			.instance = &instance,
		}
	};

	if (platformctl(&pctl) < 0) {
		return NULL;
	}

	if (dev.bus != BUS_AMBA_APB) {
		/* GRDMAC2 should be on APB bus */
		return NULL;
	}

	grdma_ctx_t *ctx = malloc(sizeof(grdma_ctx_t));
	if (ctx == NULL) {
		return NULL;
	}

	uintptr_t base = ((uintptr_t)dev.info.apb.base) & ~(_PAGE_SIZE - 1);
	ctx->base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (ctx->base == MAP_FAILED) {
		free(ctx);
		return NULL;
	}

	ctx->base += ((uintptr_t)dev.info.apb.base - base) / sizeof(uintptr_t);
	ctx->irq = dev.irqn;

	ctx->descr = NULL;
	ctx->descrSize = 0u;

	return ctx;
}
