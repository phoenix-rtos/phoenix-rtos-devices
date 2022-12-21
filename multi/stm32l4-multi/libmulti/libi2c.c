/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 I2C driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/interrupt.h>
#include <sys/threads.h>
#include <sys/pwman.h>
#include <errno.h>
#include <unistd.h>
#include "libmulti/libi2c.h"
#include "../common.h"


#define TIMEOUT (100 * 1000)


static const struct {
	void *base;
	int clk;
	int irq_ev;
	int irq_er;
} i2cinfo[] = {
	{ (void *)0x40005400, pctl_i2c1, i2c1_ev_irq, i2c1_er_irq },
	{ (void *)0x40005800, pctl_i2c2, i2c2_ev_irq, i2c2_er_irq },
	{ (void *)0x40005c00, pctl_i2c3, i2c3_ev_irq, i2c3_er_irq },
	{ (void *)0x40008400, pctl_i2c4, i2c4_ev_irq, i2c4_er_irq }
};


enum { cr1 = 0, cr2, oar1, oar2, timingr, timeoutr, isr, icr, pecr, rxdr, txdr };

enum { dir_read, dir_write };


static int libi2c_irqHandler(unsigned int n, void *arg)
{
	*(((libi2c_ctx_t *)arg)->base + cr1) &= ~((1 << 6) | (1 << 4) | (1 << 2) | (1 << 1));

	return 1;
}


static int libi2c_errHandler(unsigned int n, void *arg)
{
	libi2c_ctx_t *ctx = (libi2c_ctx_t *)arg;

	ctx->err = -1;
	*(ctx->base + cr1) &= ~(1 << 7);
	return 1;
}


static int libi2c_waitForIrq(libi2c_ctx_t *ctx)
{
	int err = EOK;

	*(ctx->base + cr1) |= (1 << 7) | (1 << 6) | (1 << 4) | (1 << 2) | (1 << 1);

	mutexLock(ctx->irqlock);
	while (((*(ctx->base + isr) & ((1 << 6) | (1 << 4) | (0x3 << 1))) == 0) && (ctx->err == 0) && (err == EOK)) {
		err = condWait(ctx->irqcond, ctx->irqlock, TIMEOUT);
	}

	if ((*(ctx->base + isr) & (1 << 4)) != 0) {
		err = -EIO;
	}
	mutexUnlock(ctx->irqlock);

	return err;
}


static void libi2c_start(libi2c_ctx_t *ctx)
{
	keepidle(1);
	devClk(ctx->clk, 1);
	*(ctx->base + cr1) |= 1;
	dataBarier();
}


static void libi2c_stop(libi2c_ctx_t *ctx)
{
	int retry = 0;

	/* Generate STOP */
	*(ctx->base + cr2) |= 1 << 14;
	dataBarier();

	while ((*(ctx->base + isr) & (1 << 5)) == 0) {
		if (++retry > 100) {
			/* we tried, nothing else to do in this case */
			break;
		}
		usleep(1000);
	}

	*(ctx->base + cr1) &= ~1;
	dataBarier();
	devClk(ctx->clk, 0);
	keepidle(0);
}


static void libi2c_transactionStart(libi2c_ctx_t *ctx, unsigned char address, int dir, size_t len)
{
	unsigned int t;

	t = *(ctx->base + cr2) & ~(0x7ffffff);
	*(ctx->base + cr2) = t | ((len & 0xff) << 16) | ((dir == dir_read) << 10) | ((address & 0x7f) << 1);
	dataBarier();
	*(ctx->base + cr2) |= 1 << 13;
	dataBarier();
}


static ssize_t _libi2c_read(libi2c_ctx_t *ctx, unsigned char addr, void *buff, size_t len)
{
	ssize_t i;

	if (len > 255) {
		len = 255;
	}

	libi2c_transactionStart(ctx, addr, dir_read, len);

	for (i = 0; i < len && !ctx->err; ++i) {
		if (libi2c_waitForIrq(ctx) < 0) {
			return -1;
		}
		((unsigned char *)buff)[i] = (unsigned char)(*(ctx->base + rxdr) & 0xff);
	}

	return (ctx->err == 0) ? i : -1;
}


ssize_t libi2c_read(libi2c_ctx_t *ctx, unsigned char addr, void *buff, size_t len)
{
	ssize_t ret;

	ctx->err = 0;
	libi2c_start(ctx);
	ret = _libi2c_read(ctx, addr, buff, len);
	libi2c_stop(ctx);

	return ret;
}


ssize_t libi2c_readReg(libi2c_ctx_t *ctx, unsigned char addr, unsigned char reg, void *buff, size_t len)
{
	ssize_t ret = -1;

	ctx->err = 0;
	libi2c_start(ctx);
	libi2c_transactionStart(ctx, addr, dir_write, 1);

	*(ctx->base + txdr) = reg;
	if ((libi2c_waitForIrq(ctx) >= 0) && (ctx->err == 0)) {
		ret = _libi2c_read(ctx, addr, buff, len);
	}
	libi2c_stop(ctx);

	return ret;
}


static ssize_t _libi2c_write(libi2c_ctx_t *ctx, unsigned char addr, unsigned char reg, int withreg, const void *buff, size_t len)
{
	ssize_t i = 0;

	if (len > 255) {
		len = 255;
	}

	if (withreg != 0) {
		libi2c_transactionStart(ctx, addr, dir_write, len + 1);
		*(ctx->base + txdr) = reg;
		if (libi2c_waitForIrq(ctx) < 0) {
			return -1;
		}
	}
	else {
		libi2c_transactionStart(ctx, addr, dir_write, len);
	}

	for (i = 0; i < (ssize_t)len && !ctx->err; ++i) {
		*(ctx->base + txdr) = ((const unsigned char *)buff)[i];
		if (libi2c_waitForIrq(ctx) < 0) {
			return -1;
		}
	}

	return (ctx->err == 0) ? i : -1;
}


ssize_t libi2c_write(libi2c_ctx_t *ctx, unsigned char addr, const void *buff, size_t len)
{
	ssize_t ret;

	ctx->err = 0;
	libi2c_start(ctx);
	ret = _libi2c_write(ctx, addr, 0, 0, buff, len);
	libi2c_stop(ctx);

	return ret;
}


ssize_t libi2c_writeReg(libi2c_ctx_t *ctx, unsigned char addr, unsigned char reg, const void *buff, size_t len)
{
	ssize_t ret;

	ctx->err = 0;
	libi2c_start(ctx);
	ret = _libi2c_write(ctx, addr, reg, 1, buff, len);
	libi2c_stop(ctx);

	return ret;
}


int libi2c_init(libi2c_ctx_t *ctx, int i2c)
{
	int cpuclk = getCpufreq(), presc;
	unsigned int t;

	if (i2c < i2c1 || i2c > i2c4 || ctx == NULL) {
		return -1;
	}

	i2c -= i2c1;

	ctx->base = i2cinfo[i2c].base;
	ctx->clk = i2cinfo[i2c].clk;

	devClk(ctx->clk, 1);

	mutexCreate(&ctx->irqlock);
	condCreate(&ctx->irqcond);

	interrupt(i2cinfo[i2c].irq_ev, libi2c_irqHandler, (void *)ctx, ctx->irqcond, NULL);
	interrupt(i2cinfo[i2c].irq_er, libi2c_errHandler, (void *)ctx, ctx->irqcond, NULL);

	*(ctx->base + cr1) &= ~1;
	dataBarier();

	t = *(ctx->base + cr1) & ~0xfffdff;
	*(ctx->base + cr1) = t | (1 << 12) | (0x7 << 8) | (1 << 7) | (1 << 6) | (1 << 4) | (1 << 2) | (1 << 1);

	presc = ((cpuclk + 500 * 1000) / (1000 * 1000)) / 4;
	t = *(ctx->base + timingr) & ~((0xf << 18) | 0xffffff);
	*(ctx->base + timingr) = t | (((presc & 0xf) << 28) | (0x4 << 20) | (0x2 << 16) | (0xf << 8) | 0x13);
	dataBarier();

	devClk(ctx->clk, 0);

	return 0;
}
