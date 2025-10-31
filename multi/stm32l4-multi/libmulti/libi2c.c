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
#if defined(__CPU_STM32N6)
#include "../rcc.h"
#endif


#define TIMEOUT (100 * 1000)

#define ANALOG_NOISE_FILTER 0


static const struct libi2c_peripheralInfo {
	void *base;
	int clk;
	int irq_ev;
	int irq_er;
#if defined(__CPU_STM32N6)
	enum ipclks clksel;    /* Clock selector */
	enum clock_ids clksrc; /* ID of source clock */
#endif
} i2cinfo[] = {
#if defined(__CPU_STM32L4X6)
	{ I2C1_BASE, pctl_i2c1, i2c1_ev_irq, i2c1_er_irq },
	{ I2C2_BASE, pctl_i2c2, i2c2_ev_irq, i2c2_er_irq },
	{ I2C3_BASE, pctl_i2c3, i2c3_ev_irq, i2c3_er_irq },
	{ I2C4_BASE, pctl_i2c4, i2c4_ev_irq, i2c4_er_irq },
#elif defined(__CPU_STM32N6)
	{ I2C1_BASE, pctl_i2c1, i2c1_ev_irq, i2c1_er_irq, pctl_ipclk_i2c1sel, clkid_per },
	{ I2C2_BASE, pctl_i2c2, i2c2_ev_irq, i2c2_er_irq, pctl_ipclk_i2c2sel, clkid_per },
	{ I2C3_BASE, pctl_i2c3, i2c3_ev_irq, i2c3_er_irq, pctl_ipclk_i2c3sel, clkid_per },
	{ I2C4_BASE, pctl_i2c4, i2c4_ev_irq, i2c4_er_irq, pctl_ipclk_i2c4sel, clkid_per },
#endif
};


enum { cr1 = 0, cr2, oar1, oar2, timingr, timeoutr, isr, icr, pecr, rxdr, txdr };

enum { dir_read, dir_write };


#if defined(__CPU_STM32L4X6)
static int libi2c_clockSetup(const struct libi2c_peripheralInfo *info, uint32_t *out)
{
	/* On this platform no extra information is used for clock setup */
	(void)info;
	*out = getCpufreq();
	return EOK;
}
#elif defined(__CPU_STM32N6)
static int libi2c_clockSetup(const struct libi2c_peripheralInfo *info, uint32_t *out)
{
	int ret;
	ret = rcc_setClksel(info->clksel, info->clksrc);
	if (ret < 0) {
		return ret;
	}

	uint64_t freq;
	ret = clockdef_getClock(info->clksrc, &freq);
	if (ret < 0) {
		return ret;
	}

	*out = (uint32_t)freq;
	return ret;
}
#endif


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
		int ret = libi2c_waitForIrq(ctx);
		if (ret < 0) {
			return ret;
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
	ssize_t ret;

	ctx->err = 0;
	libi2c_start(ctx);
	libi2c_transactionStart(ctx, addr, dir_write, 1);

	*(ctx->base + txdr) = reg;
	ret = libi2c_waitForIrq(ctx);
	if ((ret >= 0) && (ctx->err == 0)) {
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


static uint32_t divide_ceil(uint32_t n, uint32_t d)
{
	uint32_t res = (n + d - 1) / d;
	return (res == 0) ? 1 : res;
}


static int libi2c_calculateTiming(uint32_t refclk, enum libi2c_speed speed, uint32_t *timingr_val, uint32_t *digifilter)
{
	/* In constants below, each tick represents 0.0625 us, corresponding to frequency of 16 MHz */
	/* Ticks per one clock period (from I2C protocol documentation) */
	static const uint8_t lookup_tCLK[] = {
		[libi2c_speed_standard] = 160, /* 10 us => 100 kHz */
		[libi2c_speed_fast] = 40,      /* 2.5 us => 400 kHz */
		[libi2c_speed_fastplus] = 16,  /* 1 us => 1 MHz */
	};
	/* Ticks to hold high state (from I2C protocol documentation) */
	static const uint8_t lookup_tHIGH[] = {
		[libi2c_speed_standard] = 65, /* 4 us minimum + 0.0625 safety margin */
		[libi2c_speed_fast] = 11,     /* 0.6 us minimum + 0.0625 safety margin */
		[libi2c_speed_fastplus] = 5,  /* 0.25 us minimum + 0.0625 safety margin */
	};
	/* Data set-up time (tSU;DAT from I2C protocol documentation) */
	static const uint8_t lookup_tSCLDEL[] = {
		[libi2c_speed_standard] = 4, /* 0.25 us */
		[libi2c_speed_fast] = 2,     /* 0.125 us >= 0.1 us minimum */
		[libi2c_speed_fastplus] = 1, /* 0.625 us >= 0.05 us minimum */
	};
	/* SCL rise time, measured empirically to be at least 0.0625 us. */
	static const uint32_t tRISE = 1;
	/* Analog noise filter delays clock transitions by another 0.0625 us. */
	static const uint32_t tANF = (ANALOG_NOISE_FILTER != 0) ? 1 : 0;

	if ((speed < libi2c_speed_standard) || (speed > libi2c_speed_fastplus)) {
		return -EINVAL;
	}

	/* Digital noise filter period - 0.125 us */
	uint32_t tDNF = 2;

	uint32_t tSCLL = lookup_tCLK[speed] - lookup_tHIGH[speed] - tRISE - tANF;
	uint32_t tSCLH = lookup_tHIGH[speed] - tANF;
	uint32_t tSCLDEL = lookup_tSCLDEL[speed];
	uint32_t prescaler = 1;

	/* Round the clock frequency to 1 MHz */
	uint32_t m = divide_ceil(refclk, 1000000);
	/* If actual frequency is different from our "base frequency", scale the results. */
	if (m != 16) {
		tDNF = divide_ceil(tDNF * m, 16);
		if (tDNF > 15) {
			tDNF = 15;
		}

		tSCLDEL = divide_ceil(tSCLDEL * m, 16);
		tSCLL = divide_ceil(tSCLL * m, 16);
		tSCLH = divide_ceil(tSCLH * m, 16);
	}

	/* Regardless of frequency, the low period seems to be 3 cycles too long and high period 2 cycles too long.
	 * This was measured empirically. */
	tSCLL = (tSCLL > (tDNF + 3)) ? (tSCLL - (tDNF + 3)) : 1;
	tSCLH = (tSCLH > (tDNF + 2)) ? (tSCLH - (tDNF + 2)) : 1;

	/* tSCLDEL has a range of [1:16], but we can save some calculations by scaling it up to [16:256]
	 * just for the purpose of calculating prescaler. */
	uint32_t longest = max(tSCLDEL * 16, max(tSCLL, tSCLH));
	if (longest > 256) {
		prescaler = divide_ceil(longest, 256);
	}

	if (prescaler > 16) {
		/* Input frequency too fast */
		return -EINVAL;
	}

	if (prescaler > 1) {
		tSCLDEL = divide_ceil(tSCLDEL, prescaler);
		tSCLL = divide_ceil(tSCLL, prescaler);
		tSCLH = divide_ceil(tSCLH, prescaler);
	}

	*timingr_val =
			(((prescaler - 1) & 0xf) << 28) |
			(((tSCLDEL - 1) & 0xf) << 20) |
			(((tSCLH - 1) & 0xff) << 8) |
			(((tSCLL - 1) & 0xff) << 0);
	*digifilter = tDNF;
	return 0;
}


static int _libi2c_setSpeedInternal(libi2c_ctx_t *ctx, enum libi2c_speed speed)
{
	int ret;
	uint32_t timingr_val, digifilter;
	dataBarier();
	ret = libi2c_calculateTiming(ctx->refclk_freq, speed, &timingr_val, &digifilter);
	if (ret < 0) {
		return ret;
	}

	uint32_t t = *(ctx->base + cr1) & ~(0xf << 8);
	*(ctx->base + cr1) = t | (digifilter << 8);
	*(ctx->base + timingr) = timingr_val;
	dataBarier();
	return 0;
}


int libi2c_setSpeed(libi2c_ctx_t *ctx, enum libi2c_speed speed)
{
	devClk(ctx->clk, 1);
	int ret = _libi2c_setSpeedInternal(ctx, speed);
	devClk(ctx->clk, 0);
	return ret;
}


int libi2c_init(libi2c_ctx_t *ctx, int i2c)
{
	unsigned int t;

	if (i2c < i2c1 || i2c > i2c4 || ctx == NULL) {
		return -EINVAL;
	}

	i2c -= i2c1;

	ctx->base = i2cinfo[i2c].base;
	ctx->clk = i2cinfo[i2c].clk;

	devClk(ctx->clk, 1);
	if (libi2c_clockSetup(&i2cinfo[i2c], &ctx->refclk_freq) < 0) {
		return -EIO;
	}

	mutexCreate(&ctx->irqlock);
	condCreate(&ctx->irqcond);

	interrupt(i2cinfo[i2c].irq_ev, libi2c_irqHandler, (void *)ctx, ctx->irqcond, NULL);
	interrupt(i2cinfo[i2c].irq_er, libi2c_errHandler, (void *)ctx, ctx->irqcond, NULL);

	*(ctx->base + cr1) &= ~1;
	dataBarier();

	t = *(ctx->base + cr1) & ~0xfffdff;
	/* Enable analog noise filter (if requested) and interrupts (ERR, TC, NACK, RX, TX).
	 * Note: Analog noise filter is 1 to disable. */
	*(ctx->base + cr1) = t | ((ANALOG_NOISE_FILTER != 0) ? 0 : (1 << 12)) | (1 << 7) | (1 << 6) | (1 << 4) | (1 << 2) | (1 << 1);
	int ret = _libi2c_setSpeedInternal(ctx, libi2c_speed_standard);
	devClk(ctx->clk, 0);

	return ret;
}
