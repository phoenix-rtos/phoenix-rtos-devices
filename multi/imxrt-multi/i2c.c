/*
 * Phoenix-RTOS
 *
 * i.MX RT i2c driver using i2c-common api
 *
 * Copyright 2019, 2024 Phoenix Systems
 * Author: Andrzej Glowinski, Gerard Swiderski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdbool.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <i2c-msg.h>
#include <errno.h>

#include <board_config.h>
#include "common.h"
#include "config.h"
#include "trace.h"


#define I2C1_POS 0
#define I2C2_POS (I2C1_POS + I2C1)
#define I2C3_POS (I2C2_POS + I2C2)
#define I2C4_POS (I2C3_POS + I2C3)

#ifndef __CPU_IMXRT117X

#define I2C_CNT (I2C1 + I2C2 + I2C3 + I2C4)

#else

#define I2C5_POS (I2C4_POS + I2C4)
#define I2C6_POS (I2C5_POS + I2C5)

#define I2C_CNT (I2C1 + I2C2 + I2C3 + I2C4 + I2C5 + I2C6)

#endif

/* Compile-time configuration for each peripheral */
static const struct {
	volatile uint32_t *base;
	int clk;
	unsigned irq;
	uint8_t use;
	uint8_t pos;
	uint8_t speed;
	uint8_t pushpull;
} i2cPreConfig[] = {
	{ I2C1_BASE, I2C1_CLK, I2C1_IRQ, I2C1, I2C1_POS, I2C1_SPEED, I2C1_PUSHPULL },
	{ I2C2_BASE, I2C2_CLK, I2C2_IRQ, I2C2, I2C2_POS, I2C2_SPEED, I2C2_PUSHPULL },
	{ I2C3_BASE, I2C3_CLK, I2C3_IRQ, I2C3, I2C3_POS, I2C3_SPEED, I2C3_PUSHPULL },
	{ I2C4_BASE, I2C4_CLK, I2C4_IRQ, I2C4, I2C4_POS, I2C4_SPEED, I2C4_PUSHPULL },
#ifdef __CPU_IMXRT117X
	{ I2C5_BASE, I2C5_CLK, I2C5_IRQ, I2C5, I2C5_POS, I2C5_SPEED, I2C5_PUSHPULL },
	{ I2C6_BASE, I2C6_CLK, I2C6_IRQ, I2C6, I2C6_POS, I2C6_SPEED, I2C6_PUSHPULL },
#endif
};

#define N_PERIPHERALS (sizeof(i2cPreConfig) / sizeof(i2cPreConfig[0]))

#define FIFO_SIZE 4

#define STATUS_PIN_LOW_TIMEOUT  (1u << 13)
#define STATUS_FIFO_ERROR       (1u << 12)
#define STATUS_ARBITRATION_LOST (1u << 11)
#define STATUS_NACK             (1u << 10)
#define STATUS_STOP             (1u << 9)
#define STATUS_END              (1u << 8)
#define STATUS_RXFIFO_WM        (1u << 1)
#define STATUS_TXFIFO_WM        (1u << 0)
/* STATUS_STOP is not an error, but we check for it in case of a programming mistake */
#define STATUS_ERRORS (STATUS_PIN_LOW_TIMEOUT | STATUS_FIFO_ERROR | STATUS_ARBITRATION_LOST | STATUS_NACK | STATUS_STOP)

#define RET_STOP 1


/* clang-format off */

enum { veridr = 0, paramr, mcr = 4, msr, mier, mder, mcfgr0, mcfgr1, mcfgr2, mcfgr3, mdmr = 16,
	mccr0 = 18, mccr1 = 20, mfcr = 22, mfsr, mtdr, mrdr = 28, scr = 68, ssr, sier, sder,
	scfgr1 = 73, scfgr2, samr = 80, sasr = 84, star, stdr = 88, srdr = 92 };

enum { cmd_txdata = 0, cmd_rxdata = (1 << 8), cmd_stop = (2 << 8), cmd_start = (4 << 8) };

enum { i2c_stateBusy = 0, i2c_stateReady = 1 };

/* clang-format on */


static struct periph_info {
	volatile uint32_t *base;

	handle_t mutex;
	handle_t irqMutex;
	handle_t irqCond;
	handle_t irqHandle;
	int irqNo;

	volatile int state;
} i2c_common[I2C_CNT];


static int i2c_irqRoutine(unsigned int n, void *arg)
{
	TRACE_IRQ();
	struct periph_info *periph = arg;

	*(periph->base + mier) = 0;
	periph->state = i2c_stateReady;

	return 0;
}


static inline bool i2c_getTimings(uint8_t speed, uint32_t *mccr0Val, uint32_t *prescalerVal, uint32_t *mcfgr2Val)
{
	if ((speed < i2c_speed_slow) || (speed > i2c_speed_fast_plus)) {
		return false;
	}

#ifdef __CPU_IMXRT117X
	/* DATAVD, SETHOLD, CLKHI, CLKLO */
	static const uint32_t clockConfig[] = {
		[i2c_speed_slow] = 0x0f1d353eu,
		[i2c_speed_fast] = 0x03070b11u,
		[i2c_speed_fast_plus] = 0x04030406u,
	};

	/* Recommended minimum BUSIDLE values according to documentation */
	static const uint8_t busIdle[] = {
		[i2c_speed_slow] = (0x1d + 0x3e + 2) * 2,
		[i2c_speed_fast] = (0x07 + 0x11 + 2) * 2,
		[i2c_speed_fast_plus] = (0x03 + 0x06 + 2) * 2,
	};

	*prescalerVal = 1;
	const uint32_t filt = 1u;
#else
	static const uint32_t clockConfig[] = {
		[i2c_speed_slow] = 0x08111f28u,
		[i2c_speed_fast] = 0x08111f28u,
		[i2c_speed_fast_plus] = 0x01070b0fu,
	};

	static const uint8_t busIdle[] = {
		[i2c_speed_slow] = (0x11 + 0x28 + 2) * 2,
		[i2c_speed_fast] = (0x11 + 0x28 + 2) * 2,
		[i2c_speed_fast_plus] = (0x07 + 0x0f + 2) * 2,
	};

	*prescalerVal = (speed == i2c_speed_slow) ? 3u : 1u;
	const uint32_t filt = 2u;
#endif

	*mccr0Val = clockConfig[speed];
	*mcfgr2Val = (filt << 24u) | (filt << 16u) | (uint32_t)busIdle[speed];
	return true;
}


static int i2c_setupMaster(int pos, uint8_t speed, uint8_t pushpull)
{
	uint32_t mccr0Val, prescalerVal, mcfgr2Val;
	if (!i2c_getTimings(speed, &mccr0Val, &prescalerVal, &mcfgr2Val)) {
		return -EINVAL;
	}

	mutexLock(i2c_common[pos].mutex);

	TRACE();
	volatile uint32_t *base = i2c_common[pos].base;

	/* Reset before configuring */
	*(base + mcr) = 1u << 1u;
	common_dataBarrier();
	*(base + mcr) = 0u;
	common_dataBarrier();

	/* Disable doze mode */
	*(base + mcr) |= 1u << 2u;

	/* Disable host request (HREN,HRSEL), HRPOL active high */
	*(base + mcfgr0) = (*(base + mcfgr0) & ~5u) | 2u;

	/* Set pin configuration and prescaler, other bits set to 0 */
	*(base + mcfgr1) = (((pushpull != 0) ? 2u : 0u) << 24u) | prescalerVal;

	/* Set up glitch filter, bus idle timeout */
	*(base + mcfgr2) = mcfgr2Val;

	/* Disable pin low timeout feature */
	*(base + mcfgr3) &= ~(0xfffu << 8u);

	/* Configure timings */
	*(base + mccr0) = mccr0Val;

	/* Set tx and rx watermarks */
	*(base + mfcr) = 0u;

	/* Enable I2C */
	*(base + mcr) |= 1u;

	mutexUnlock(i2c_common[pos].mutex);

	return EOK;
}


static inline bool _i2c_checkBusBusy(volatile uint32_t *base)
{
	uint32_t status = *(base + msr);
	return ((status & (1u << 25u)) != 0u) && ((status & (1u << 24u)) == 0u);
}


static inline uint32_t _i2c_getFifoAvailable(volatile uint32_t *base, bool rx)
{
	uint32_t status = (*(base + mfsr) >> (rx ? 16u : 0u)) & 0x7u;
	return rx ? status : (FIFO_SIZE - status);
}


static inline int _i2c_getAndClearStatus(volatile uint32_t *base)
{
	uint32_t status = *(base + msr);
	*(base + msr) = status;

	if ((status & STATUS_PIN_LOW_TIMEOUT) != 0) {
		return -ETIME;
	}
	else if ((status & STATUS_FIFO_ERROR) != 0) {
		return -EIO;
	}
	else if ((status & STATUS_ARBITRATION_LOST) != 0) {
		return -EIO;
	}
	else if ((status & STATUS_NACK) != 0) {
		return -ENOENT;
	}
	else if ((status & STATUS_STOP) != 0) {
		return RET_STOP;
	}
	else {
		return EOK;
	}
}


/* Wait for interrupt, return status condition */
static int _i2c_waitForInterrupt(int pos, uint32_t bitWait)
{
	volatile uint32_t *base = i2c_common[pos].base;
	mutexLock(i2c_common[pos].irqMutex);
	i2c_common[pos].state = i2c_stateBusy;
	common_dataBarrier();
	*(base + mier) = STATUS_ERRORS | bitWait;
	do {
		int ret = condWait(i2c_common[pos].irqCond, i2c_common[pos].irqMutex, 1000);
		if (ret < 0) {
			*(base + mier) = 0;
			/* More cleanup will be done by _i2c_finishTransaction */
			mutexUnlock(i2c_common[pos].irqMutex);
			return -ETIME;
		}
	} while (i2c_common[pos].state != i2c_stateReady);

	mutexUnlock(i2c_common[pos].irqMutex);

	return _i2c_getAndClearStatus(base);
}


static inline int _i2c_waitForFifo(int pos, bool receive)
{
	if (_i2c_getFifoAvailable(i2c_common[pos].base, receive) == 0) {
		return _i2c_waitForInterrupt(pos, receive ? STATUS_RXFIFO_WM : STATUS_TXFIFO_WM);
	}
	else {
		return EOK;
	}
}


static inline int _i2c_putIntoFifo(int pos, uint16_t command)
{
	int ret = _i2c_waitForFifo(pos, false);
	if (ret != EOK) {
		return ret;
	}

	*(i2c_common[pos].base + mtdr) = command;
	common_dataBarrier();
	return EOK;
}


static inline int _i2c_transmit(int pos, const uint8_t *data, uint32_t len)
{
	while (len > 0) {
		int ret = _i2c_putIntoFifo(pos, *data);
		if (ret != EOK) {
			return ret;
		}

		data++;
		len--;
	}

	return EOK;
}


static int _i2c_receive(int pos, uint8_t *data, uint32_t len)
{
	volatile uint32_t *base = i2c_common[pos].base;
	while (len > 0) {
		uint32_t rxNow = (len > 256) ? 256 : len;
		len -= rxNow;
		int ret = _i2c_putIntoFifo(pos, (uint16_t)cmd_rxdata | (rxNow - 1));
		if (ret != EOK) {
			return ret;
		}

		while (rxNow > 0) {
			int ret = _i2c_waitForFifo(pos, true);
			if (ret != EOK) {
				return ret;
			}

			*data = *(base + mrdr) & 0xff;
			common_dataBarrier();
			data++;
			rxNow--;
		}
	}

	return EOK;
}


static inline int _i2c_start(int pos, uint8_t dev_addr, bool receive)
{
	uint16_t cmd = (uint16_t)cmd_start | (uint16_t)(dev_addr << 1u) | (receive ? 1u : 0u);
	return _i2c_putIntoFifo(pos, cmd);
}


static int _i2c_beginTransaction(int pos)
{
	volatile uint32_t *base = i2c_common[pos].base;
	if (_i2c_checkBusBusy(base)) {
		return -EBUSY;
	}

	/* Clear all flag */
	*(base + msr) = 0x7f << 8u;

	/* Turn off auto-stop option */
	*(base + mcfgr1) &= ~(1u << 8u);
	return EOK;
}


static int _i2c_finishTransaction(int pos, int ret)
{
	if (ret != EOK) {
		if (ret == RET_STOP) {
			TRACE("unexpected STOP");
			ret = -EIO;
		}

		if (ret == -ETIME) {
			mutexLock(i2c_common[pos].irqMutex);
			i2c_common[pos].state = i2c_stateReady;
			/* cond may have been signalled before we turned off IRQ - try to take it to ensure it's
			 * not signalled for next use */
			condWait(i2c_common[pos].irqCond, i2c_common[pos].irqMutex, 1);
			mutexUnlock(i2c_common[pos].irqMutex);
		}

		/* Reset FIFOs */
		*(i2c_common[pos].base + mcr) |= (1u << 9u) | (1u << 8u);
		return ret;
	}

	ret = _i2c_putIntoFifo(pos, (uint16_t)cmd_stop);
	if (ret != EOK) {
		return ret;
	}

	ret = _i2c_waitForInterrupt(pos, STATUS_END);
	return ret == RET_STOP ? EOK : ret;
}


static int _i2c_busWrite(int pos, uint8_t dev_addr, const uint8_t *data, uint32_t len)
{
	int ret = _i2c_beginTransaction(pos);
	ret = (ret == EOK) ? _i2c_start(pos, dev_addr, false) : ret;
	ret = (ret == EOK) ? _i2c_transmit(pos, data, len) : ret;
	return _i2c_finishTransaction(pos, ret);
}

static int _i2c_busRead(int pos, uint8_t dev_addr, uint8_t *data, uint32_t len)
{
	int ret = _i2c_beginTransaction(pos);
	ret = (ret == EOK) ? _i2c_start(pos, dev_addr, true) : ret;
	ret = (ret == EOK) ? _i2c_receive(pos, data, len) : ret;
	return _i2c_finishTransaction(pos, ret);
}


/* Performs i2c regiester read operation from the given slave device */
static int _i2c_regRead(int pos, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
{
	int ret = _i2c_beginTransaction(pos);
	ret = (ret == EOK) ? _i2c_start(pos, dev_addr, false) : ret;
	ret = (ret == EOK) ? _i2c_transmit(pos, &reg_addr, 1) : ret;
	ret = (ret == EOK) ? _i2c_start(pos, dev_addr, true) : ret;
	ret = (ret == EOK) ? _i2c_receive(pos, data_out, len) : ret;
	return _i2c_finishTransaction(pos, ret);
}


static int i2c_devctl(int pos, msg_t *msg)
{
	i2c_devctl_t *ctl = (i2c_devctl_t *)msg->i.raw;
	int ret;

	mutexLock(i2c_common[pos].mutex);
	switch (ctl->i.type) {
		case i2c_devctl_bus_write:
			ret = _i2c_busWrite(pos, ctl->i.dev_addr, msg->i.data, msg->i.size);

		case i2c_devctl_bus_read:
			ret = _i2c_busRead(pos, ctl->i.dev_addr, msg->o.data, msg->o.size);

		case i2c_devctl_reg_read:
			ret = _i2c_regRead(pos, ctl->i.dev_addr, ctl->i.reg_addr, msg->o.data, msg->o.size);

		default:
			ret = -ENOSYS;
	}

	mutexUnlock(i2c_common[pos].mutex);
	return ret;
}


void i2c_handleMsg(msg_t *msg, int dev)
{
	if (dev < (int)id_i2c1) {
		msg->o.err = -ENODEV;
		return;
	}

	dev -= (int)id_i2c1;
	if (((size_t)dev >= N_PERIPHERALS) || (i2cPreConfig[dev].use == 0)) {
		msg->o.err = -ENODEV;
		return;
	}

	int pos = i2cPreConfig[dev].pos;

	switch (msg->type) {
		case mtOpen:
			/* fall-through */
		case mtClose:
			msg->o.err = EOK;
			break;
		case mtRead:
			/* fall-through */
		case mtWrite:
			msg->o.err = -ENOSYS;
			break;

		case mtDevCtl:
			msg->o.err = i2c_devctl(pos, msg);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


static int i2c_getMuxData(int mux, int *mode, int *isel, int *daisy)
{
	switch (mux) {
		/* clang-format off */
#ifdef __CPU_IMXRT117X
		case pctl_mux_gpio_ad_32:		*mode = 0; *isel = pctl_isel_lpi2c1_scl; *daisy = 1; break;
		case pctl_mux_gpio_ad_08:		*mode = 1; *isel = pctl_isel_lpi2c1_scl; *daisy = 0; break;
		case pctl_mux_gpio_ad_33:		*mode = 0; *isel = pctl_isel_lpi2c1_sda; *daisy = 1; break;
		case pctl_mux_gpio_ad_09:		*mode = 1; *isel = pctl_isel_lpi2c1_sda; *daisy = 0; break;
		case pctl_mux_gpio_emc_b2_00:	*mode = 9; *isel = pctl_isel_lpi2c2_scl; *daisy = 0; break;
		case pctl_mux_gpio_ad_18:		*mode = 9; *isel = pctl_isel_lpi2c2_scl; *daisy = 1; break;
		case pctl_mux_gpio_emc_b2_01:	*mode = 9; *isel = pctl_isel_lpi2c2_sda; *daisy = 0; break;
		case pctl_mux_gpio_ad_19:		*mode = 9; *isel = pctl_isel_lpi2c2_sda; *daisy = 1; break;
		case pctl_mux_gpio_disp_b1_02:	*mode = 2; *isel = pctl_isel_lpi2c3_scl; *daisy = 0; break;
		case pctl_mux_gpio_disp_b2_10:	*mode = 6; *isel = pctl_isel_lpi2c3_scl; *daisy = 1; break;
		case pctl_mux_gpio_disp_b1_03:	*mode = 2; *isel = pctl_isel_lpi2c3_sda; *daisy = 0; break;
		case pctl_mux_gpio_disp_b2_11:	*mode = 6; *isel = pctl_isel_lpi2c3_sda; *daisy = 1; break;
		case pctl_mux_gpio_disp_b2_12:	*mode = 6; *isel = pctl_isel_lpi2c4_scl; *daisy = 1; break;
		case pctl_mux_gpio_ad_24:		*mode = 9; *isel = pctl_isel_lpi2c4_scl; *daisy = 0; break;
		case pctl_mux_gpio_disp_b2_13:	*mode = 6; *isel = pctl_isel_lpi2c4_sda; *daisy = 1; break;
		case pctl_mux_gpio_ad_25:		*mode = 9; *isel = pctl_isel_lpi2c4_sda; *daisy = 0; break;
		case pctl_mux_gpio_lpsr_05:		*mode = 0; *isel = pctl_isel_lpi2c5_scl; *daisy = 0; break;
		case pctl_mux_gpio_lpsr_09:		*mode = 6; *isel = pctl_isel_lpi2c5_scl; *daisy = 1; break;
		case pctl_mux_gpio_lpsr_04:		*mode = 0; *isel = pctl_isel_lpi2c5_sda; *daisy = 0; break;
		case pctl_mux_gpio_lpsr_08:		*mode = 6; *isel = pctl_isel_lpi2c5_sda; *daisy = 1; break;
		case pctl_mux_gpio_lpsr_07:		*mode = 0; *isel = pctl_isel_lpi2c6_scl; *daisy = 0; break;
		case pctl_mux_gpio_lpsr_11:		*mode = 2; *isel = pctl_isel_lpi2c6_scl; *daisy = 1; break;
		case pctl_mux_gpio_lpsr_06:		*mode = 0; *isel = pctl_isel_lpi2c6_sda; *daisy = 0; break;
		case pctl_mux_gpio_lpsr_10:		*mode = 2; *isel = pctl_isel_lpi2c6_sda; *daisy = 1; break;
#else
		case pctl_mux_gpio_sd_b1_04:	*mode = 2; *isel = pctl_isel_lpi2c1_scl; *daisy = 0; break;
		case pctl_mux_gpio_ad_b1_00:	*mode = 3; *isel = pctl_isel_lpi2c1_scl; *daisy = 1; break;
		case pctl_mux_gpio_sd_b1_05:	*mode = 2; *isel = pctl_isel_lpi2c1_sda; *daisy = 0; break;
		case pctl_mux_gpio_ad_b1_01:	*mode = 3; *isel = pctl_isel_lpi2c1_sda; *daisy = 1; break;
		case pctl_mux_gpio_b0_04:		*mode = 2; *isel = pctl_isel_lpi2c2_scl; *daisy = 1; break;
		case pctl_mux_gpio_sd_b1_11:	*mode = 3; *isel = pctl_isel_lpi2c2_scl; *daisy = 0; break;
		case pctl_mux_gpio_b0_05:		*mode = 2; *isel = pctl_isel_lpi2c2_sda; *daisy = 1; break;
		case pctl_mux_gpio_sd_b1_10:	*mode = 3; *isel = pctl_isel_lpi2c2_sda; *daisy = 0; break;
		case pctl_mux_gpio_ad_b1_07:	*mode = 1; *isel = pctl_isel_lpi2c3_scl; *daisy = 2; break;
		case pctl_mux_gpio_emc_22:		*mode = 2; *isel = pctl_isel_lpi2c3_scl; *daisy = 0; break;
		case pctl_mux_gpio_sd_b0_00:	*mode = 2; *isel = pctl_isel_lpi2c3_scl; *daisy = 1; break;
		case pctl_mux_gpio_ad_b1_06:	*mode = 1; *isel = pctl_isel_lpi2c3_sda; *daisy = 2; break;
		case pctl_mux_gpio_emc_21:		*mode = 2; *isel = pctl_isel_lpi2c3_sda; *daisy = 0; break;
		case pctl_mux_gpio_sd_b0_01:	*mode = 2; *isel = pctl_isel_lpi2c3_sda; *daisy = 1; break;
		case pctl_mux_gpio_emc_12:		*mode = 2; *isel = pctl_isel_lpi2c4_scl; *daisy = 0; break;
		case pctl_mux_gpio_ad_b0_12:	*mode = 0; *isel = pctl_isel_lpi2c4_scl; *daisy = 1; break;
		case pctl_mux_gpio_emc_11:		*mode = 2; *isel = pctl_isel_lpi2c4_sda; *daisy = 0; break;
		case pctl_mux_gpio_ad_b0_13:	*mode = 0; *isel = pctl_isel_lpi2c4_sda; *daisy = 1; break;
#endif
		default: return -1;
		/* clang-format off */
	}

	return 0;
}


static void i2c_initPins(void)
{
	static const struct {
		int mux;
		int pad;
	} info[] = {
#if I2C1
		{ PIN2MUX(I2C1_PIN_SCL), PIN2PAD(I2C1_PIN_SCL) },
		{ PIN2MUX(I2C1_PIN_SDA), PIN2PAD(I2C1_PIN_SDA) },
#endif

#if I2C2
		{ PIN2MUX(I2C2_PIN_SCL), PIN2PAD(I2C2_PIN_SCL) },
		{ PIN2MUX(I2C2_PIN_SDA), PIN2PAD(I2C2_PIN_SDA) },
#endif

#if I2C3
		{ PIN2MUX(I2C3_PIN_SCL), PIN2PAD(I2C3_PIN_SCL) },
		{ PIN2MUX(I2C3_PIN_SDA), PIN2PAD(I2C3_PIN_SDA) },
#endif

#if I2C4
		{ PIN2MUX(I2C4_PIN_SCL), PIN2PAD(I2C4_PIN_SCL) },
		{ PIN2MUX(I2C4_PIN_SDA), PIN2PAD(I2C4_PIN_SDA) },
#endif

#if I2C5
		{ PIN2MUX(I2C5_PIN_SCL), PIN2PAD(I2C5_PIN_SCL) },
		{ PIN2MUX(I2C5_PIN_SDA), PIN2PAD(I2C5_PIN_SDA) },
#endif

#if I2C6
		{ PIN2MUX(I2C6_PIN_SCL), PIN2PAD(I2C6_PIN_SCL) },
		{ PIN2MUX(I2C6_PIN_SDA), PIN2PAD(I2C6_PIN_SDA) },
#endif
	};

	for (int i = 0; i < sizeof(info) / sizeof(info[0]); ++i) {
		int mode, isel, daisy;
		int ret = i2c_getMuxData(info[i].mux, &mode, &isel, &daisy);
		if (ret == 0) {
			common_setMux(info[i].mux, 1, mode); /* SION has to be set to 1 for the I2C peripheral to work */
			common_setInput(isel, daisy);
			int ode = (i2cPreConfig[i].pushpull == 0) ? 1 : 0;
			common_setPad(info[i].pad, 0, 1, 1, 1, ode, 0, 0, 0);
		}
	}
}


int i2c_init(void)
{
	TRACE();
	int pos, dev;

	i2c_initPins();

	pos = 0;
	for (dev = 0; dev < N_PERIPHERALS; ++dev) {
		if (i2cPreConfig[dev].use == 0) {
			continue;
		}

#ifdef __CPU_IMXRT117X
		/* 	set clock - default mux is 0 -> OSC_RC_48M_DIV2, divider is 1
			final frequency is 24 MHz.
		*/

		if (common_setClock(i2cPreConfig[dev].clk, -1, -1, -1, -1, 1) < 0) {
			return -EFAULT;
		}
#else
		/*  set clock - default mux is 0 -> pll3_60m, divider is 1
			final frequency is 60 MHz.
		*/

		if (common_setClock(i2cPreConfig[dev].clk, clk_state_run) < 0) {
			return -EFAULT;
		}
#endif
		if (mutexCreate(&i2c_common[pos].mutex) != EOK) {
			return -EFAULT;
		}

		if (condCreate(&i2c_common[pos].irqCond) != EOK) {
			resourceDestroy(i2c_common[pos].mutex);
			return -EFAULT;
		}

		if (mutexCreate(&i2c_common[pos].irqMutex) != EOK) {
			resourceDestroy(i2c_common[pos].irqCond);
			resourceDestroy(i2c_common[pos].mutex);
			return -EFAULT;
		}

		i2c_common[pos].base = i2cPreConfig[dev].base;
		i2c_common[pos].irqNo = i2cPreConfig[dev].irq;
		i2c_common[pos].state = i2c_stateReady;

		i2c_setupMaster(pos, i2cPreConfig[dev].speed, i2cPreConfig[dev].pushpull);

		/* Attach ISR */
		interrupt(i2c_common[pos].irqNo, i2c_irqRoutine, &i2c_common[pos], i2c_common[pos].irqCond, &i2c_common[pos].irqHandle);
		pos++;
	}

	return EOK;
}
