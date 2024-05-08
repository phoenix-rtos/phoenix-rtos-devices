/*
 * Phoenix-RTOS
 *
 * i.MX RT i2c driver using i2c-common api
 *
 * Copyright 2019, 2024 Phoenix Systems
 * Author: Andrzej Glowinski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <i2c-msg.h>
#include <errno.h>

#include <board_config.h>
#include "common.h"
#include "config.h"
// #define TRACE_ENABLE
#include "trace.h"


#define I2C1_POS 0
#define I2C2_POS (I2C1_POS + I2C1)
#define I2C3_POS (I2C2_POS + I2C2)
#define I2C4_POS (I2C3_POS + I2C3)

#ifndef __CPU_IMXRT117X

#define I2C_CNT (I2C1 + I2C2 + I2C3 + I2C4)

static const int i2cConfig[] = { I2C1, I2C2, I2C3, I2C4 };

static const int i2cPos[] = { I2C1_POS, I2C2_POS, I2C3_POS, I2C4_POS };

#else

#define I2C5_POS (I2C4_POS + I2C4)
#define I2C6_POS (I2C5_POS + I2C5)

#define I2C_CNT (I2C1 + I2C2 + I2C3 + I2C4 + I2C5 + I2C6)

static const int i2cConfig[] = { I2C1, I2C2, I2C3, I2C4, I2C5, I2C6 };

static const int i2cPos[] = { I2C1_POS, I2C2_POS, I2C3_POS, I2C4_POS, I2C5_POS, I2C6_POS };

#endif


/* clang-format off */

enum { veridr = 0, paramr, mcr = 4, msr, mier, mder, mcfgr0, mcfgr1, mcfgr2, mcfgr3, mdmr = 16,
	mccr0 = 18, mccr1 = 20, mfcr = 22, mfsr, mtdr, mrdr = 28, scr = 68, ssr, sier, sder,
	scfgr1 = 73, scfgr2, samr = 80, sasr = 84, star, stdr = 88, srdr = 92 };

enum { cmd_txdata = 0, cmd_rxdata = (1 << 8), cmd_stop = (2 << 8), cmd_start = (4 << 8) };

enum { i2c_stateReady = 1 };

/* clang-format on */


struct {
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
	int pos = (int)arg;

	*(i2c_common[pos].base + mier) = 0;
	i2c_common[pos].state = i2c_stateReady;

	return 0;
}


static void i2c_setupMaster(int pos, uint8_t filterSDA, uint8_t filterSCL, uint16_t busIdle)
{
	TRACE();
	volatile uint32_t *base = i2c_common[pos].base;

	/* Reset before configuring */
	*(base + mcr) = 1u << 1u;
	*(base + mcr) = 0u;

	/* Disable doze mode */
	*(base + mcr) |= 1u << 2u;

	/* Disable host request (HREN,HRSEL), HRPOL active high */
	*(base + mcfgr0) = (*(base + mcfgr0) & ~5u) | 2u;

	/* Disable AUTOSTOP and turn NAK Ignore off */
	*(base + mcfgr1) &= ~((1u << 9u) | (1u << 8u));

	/* Set tx and rx watermarks */
	*(base + mfcr) = 0u;

	// TODO: set frequency

	*(base + mcfgr2) =
		((uint32_t)(filterSDA & 0xfu) << 24u) |
		((uint32_t)(filterSCL & 0xf) << 16u) |
		((uint32_t)busIdle & 0xfffu);

	/* Disable pin low timeout feature */
	*(base + mcfgr3) &= ~(0xfffu << 8u);

	/* Attach ISR */
	interrupt(i2c_common[pos].irqNo, i2c_irqRoutine, (void *)pos, i2c_common[pos].irqCond, &i2c_common[pos].irqHandle);

	/* Enable I2C */
	*(base + mcr) |= 1u;
}


static inline int i2c_checkBusBusy(volatile uint32_t *base)
{
	TRACE();
	uint32_t status = *(base + msr);
	return (((status & (1u << 25u)) != 0u) && ((status & (1u << 24u)) == 0u)) ? 1 : 0;
}


/* Performs i2c generic write operation to the given slave device */
static int i2c_busWrite(int pos, int8_t dev_addr, const uint8_t *data, uint32_t len)
{
	TRACE("pos %d", pos);
	volatile uint32_t *base = i2c_common[pos].base;

	if (i2c_checkBusBusy(base) != 0) {
		return -EBUSY;
	}

	/* Clear all flag */
	*(base + msr) = 7u << 8u;

	/* Turn off auto-stop option */
	*(base + mcfgr1) &= ~(1u << 8u);


	*(base + mtdr) = (uint16_t)cmd_start | (uint16_t)(dev_addr << 1u);

	return -EIO;
}


/* Performs i2c generic read operation from the given slave device */
static int i2c_busRead(int pos, uint8_t dev_addr, uint8_t *data_out, uint32_t len)
{
	TRACE("pos %d", pos);
	return -EIO;
}


/* Performs i2c regiester read operation from the given slave device */
static int i2c_regRead(int pos, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
{
	TRACE("pos %d", pos);
	int ret = i2c_busWrite(pos, dev_addr, &reg_addr, 1);
	return (ret < 0) ? ret : i2c_busRead(pos, dev_addr, data_out, len);
}


static int i2c_devctl(int pos, msg_t *msg)
{
	i2c_devctl_t *ctl = (i2c_devctl_t *)msg->i.raw;

	TRACE("pos %d", pos);
	switch (ctl->i.type) {
		case i2c_devctl_bus_write:
			return i2c_busWrite(pos, ctl->i.dev_addr, msg->i.data, msg->i.size);

		case i2c_devctl_bus_read:
			return i2c_busRead(pos, ctl->i.dev_addr, msg->o.data, msg->o.size);

		case i2c_devctl_reg_read:
			return i2c_regRead(pos, ctl->i.dev_addr, ctl->i.reg_addr, msg->o.data, msg->o.size);

		default:
			break;
	}

	return -ENOSYS;
}


void i2c_handleMsg(msg_t *msg, int dev)
{
	dev -= (int)id_i2c1;
	TRACE("dev %d", dev);
	if (((size_t)dev >= sizeof(i2cConfig) / sizeof(i2cConfig[0])) || (i2cConfig[dev] == 0)) {
		msg->o.err = -ENODEV;
	}

	int pos = i2cPos[dev];
	TRACE("pos %d", pos);

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


int i2c_init(void)
{
	TRACE();
	int pos, dev;

	static const struct {
		volatile uint32_t *base;
		int clk;
		unsigned irq;
	} i2cInfo[] = {
		{ I2C1_BASE, I2C1_CLK, I2C1_IRQ },
		{ I2C2_BASE, I2C2_CLK, I2C2_IRQ },
		{ I2C3_BASE, I2C3_CLK, I2C3_IRQ },
		{ I2C4_BASE, I2C4_CLK, I2C4_IRQ },
#ifdef __CPU_IMXRT117X
		{ I2C5_BASE, I2C5_CLK, I2C5_IRQ },
		{ I2C6_BASE, I2C6_CLK, I2C6_IRQ },
#endif
	};

	// TODO: setup pins
	// TODO: implement write
	// TODO: implement read
	// TODO: implement frequency set

	pos = 0;
	for (dev = 0; dev < sizeof(i2cConfig) / sizeof(i2cConfig[0]); ++dev) {
		if (i2cConfig[dev] == 0) {
			continue;
		}

#ifdef __CPU_IMXRT117X
		if (common_setClock(i2cInfo[dev].clk, -1, -1, -1, -1, 1) < 0) {
			return -EFAULT;
		}
#else
		if (common_setClock(i2cInfo[dev].clk, clk_state_run) < 0) {
			return -EFAULT;
		}
#endif
		if (condCreate(&i2c_common[pos].mutex) != EOK) {
			return -EFAULT;
		}

		if (mutexCreate(&i2c_common[pos].irqCond) != EOK) {
			resourceDestroy(i2c_common[pos].mutex);
			return -EFAULT;
		}

		if (mutexCreate(&i2c_common[pos].irqMutex) != EOK) {
			resourceDestroy(i2c_common[pos].irqCond);
			resourceDestroy(i2c_common[pos].mutex);
			return -EFAULT;
		}

		i2c_common[pos].base = i2cInfo[dev].base;
		i2c_common[pos].irqNo = i2cInfo[dev].irq;
		i2c_common[pos].state = i2c_stateReady;

		i2c_setupMaster(pos, 0u, 0u, 0u);
	}

	return EOK;
}
