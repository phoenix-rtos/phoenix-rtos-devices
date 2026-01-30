/*
 * Phoenix-RTOS
 *
 * GRLIB I2C MST driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <limits.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>


#include <board_config.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <phoenix/gaisler/ambapp.h>

#include <i2c.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#endif


#define I2C_CTRL_IEN (1U << 6) /* Interrupt Enable */
#define I2C_CTRL_EN  (1U << 7) /* I2C Enable */

#define I2C_CMD_IACK (1U << 0) /* Interrupt Acknowledge */
#define I2C_CMD_ACK  (1U << 3) /* 0 = ACK, 1 = NACK */
#define I2C_CMD_WR   (1U << 4) /* Write to Slave */
#define I2C_CMD_RD   (1U << 5) /* Read from Slave */
#define I2C_CMD_STO  (1U << 6) /* Generate STOP Condition */
#define I2C_CMD_STA  (1U << 7) /* Generate START Condition */

#define I2C_STAT_IF    (1U << 0) /* Interrupt Flag */
#define I2C_STAT_TIP   (1U << 1) /* Transfer in Progress */
#define I2C_STAT_AL    (1U << 5) /* Arbitration Lost */
#define I2C_STAT_BUSY  (1U << 6) /* Bus Busy */
#define I2C_STAT_RXACK (1U << 7) /* Received Acknowledge */


#ifndef I2C_SCL_FREQ_HZ
#define I2C_SCL_FREQ_HZ 100 * 1000U /* 100 kHz */
#endif


enum {
	i2c_cmdWrite = 0U,
	i2c_cmdRead = 1U,
};


typedef struct {
	volatile uint32_t prescale;
	volatile uint32_t ctrl;
	union {
		volatile uint32_t tx;
		volatile uint32_t rx;
	};
	union {
		volatile uint32_t cmd;
		volatile uint32_t stat;
	};
	volatile uint32_t filt;
} i2cmst_regs_t;


static struct {
	i2cmst_regs_t *regs;
	int irq;

	handle_t cond;
	handle_t lock;
} common;


__attribute__((section(".interrupt"), aligned(0x1000))) static int irqHandler(unsigned int intr, void *data)
{
	(void)intr;

	i2cmst_regs_t *regs = (i2cmst_regs_t *)data;

	if ((regs->stat & I2C_STAT_IF) != 0) {
		regs->cmd = I2C_CMD_IACK;
		return 1;
	}

	return -1;
}


static bool i2cBusBusy(void)
{
	return (common.regs->stat & I2C_STAT_BUSY) != 0;
}


static int i2c_wait(void)
{
	const time_t timeoutUs = 1000 * 1000;
	int ret = 0;

	/* Wait for transfer complete interrupt */
	while (((common.regs->stat & I2C_STAT_TIP) != 0) && (ret == 0)) {
		mutexLock(common.lock);
		ret = condWait(common.cond, common.lock, timeoutUs);
		if (ret == -ETIME) {
			ret = -ETIMEDOUT;
		}
		mutexUnlock(common.lock);
	}

	return ret;
}


static int i2c_checkStatus(bool expectAck)
{
	uint32_t status = common.regs->stat;

	/* Check for arbitration lost */
	if ((status & I2C_STAT_AL) != 0) {
		return -EIO;
	}

	/* Check ACK/NACK */
	bool receivedAck = (status & I2C_STAT_RXACK) == 0;

	if (receivedAck != expectAck) {
		return -EIO;
	}

	return 0;
}
static int startTransfer(uint8_t devAddr, uint8_t cmd)
{
	/* Send device address with R/W request */
	common.regs->tx = (devAddr << 1) | cmd;

	/* Generate START (address is always written) */
	common.regs->cmd = I2C_CMD_STA | I2C_CMD_WR;

	int ret = i2c_wait();
	if (ret < 0) {
		return ret;
	}

	return i2c_checkStatus(true); /* Expect ACK */
}


/* Performs i2c generic write operation to the given slave device. */
int i2c_busWrite(uint8_t devAddr, const uint8_t *data, uint32_t len)
{
	/* Single-master mode, bus should always be idle when we're not transferring */
	if (i2cBusBusy()) {
		return -EBUSY;
	}

	int ret = 0;
	uint32_t retries = 0U;
	const uint32_t maxRetries = 3U;

	/* Retry START + address until ACK or max retries */
	do {
		ret = startTransfer(devAddr, i2c_cmdWrite);
	} while ((retries++ < maxRetries) && (ret < 0));

	if (ret < 0) {
		common.regs->cmd = I2C_CMD_STO;
		(void)i2c_wait();
		return ret;
	}

	/* Write data bytes */
	for (size_t i = 0; (ret == 0) && (i < len); ++i) {
		common.regs->tx = data[i];

		/* Last byte: generate STOP */
		if (i == (len - 1U)) {
			common.regs->cmd = I2C_CMD_WR | I2C_CMD_STO;
		}
		else {
			common.regs->cmd = I2C_CMD_WR;
		}

		ret = i2c_wait();
		if (ret < 0) {
			break;
		}

		ret = i2c_checkStatus(true); /* Expect ACK */
	}

	if (ret != 0) {
		common.regs->cmd = I2C_CMD_STO;
	}

	/* Wait for STOP to complete */
	(void)i2c_wait();

	return ret;
}


/* Performs i2c generic read operation from the given slave device. */
int i2c_busRead(uint8_t devAddr, uint8_t *dataOut, uint32_t len)
{
	/* Single-master mode, bus should always be idle when we're not transferring */
	if (i2cBusBusy()) {
		return -EBUSY;
	}

	int ret = 0;
	uint32_t retries = 0U;
	const uint32_t maxRetries = 3U;

	/* Retry START + address until ACK or max retries */
	do {
		ret = startTransfer(devAddr, i2c_cmdRead);
	} while ((retries++ < maxRetries) && (ret < 0));

	if (ret < 0) {
		common.regs->cmd = I2C_CMD_STO;
		(void)i2c_wait();
		return ret;
	}

	/* Read data bytes */
	for (size_t i = 0; (ret == 0) && (i < len); ++i) {
		uint32_t cmd = I2C_CMD_RD;

		/* Last byte: NACK + STOP */
		if (i == (len - 1U)) {
			cmd |= I2C_CMD_ACK | I2C_CMD_STO;
		}

		common.regs->cmd = cmd;

		ret = i2c_wait();
		if (ret < 0) {
			break;
		}

		/* Check status: last byte expects NACK, others expect ACK */
		ret = i2c_checkStatus(i != (len - 1U));
		if (ret < 0) {
			break;
		}

		dataOut[i] = (uint8_t)(common.regs->rx & 0xffU);
	}

	if (ret != 0) {
		common.regs->cmd = I2C_CMD_STO;
	}

	/* Wait for STOP to complete */
	(void)i2c_wait();

	return ret;
}


/* Performs i2c register read operation from the given slave device */
int i2c_regRead(uint8_t devAddr, uint8_t regAddr, uint8_t *dataOut, uint32_t len)
{
	int ret = i2c_busWrite(devAddr, &regAddr, 1);
	if (ret < 0) {
		return ret;
	}

	return i2c_busRead(devAddr, dataOut, len);
}


static void setPrescaler(uint32_t i2cFreqHz)
{
	common.regs->prescale = (SYSCLK_FREQ / (i2cFreqHz * 5U)) - 1U;
}


int i2c_init(unsigned int dev)
{
	unsigned int instance = dev;
	ambapp_dev_t adev = { .devId = CORE_ID_I2CMST };
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_ambapp,
		.task.ambapp = {
			.dev = &adev,
			.instance = &instance,
		}
	};

	if (platformctl(&pctl) < 0) {
		return -ENODEV;
	}

	if (adev.bus != BUS_AMBA_APB) {
		/* I2C MST should be on APB bus */
		return -ENODEV;
	}

	common.irq = adev.irqn;

	uintptr_t pbase = (uintptr_t)adev.info.apb.base;
	uintptr_t base = (pbase & ~(PAGE_SIZE - 1U));
	common.regs = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (common.regs == MAP_FAILED) {
		fprintf(stderr, "i2c: mmap failed\n");
		return -ENOMEM;
	}
	common.regs = (i2cmst_regs_t *)((uintptr_t)common.regs + (pbase - base));

	int ret = condCreate(&common.cond);
	if (ret < 0) {
		fprintf(stderr, "i2c: condCreate failed\n");
		return ret;
	}

	ret = mutexCreate(&common.lock);
	if (ret < 0) {
		fprintf(stderr, "i2c: mutexCreate failed\n");
		return ret;
	}

	ret = interrupt(common.irq, irqHandler, common.regs, common.cond, NULL);
	if (ret < 0) {
		fprintf(stderr, "i2c: interrupt registration failed\n");
		return ret;
	}

	/* Disable core before setting prescaler */
	common.regs->ctrl = 0U;

	setPrescaler(I2C_SCL_FREQ_HZ);

	common.regs->ctrl = I2C_CTRL_EN | I2C_CTRL_IEN;
	common.regs->cmd = I2C_CMD_STO | I2C_CMD_ACK;

	return 0;
}
