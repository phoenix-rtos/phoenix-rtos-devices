/*
 * Phoenix-RTOS
 *
 * Xilinx/AMD AXI_I2C (axi_iic) driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Kamil Ber, Krzysztof Szostek
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <posix/utils.h>

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <i2c.h>

#if defined(__CPU_ZYNQ7000)
#include <phoenix/arch/armv7a/zynq7000/zynq7000.h>
#elif defined(__CPU_ZYNQMP)
#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>
#else
#error "Unsupported platform"
#endif

#include <board_config.h>

#include "axi-i2c-regs.h"
#include "axi-irq-ctrl-regs.h"


#define I2C_FIFO_DEPTH 16
/* TODO: not used in this version*/
#define I2C_TRANS_SIZE_MAX 16

typedef struct {
	unsigned int irq; /* I2C controller IRQ */
	struct {
		int clk_freq; /* Frequency of Clock, in Hz */
		int i2c_freq; /* Frequency of I2C BUSm in Hz */
		int tsusta;   /* TSUSTA timing in ns (tsusta + tr) */
		int tsusto;   /* TSUSTO timing in ns (tsusto + tr) */
		int thdsta;   /* THDSTA timing in ns (thdsta + tf) */
		int tsudat;   /* TSUDAT timing in ns (tsudat + tf) */
		int tbuf;     /* TSUDAT timing in ns */
	} timings;
	addr_t paddr; /* I2C controller base physical address */
	addr_t irq_addr;
} i2c_info_t;

/* Timings from IMX219 from IMX219 PG:
 * tr 		= 120 ns (max)
 * tf 		= 120 ns (max)
 * tsusta 	= 260 ns (min)
 * tsusto 	= 260 ns (min)
 * thdsta 	= 260 ns (min)
 * tsudat 	= 50 ns (min)
 * tbus   	= 500 ns (min)
 */

/* clang-format off */
/* Device configuration based on board_config */
/* TODO:: inject base address*/
#if defined(__CPU_ZYNQMP)
static const i2c_info_t devsInfo[] = {
	{
		.irq = 121,
		.paddr = 0xA0030000,
    .irq_addr = 0xA0000000,
		.timings = {
			.clk_freq = 100000000,
			.i2c_freq = 100000,
			.tsusta = 5700,
			.tsusto = 5000,
			.thdsta = 4300,
			.tsudat = 550,
			.tbuf = 5000
		}
	},
	{
		.irq = 122,
		.paddr = 0xA0030000,
    .irq_addr = 0xA0000000,
		.timings = {
			.clk_freq = 100000000,
			.i2c_freq = 100000,
			.tsusta = 5700,
			.tsusto = 5000,
			.thdsta = 4300,
			.tsudat = 550,
			.tbuf = 5000
		}
	}
};
#endif
/* clang-format on */

static struct {
	unsigned int devID;
	volatile uint32_t *base;     /* I2C registers base address */
	volatile uint32_t *irq_addr; /* AXI_IRQ controller */
	int initialized;
	handle_t lock; /* I2C IRQ mutex */
	handle_t cond; /* I2C IRQ cond */
	handle_t inth; /* I2C IRQ handle */
	volatile uint32_t st;
} i2c = { 0 };

/* Not implemented:
 *  - slave mode
 *  - multi-master mode (arbitration lost status flag inspection)
 *  - dynamic control mode
 */

static int i2c_isr(unsigned int n, void *arg)
{
	uint32_t reg;

	*(i2c.irq_addr + 0x14) = 0x2;

	i2c.st = *(i2c.base + I2C_REG_ISR) & *(i2c.base + I2C_REG_IER);


	/* TODO: what if more than one irq status bit is asserted???*/
	/* For some irq sources this won't work! Eg. RX Full -> Read the RX data first!*/
	/* clear status register */
	*(i2c.base + I2C_REG_ISR) = i2c.st;

	/* arbitration lost */
	if (i2c.st & ISR_ARB_LOST) {
		/* from pg090: Firmware must respond by first clearing the Control Register (CR) MSMS bit*/
		reg = *(i2c.base + I2C_REG_CR) & ~CR_MSMS;

		*(i2c.base + I2C_REG_CR) = reg;
		/* From pg090: this bit must be set to flush the FIFO if either
		 * (a) arbitration is lost or
		 * (b) if a transmit error occurs.
		 */
		*(i2c.base + I2C_REG_CR) = reg | CR_TX_FIFO_RST;
		*(i2c.base + I2C_REG_CR) = reg & ~CR_TX_FIFO_RST;

		*(i2c.base + I2C_REG_IER) &= ~ISR_ARB_LOST;
	}


	/* tx fifo empty (tx error or receivecompletion) */
	if (i2c.st & ISR_TX_ERR_OR_COMPLETE) {
		*(i2c.base + I2C_REG_IER) &= ~ISR_TX_ERR_OR_COMPLETE;
	}
	/* tx fifo empty (receive underflow) */
	else if (i2c.st & ISR_RX_FIFO_FULL) {
		/* first, need to clear the RX FULL status!!!*/
		*(i2c.base + I2C_REG_IER) &= ~ISR_RX_FIFO_FULL;
	}
	/* rx fifo full (receive underflow) */
	else if (i2c.st & ISR_TX_FIFO_EMPTY) {
		*(i2c.base + I2C_REG_IER) &= ~ISR_TX_FIFO_EMPTY;
	}
	/* bus not busy */
	else if (i2c.st & ISR_BUS_NOT_BUSY) {
		*(i2c.base + I2C_REG_IER) &= ~ISR_BUS_NOT_BUSY;
	}
	/* addressed as a slave */
	else if (i2c.st & ISR_ADDR_AS_SLAVE) {
		*(i2c.base + I2C_REG_IER) &= ~ISR_ADDR_AS_SLAVE;
	}
	/* not addressed as a slave */
	else if (i2c.st & ISR_NOT_ADDR_AS_SLAVE) {
		*(i2c.base + I2C_REG_IER) &= ~ISR_NOT_ADDR_AS_SLAVE;
	}
	/* tx fifo half empty */
	else if (i2c.st & ISR_TX_FIFO_HALF_EMPTY) {
		*(i2c.base + I2C_REG_IER) &= ~ISR_TX_FIFO_HALF_EMPTY;
	}

	return 1;
}


static inline int i2c_isBusBusy(void)
{
	return *(i2c.base + I2C_REG_SR) & SR_BB;
}


static inline void i2c_enableIrq(uint32_t mask)
{
	*(i2c.base + I2C_REG_IER) |= mask;
}

static inline void i2c_disableIrq(uint32_t mask)
{
	*(i2c.base + I2C_REG_IER) &= ~mask;

	i2c.st &= ~mask;
}


static inline void i2c_clearIrq(uint32_t mask)
{
	/* clear interrupts */
	*(i2c.base + I2C_REG_ISR) = mask;

	i2c.st &= ~mask;
}

static inline void i2c_clearIrqSt(void)
{
	uint32_t reg;

	/* clear status interrupts */
	reg = *(i2c.base + I2C_REG_ISR);
	*(i2c.base + I2C_REG_ISR) = reg;

	/* clear status */
	i2c.st = 0;
}


static inline void writeToFIFO(uint8_t data)
{
	*(i2c.base + I2C_REG_TX_FIFO) = data;
	printf("Writing to TX_FIFO: 0x%02x\n", data);
}


int i2c_regWrite16(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, uint32_t len)
{
	int ret = EOK;
	volatile uint32_t *base = i2c.base;

	if (i2c.initialized == 0) {
		return -EIO;
	}

	if (data == NULL) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	if (i2c_isBusBusy() == 1) {
		return -EBUSY;
	}

	*(base + I2C_REG_CR) = CR_EN | CR_TX_FIFO_RST;
	*(base + I2C_REG_CR) = CR_EN;

	i2c_clearIrqSt();
	/*
	 * Set Rx FIFO Occupancy depth to throttle at first byte (after reset = 0)
	 * */
	*(base + I2C_REG_RX_FIFO_PIRQ) = 15;

	*(base + I2C_REG_TX_FIFO) = (dev_addr << 1);
	*(base + I2C_REG_TX_FIFO) = (reg_addr >> 8) & 0xff;
	*(base + I2C_REG_TX_FIFO) = reg_addr & 0xff;

	/* Enable interrupts, arbitration lost, tx overflow, transfer NACK,
	 * completion irq, bus not busy
	 */
	*(i2c.base + I2C_REG_IER) = ISR_TX_FIFO_EMPTY;

	*(base + I2C_REG_CR) = CR_EN | CR_MSMS | CR_TX;
	usleep(1);
	if (ret < 0) {
		return ret;
	}
	*(base + I2C_REG_CR) = CR_EN | CR_TX;
	*(base + I2C_REG_TX_FIFO) = *data;

	/* arbitration lost, tx overflow, transfer NACK, completion irq, bus not busy */
	*(i2c.base + I2C_REG_IER) = ISR_RX_FIFO_FULL | ISR_TX_FIFO_EMPTY |
			ISR_TX_ERR_OR_COMPLETE | ISR_ARB_LOST;
	i2c_clearIrq(ISR_TX_FIFO_EMPTY);

	/* disable interrupts */
	*(i2c.base + I2C_REG_IER) = 0;

	usleep(1);

	return ret;
}

int i2c_regRead16(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data_out, uint32_t len)
{
	int ret = 0;
	volatile uint32_t *base = i2c.base;
	*(base + I2C_REG_CR) = CR_EN | CR_TX_FIFO_RST;
	*(base + I2C_REG_CR) = CR_EN;

	/*
	 * Set Rx FIFO Occupancy depth to throttle at first byte (after reset = 0)
	 * */
	*(base + I2C_REG_RX_FIFO_PIRQ) = 0;

	*(base + I2C_REG_TX_FIFO) = (dev_addr << 1);
	*(base + I2C_REG_TX_FIFO) = (reg_addr >> 8) & 0xff;
	*(base + I2C_REG_TX_FIFO) = reg_addr & 0xff;

	*(base + I2C_REG_CR) = CR_EN | CR_MSMS | CR_TX;
	usleep(1);
	*(base + I2C_REG_CR) = CR_EN | CR_MSMS | CR_RSTA | CR_TXAK;
	*(base + I2C_REG_TX_FIFO) = (dev_addr << 1) | 1;
	usleep(1);
	*(base + I2C_REG_CR) = CR_EN;

	*data_out = *(base + I2C_REG_RX_FIFO);
	return ret;
}


static void i2c_initCtrl(void)
{

	/* Depth of RX fifo for interrupt generation*/
	*(i2c.base + I2C_REG_RX_FIFO_PIRQ) = I2C_FIFO_DEPTH - 1;

	/* clear TX and RX fifos */
	*(i2c.base + I2C_REG_CR) = CR_EN | CR_TX_FIFO_RST;

	/* enable controller */
	*(i2c.base + I2C_REG_CR) = CR_EN;

	/* enable interrupts */
	// *(i2c.base + I2C_REG_GIE) = GIE_EN;
	*(i2c.base + I2C_REG_GIE) = 0;

	/* reset GPO - necessary to see devices */
	*(i2c.base + I2C_REG_GPO) = 0;
	usleep(1000);
	*(i2c.base + I2C_REG_GPO) = 1;

	i2c_clearIrqSt();
}


int i2c_initClk(void)
{
	int reg;
	const i2c_info_t *info = &devsInfo[i2c.devID];

	/* SCL low/high -> in number of input clock cycles*/
	reg = info->timings.clk_freq / (2 * info->timings.i2c_freq) - 7;
	if (reg == 0) {
		return -EINVAL;
	}
	*(i2c.base + I2C_REG_THIGH) = reg;
	*(i2c.base + I2C_REG_TLOW) = reg;

	/* this div can lead to a significant error, however it ensures
	 * set timings are slower than the I2C spec in worst case
	 */
	reg = 1000000000 / info->timings.clk_freq;

	*(i2c.base + I2C_REG_TSUSTA) = info->timings.tsusta / reg;
	*(i2c.base + I2C_REG_TSUSTO) = info->timings.tsusto / reg;
	*(i2c.base + I2C_REG_THDSTA) = info->timings.thdsta / reg;
	*(i2c.base + I2C_REG_TSUDAT) = info->timings.tsudat / reg;
	*(i2c.base + I2C_REG_TBUF) = info->timings.tbuf / reg;

	return EOK;
}

static void i2c_initIrq(unsigned int dev_no)
{
	// TODO: Macro these values.
	// TODO: This function should be put in another file as a separate component.
	*(i2c.irq_addr + AXI_REG_IER) = 0x1f;
	*(i2c.irq_addr + AXI_REG_MER) = AXI_MER_ME | AXI_MER_HIE;
	*(i2c.irq_addr + AXI_REG_ILR) = 0x05;
	if (0)
		interrupt(devsInfo[dev_no].irq, i2c_isr, NULL, i2c.cond, &i2c.inth);
}

static int i2c_reset(void)
{
	/* from pg090: write 0xA to reset registers to their default states.*/
	*(i2c.base + I2C_REG_SOFTR) = 0xA;  // Ekhm, we assume that i2c is initialized

	return EOK;
}


int i2c_init(unsigned int dev_no)
{
	int err = EOK;

	if (dev_no >= sizeof(devsInfo) / sizeof(devsInfo[0])) {
		return -ENODEV;
	}

	i2c.devID = dev_no;

	err = mutexCreate(&i2c.lock);
	if (err < 0) {
		return err;
	}

	err = condCreate(&i2c.cond);
	if (err < 0) {
		resourceDestroy(i2c.lock);
		return err;
	}

	/* map i2c registers */
	i2c.base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, devsInfo[dev_no].paddr);
	if (i2c.base == MAP_FAILED) {
		resourceDestroy(i2c.cond);
		resourceDestroy(i2c.lock);
		return -ENOMEM;
	}

	i2c.irq_addr = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, devsInfo[dev_no].irq_addr);
	if (i2c.base == MAP_FAILED) {
		resourceDestroy(i2c.cond);
		resourceDestroy(i2c.lock);
		return -ENOMEM;
	}

	err = i2c_reset();
	if (err < 0) {
		return err;
	}

	/* initialize clock */
	err = i2c_initClk();
	if (err < 0) {
		return err;
	}

	/* controller initialization */
	i2c_initCtrl();

	/* interrupts initialization */
	i2c_initIrq(dev_no);


	i2c.initialized = 1;

	return EOK;
}
