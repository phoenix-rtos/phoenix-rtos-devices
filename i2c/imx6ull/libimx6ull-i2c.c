/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL I2C driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <i2c.h>

#include <phoenix/arch/imx6ull.h>


/* clang-format off */
/* NOTE: i2c registers are 16-bit and every second offset is reserved for future use */
enum { iadr = 0, ifdr = 2, i2cr = 4, i2sr = 6, i2dr = 8 };
enum { i2c_cmd_write = 0x0, i2c_cmd_read = 0x1 };
/* clang-format on */

static const uint32_t i2c_addr[] = { 0x021a0000, 0x021a4000, 0x021a8000, 0x021f8000 };
static const int clocks[] = { pctl_clk_i2c1_serial, pctl_clk_i2c2_serial, pctl_clk_i2c3_serial, pctl_clk_i2c4_serial };


typedef struct {
	int pctl;
	char val;
} i2c_pctl_t;

static const i2c_pctl_t i2c_pctl_mux[4][2] = {
	{ { pctl_mux_gpio1_02, 0 }, { pctl_mux_gpio1_03, 0 } },
	{ { pctl_mux_gpio1_00, 0 }, { pctl_mux_gpio1_01, 0 } },
	{ { pctl_mux_lcd_d1, 4 }, { pctl_mux_lcd_d0, 4 } },
	{ { pctl_mux_lcd_d3, 4 }, { pctl_mux_lcd_d2, 4 } },
};

static const i2c_pctl_t i2c_pctl_isel[4][2] = {
	{ { pctl_isel_i2c1_scl, 0 }, { pctl_isel_i2c1_sda, 1 } },
	{ { pctl_isel_i2c2_scl, 1 }, { pctl_isel_i2c2_sda, 1 } },
	{ { pctl_isel_i2c3_scl, 2 }, { pctl_isel_i2c3_sda, 2 } },
	{ { pctl_isel_i2c4_scl, 2 }, { pctl_isel_i2c4_sda, 2 } },
};


static const unsigned i2c_int_no[] = { 32 + 36, 32 + 37, 32 + 38, 32 + 35 };


static struct {
	volatile uint16_t *base;
	uint16_t dev_no;
	int initialized;

	handle_t cond;
	handle_t inth;
	handle_t lock;
} i2c = { 0 };

/* Not implemented:
 *  - slave mode
 *  - multi-master mode (arbitration lost status flag inspection)
 *  - register readout could be written by generating RESTART event instead of STOP / START (beneficial in multi-master mode)
 */


static void platform_init(void)
{
	platformctl_t pctl;
	unsigned int i;

	/* enable clock */
	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = clocks[i2c.dev_no - 1];
	pctl.devclock.state = 0x3;

	platformctl(&pctl);

	/* set MUXes */
	pctl.type = pctl_iomux;
	pctl.iomux.sion = 1; /* from RM: SION bit needs to be enabled for i2c pads */

	for (i = 0; i < 2; ++i) {
		pctl.iomux.mux = i2c_pctl_mux[i2c.dev_no - 1][i].pctl;
		pctl.iomux.mode = i2c_pctl_mux[i2c.dev_no - 1][i].val;
		platformctl(&pctl);
	}

	/* set IOsel */
	pctl.type = pctl_ioisel;

	for (i = 0; i < 2; ++i) {
		pctl.ioisel.isel = i2c_pctl_isel[i2c.dev_no - 1][i].pctl;
		pctl.ioisel.daisy = i2c_pctl_isel[i2c.dev_no - 1][i].val;
		platformctl(&pctl);
	}

	/* PAD configuration taken from linux Device Tree, not needed for our current targets, leaving commented-out
	 * as it might need to be enabled for targets with missing pullup resistors (and written in generic way) */
#if 0
	pctl.type = pctl_iopad;
	pctl.iopad.pad = pctl_pad_gpio1_02;
	pctl.iopad.hys = 1;
	pctl.iopad.pus = 2; /* 100kOhm pull-up */
	pctl.iopad.pue = 0;    /* pull/keeper enabled */
	pctl.iopad.pke = 1;
	pctl.iopad.ode = 1;      /* open drain enable */
	pctl.iopad.speed = 2; /* speed - 100 MHz */
	pctl.iopad.dse = 0x4;
	pctl.iopad.sre = 0;

	platformctl(&pctl);

	pctl.iopad.pad = pctl_pad_gpio1_03;
	platformctl(&pctl);
#endif
}


static int isBusBusy(void)
{
	return *(i2c.base + i2sr) & (1 << 5);
}


static int waitBusBusy(int for_busy)
{
	int timeout = 10; /* x 10 us = 100us */

	while (!!isBusBusy() != !!for_busy) {
		usleep(10);
		if (--timeout <= 0)
			return -ETIMEDOUT;
	}

	return 0;
}


static int i2c_intr(unsigned int intr, void *data)
{
	/* clear interrupt */
	*(i2c.base + i2sr) &= ~(1 << 1);

	return 1;
}


static int i2c_trxComplete(void)
{
	const time_t timeout_us = 100; /* 100 us */
	int ret = 0;

	mutexLock(i2c.lock);

	/* wait for transfer complete interrupt - with timeout */
	while (!(*(i2c.base + i2sr) & (1 << 7))) {
		if ((ret = condWait(i2c.cond, i2c.lock, timeout_us)) == -ETIME) {
			ret = -ETIMEDOUT;
			break;
		}
	}

	/* clear interrupt second time - sticky conditionals */
	*(i2c.base + i2sr) &= ~(1 << 1);

	mutexUnlock(i2c.lock);
	return ret;
}


static int i2c_acked(void)
{
	if (*(i2c.base + i2sr) & (1 << 0))
		return -EIO;

	return 0;
}


/* writes 1 byte and waits for completion / ack */
static int writeByte(uint8_t byte)
{
	int ret = 0;

	*(i2c.base + i2sr) = 0; /* clear previous interrupts */
	*(i2c.base + i2dr) = byte;

	/* wait for byte to be sent */
	ret = i2c_trxComplete();
	if (ret < 0)
		return ret;

	return i2c_acked();
}


/* Performs i2c generic write operation to the given slave device. */
int i2c_busWrite(uint8_t dev_addr, const uint8_t *data, uint32_t len)
{
	unsigned int i;
	int ret;

	if (!i2c.initialized)
		return -EIO;

	/* single-master mode, bus should always be idle when we're not transferring */
	if (isBusBusy())
		return -EBUSY;

	/* generate START: enable master mode, enable TX, enable interrupt */
	*(i2c.base + i2cr) |= (1 << 6) | (1 << 5) | (1 << 4);

	/* wait for the bus to be busy */
	ret = waitBusBusy(1);

	/* send first byte - DEVICE WRITE request */
	if (ret == 0)
		ret = writeByte((dev_addr << 1) | i2c_cmd_write);

	/* send data */
	for (i = 0; (ret == 0) && (i < len); ++i)
		ret = writeByte(data[i]);

	/* generate STOP: disable master mode, disable TX */
	*(i2c.base + i2cr) &= ~((1 << 6) | (1 << 5) | (1 << 4));

	/* wait for the bus to be idle */
	waitBusBusy(0); /* don't overwrite possible error in ret */

	return ret;
}


static int _doRead(uint8_t dev_addr, uint8_t *data_out, uint32_t len)
{
	unsigned int i;
	uint16_t temp;
	int ret;

	/* send first byte - DEVICE READ request */
	ret = writeByte((dev_addr << 1) | i2c_cmd_read);
	if (ret < 0)
		return ret;

	/* setup bus for data readout - disable TX, enable ACKing for every byte except the last one */
	temp = *(i2c.base + i2cr) & ~((1 << 4) | (1 << 3));
	if (len <= 1)
		temp |= (1 << 3);
	*(i2c.base + i2cr) = temp;

	/* RM: dummy read to enter read mode */
	(void)*(i2c.base + i2dr);

	/* read data */
	for (i = 0; (ret == 0) && (i < len); ++i) {
		ret = i2c_trxComplete();
		if (ret < 0)
			break;

		if (i == len - 1) {
			/* we must generate STOP before reading i2dr to avoid generating another clock cycle */
			*(i2c.base + i2cr) &= ~((1 << 5) | (1 << 4));
		}
		else if (i == len - 2) {
			/* we must set TXAK before receiving next-to-last byte to acknowledge the whole message */
			*(i2c.base + i2cr) |= (1 << 3);
		}

		data_out[i] = *(i2c.base + i2dr);
	}

	return ret;
}


/* Performs i2c generic read operation from the given slave device. */
int i2c_busRead(uint8_t dev_addr, uint8_t *data_out, uint32_t len)
{
	int ret;

	if (!i2c.initialized)
		return -EIO;

	/* single-master mode, bus should always be idle when we're not transferring */
	if (isBusBusy())
		return -EBUSY;

	/* generate START: enable master mode, enable TX for sending register address */
	*(i2c.base + i2cr) |= (1 << 6) | (1 << 5) | (1 << 4);

	/* wait for the bus to be busy */
	ret = waitBusBusy(1);

	if (ret >= 0)
		ret = _doRead(dev_addr, data_out, len);

	/* generate STOP: disable master mode, disable TX (might be done twice without any issue) */
	*(i2c.base + i2cr) &= ~((1 << 6) | (1 << 5) | (1 << 4));

	/* wait for the bus to be idle */
	waitBusBusy(0); /* don't overwrite possible error in ret */

	return ret;
}


/* Performs i2c regiester read operation from the given slave device */
int i2c_regRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
{
	int ret;

	ret = i2c_busWrite(dev_addr, &reg_addr, 1);
	if (ret < 0)
		return ret;

	return i2c_busRead(dev_addr, data_out, len);
}


int i2c_init(unsigned int dev_no)
{
	i2c.dev_no = dev_no;

	if ((i2c.dev_no < 1) || (i2c.dev_no > (sizeof(i2c_addr) / sizeof(i2c_addr[0]))))
		return -1;

	/* initialize */
	i2c.base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, i2c_addr[i2c.dev_no - 1]);

	if (i2c.base == MAP_FAILED)
		return -2;

	platform_init();

	/* prepare interrupt routine */
	if (mutexCreate(&i2c.lock) != EOK) {
		munmap((void *)i2c.base, _PAGE_SIZE);
		return -3;
	}

	if (condCreate(&i2c.cond) != EOK) {
		munmap((void *)i2c.base, _PAGE_SIZE);
		resourceDestroy(i2c.lock);
		return -3;
	}

	interrupt(i2c_int_no[i2c.dev_no - 1], i2c_intr, NULL, i2c.cond, &i2c.inth);

	/* disable i2c - soft reset */
	*(i2c.base + i2cr) = 0;

	/* assuming IPG_CLK_ROOT = 66 MHz - for i2c Fast Mode (400kbps) we need 66000/400 = 165 divider */
	/* let's use divider == 160 (register value 0x30), using oscilloscope we measure ~375kHz SCLK */
	*(i2c.base + ifdr) = 0x30;

	/* set our slave address - leave at 0x0 to avoid device id collisions */
	*(i2c.base + iadr) = (0x0 << 1);

	/* enable i2c - needs to be set before any other bits in this register */
	*(i2c.base + i2cr) = (1 << 7);

	i2c.initialized = 1;
	return 0;
}
