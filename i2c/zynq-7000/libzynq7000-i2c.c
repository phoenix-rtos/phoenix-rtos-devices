/*
 * Phoenix-RTOS
 *
 * Zynq-7000 I2C driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
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
#include <sys/mman.h>

#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <i2c.h>

#include <phoenix/arch/zynq7000.h>
#include <board_config.h>

/* I2C registers */
#define I2C_CR        0
#define I2C_SR        1
#define I2C_ADDR      2
#define I2C_DATA      3
#define I2C_ISR       4
#define I2C_TRANS_SZ  5
#define I2C_SLV_PAUSE 6
#define I2C_TIMEOUT   7
#define I2C_IMR       8
#define I2C_IER       9
#define I2C_IDR       10

#define I2C_FIFO_DEPTH     16
#define I2C_TRANS_SIZE_MAX 255

/* MIO I2C pins configuration */
#define MIO_SDA 0x1240
#define MIO_SCL 0x1240


typedef struct {
	unsigned int id;  /* I2C controller ID */
	unsigned int irq; /* I2C controller IRQ */
	addr_t paddr;     /* I2C controller base physical address */

	struct {              /* I2C MIO pins configuration */
		int pin;          /* MIO pin */
		unsigned int cfg; /* MIO pin configuration */
	} pins[2];            /* I2C pins, CLK, SDA */
} i2c_info_t;


/* device configuration based on board_config */
static const i2c_info_t devsInfo[] = {
	{
		.id = 0,
		.irq = 57,
		.paddr = 0xe0004000,
		.pins = {
			{ I2C0_SDA, MIO_SDA },
			{ I2C0_SCL, MIO_SCL },
		}
	},
	{
		.id = 1,
		.irq = 80,
		.paddr = 0xe0005000,
		.pins = {
			{ I2C1_SDA, MIO_SDA },
			{ I2C1_SCL, MIO_SCL },
		}
	}
};


static struct {
	unsigned int devID;
	volatile uint32_t *base; /* I2C registers base address */
	int initialized;

	handle_t lock;           /* I2C IRQ mutex */
	handle_t cond;           /* I2C IRQ cond */
	handle_t inth;           /* I2C IRQ handle */

	volatile uint32_t st;
} i2c = { 0 };


static int i2c_isr(unsigned int n, void *arg)
{
	i2c.st = *(i2c.base + I2C_ISR);

	/* clear status register */
	*(i2c.base + I2C_ISR) = i2c.st;

	/* transfer complete */
	if (i2c.st & 0x1) {
		*(i2c.base + I2C_IDR) |= 0x1;
	}
	/* data irq */
	else if (i2c.st & (1 << 1)) {
		*(i2c.base + I2C_IDR) |= (1 << 1);
	}
	/* transfer NACK */
	else if (i2c.st & (1 << 2)) {
		*(i2c.base + I2C_IDR) |= (1 << 2);
	}
	/* transfer timeout */
	else if (i2c.st & (1 << 3)) {
		*(i2c.base + I2C_IDR) |= (1 << 3);
	}
	/* transmit overflow */
	else if (i2c.st & (1 << 6)) {
		*(i2c.base + I2C_IDR) |= (1 << 6);
	}
	/* receive underflow */
	else if (i2c.st & (1 << 7)) {
		*(i2c.base + I2C_IDR) |= (1 << 7);
	}
	/* arbitration lost */
	else if (i2c.st & (1 << 9)) {
		*(i2c.base + I2C_IDR) |= (1 << 9);
	}

	return 1;
}


static inline int i2c_isBusBusy(void)
{
	return *(i2c.base + I2C_SR) & (1 << 8);
}


static inline void i2c_clearIrqSt(void)
{
	uint32_t reg;

	/* clear status interrupts */
	reg = *(i2c.base + I2C_ISR);
	*(i2c.base + I2C_ISR) = reg;

	/* clear status */
	i2c.st = 0;
}


static int i2c_trxComplete(void)
{
	int ret = EOK;
	const time_t timeoutUs = 3000;

	mutexLock(i2c.lock);
	/* wait for data or completion irq */
	while ((i2c.st & 0x3) == 0) {
		if (condWait(i2c.cond, i2c.lock, timeoutUs) == -ETIME) {
			ret = -ETIMEDOUT;
			break;
		}

		/* transfer NACK, transfer timeout, transmit overflow, receive underflow, arbitration lost */
		if (i2c.st & ((1 << 2) | (1 << 3) | (1 << 6) | (1 << 7) | (1 << 9))) {
			ret = -EIO;
		}
	}

	i2c.st = 0;
	mutexUnlock(i2c.lock);

	return ret;
}


int i2c_busWrite(uint8_t dev_addr, const uint8_t *data, uint32_t len)
{
	unsigned int fifoAvail;
	int i, ret = EOK, sz = 0, start = 0;

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

	/* clear FIFO, enable ACK, master mode, master transmitter */
	*(i2c.base + I2C_CR) |= (1 << 6) | (1 << 3) | (1 << 1);
	*(i2c.base + I2C_CR) &= ~1;

	i2c_clearIrqSt();

	/* enable bus hold logic */
	*(i2c.base + I2C_CR) |= (1 << 4);

	while (sz < len) {
		/* arbitration lost, tx overflow, enable timeout, transfer NACK, completion irq */
		*(i2c.base + I2C_IER) = (1 << 9) | (1 << 6) | (1 << 3) | (1 << 2) | 0x1;

		fifoAvail = I2C_FIFO_DEPTH - *(i2c.base + I2C_TRANS_SZ);

		/* write data to FIFO */
		for (i = 0; i < fifoAvail && sz++ < len; ++i) {
			*(i2c.base + I2C_DATA) = *(data++);
		}

		/* set slave address and start transmission - do it only once */
		/* TODO: len > I2C_TRANS_SIZE_MAX has not been tested. If transfer will not be delivered, set dev_addr again */
		if (start == 0) {
			start = 1;
			*(i2c.base + I2C_ADDR) = dev_addr;
		}

		ret = i2c_trxComplete();
		if (ret < 0) {
			break;
		}
	}

	/* HOLD = 0 - terminate the transfer. It generates a STOP condition. */
	*(i2c.base + I2C_CR) &= ~(1 << 4);

	/* disable interrupts */
	*(i2c.base + I2C_IDR) = (1 << 9) | (1 << 6) | (1 << 3) | (1 << 2) | (1 << 1) | 0x1;

	return ret;
}


static int i2c_transferRead(uint8_t dev_addr, uint8_t *buff, uint32_t sz)
{
	int ret = EOK, i, tmp;

	if (sz > I2C_FIFO_DEPTH) {
		*(i2c.base + I2C_CR) |= (1 << 4);
	}

	/* set data count to receive */
	*(i2c.base + I2C_TRANS_SZ) = sz;

	/* set slave address and start transmission */
	*(i2c.base + I2C_ADDR) = dev_addr;

	while (sz > 0 && *(i2c.base + I2C_TRANS_SZ) != 0) {
		/* arbitration lost, rx underflow, enable timeout, data, completion Irq */
		*(i2c.base + I2C_IER) = (1 << 9) | (1 << 7) | (1 << 3) | (1 << 2) | (1 << 1) | 0x1;

		ret = i2c_trxComplete();
		if (ret < 0) {
			break;
		}

		/* data irq is generated when there are two free locations available in the FIFO */
		if (sz >= (I2C_FIFO_DEPTH - 2)) {
			for (i = 0; i < I2C_FIFO_DEPTH - 2; ++i) {
				*(buff++) = *(i2c.base + I2C_DATA);
				sz--;
			}

			/* set data count to receive */
			*(i2c.base + I2C_TRANS_SZ) = sz;
		}
		/* transfer complete irq */
		else {
			tmp = sz - *(i2c.base + I2C_TRANS_SZ);
			for (i = 0; i < tmp; ++i) {
				*(buff++) = *(i2c.base + I2C_DATA);
				--sz;
			}
		}
	}

	return ret;
}


int i2c_busRead(uint8_t dev_addr, uint8_t *data_out, uint32_t len)
{
	int ret = EOK, sz = 0, bytes2Read;

	if (i2c.initialized == 0) {
		return -EIO;
	}

	if (data_out == NULL) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	if (i2c_isBusBusy() == 1) {
		return -EBUSY;
	}

	/* clear FIFO, enable ACK, master mode, master receiver */
	*(i2c.base + I2C_CR) |= (1 << 6) | (1 << 3) | 0x3;

	i2c_clearIrqSt();

	do {
		bytes2Read = (len - sz >= I2C_TRANS_SIZE_MAX) ? I2C_TRANS_SIZE_MAX : (len - sz);

		ret = i2c_transferRead(dev_addr, data_out, bytes2Read);
		if (ret < 0) {
			break;
		}

		data_out += bytes2Read;
		sz += bytes2Read;
	} while (sz < len);

	/* HOLD = 0 - terminate the transfer */
	*(i2c.base + I2C_CR) &= ~(1 << 4);

	/* disable interrupts */
	*(i2c.base + I2C_IDR) = (1 << 9) | (1 << 7) | (1 << 3) | (1 << 2) | (1 << 1) | 0x1;

	return ret;
}


int i2c_regRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_out, uint32_t len)
{
	int ret;

	ret = i2c_busWrite(dev_addr, &reg_addr, sizeof(reg_addr));
	if (ret < 0) {
		return ret;
	}

	return i2c_busRead(dev_addr, data_out, len);
}


static void i2c_initCtrl(void)
{
	uint32_t reg = *(i2c.base + I2C_CR) & ~0xffff;

	/* I2C clock frequency to 100 kHz; base: CPU_1X_Clock = 111 MHz
	   -divisor_a = 2, -divisor_b = 16 */
	reg |= (2 << 14) | (16 << 8);

	/* clear FIFO, - 7bit addressing */
	reg |= (1 << 6) | (1 << 2);

	*(i2c.base + I2C_CR) = reg;

	/* initialize timeout; wait 255 SCL cycles when the SCL is held Low, before generating a timeout interrupt. */
	*(i2c.base + I2C_TIMEOUT) = 255;
}


static int i2c_setPin(int pin, unsigned int cfg)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_mio;
	pctl.mio.pin = pin;
	pctl.mio.disableRcvr = (cfg >> 13) & 1;
	pctl.mio.pullup = (cfg >> 12) & 1;
	pctl.mio.ioType = (cfg >> 9) & 7;
	pctl.mio.speed = (cfg >> 8) & 1;
	pctl.mio.l3 = (cfg >> 5) & 7;
	pctl.mio.l2 = (cfg >> 3) & 3;
	pctl.mio.l1 = (cfg >> 2) & 1;
	pctl.mio.l0 = (cfg >> 1) & 1;
	pctl.mio.triEnable = (cfg >> 0) & 1;

	return platformctl(&pctl);
}


static int i2c_initPins(void)
{
	int err;
	unsigned int i;
	const i2c_info_t *info = &devsInfo[i2c.devID];

	for (i = 0; i < sizeof(info->pins) / sizeof(info->pins[0]); i++) {
		/* skip not configured pins */
		if (info->pins[i].pin < 0) {
			continue;
		}

		err = i2c_setPin(info->pins[i].pin, info->pins[i].cfg);
		if (err < 0) {
			return err;
		}
	}

	return EOK;
}


static int i2c_initClk(void)
{
	platformctl_t pctl;

	/* enable AMBA clock */
	pctl.type = pctl_ambaclock;
	pctl.ambaclock.dev = pctl_amba_i2c0_clk + i2c.devID;
	pctl.ambaclock.state = 1;

	return platformctl(&pctl);
}


static int i2c_reset(void)
{
	platformctl_t pctl;
	int err;

	pctl.action = pctl_get;
	pctl.type = pctl_devreset;
	pctl.devreset.dev = pctl_ctrl_i2c_rst;
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	pctl.action = pctl_set;
	pctl.devreset.state |= (1 << (i2c.devID + 2)) | (1 << i2c.devID);
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	pctl.devreset.state &= ~((1 << (i2c.devID + 2)) | (1 << i2c.devID));
	err = platformctl(&pctl);

	return err;
}


int i2c_init(unsigned int dev_no)
{
	int err;

	if (dev_no >= sizeof(devsInfo) / sizeof(devsInfo[0])) {
		return -ENODEV;
	}

	i2c.devID = dev_no;

	/* reset controller */
	err = i2c_reset();
	if (err < 0) {
		return err;
	}

	/* initialize clock */
	err = i2c_initClk();
	if (err < 0) {
		return err;
	}

	/* configure pins */
	err = i2c_initPins();
	if (err < 0) {
		return err;
	}

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

	/* controller initialization */
	i2c_initCtrl();

	interrupt(devsInfo[dev_no].irq, i2c_isr, NULL, i2c.cond, &i2c.inth);

	i2c.initialized = 1;

	return EOK;
}
