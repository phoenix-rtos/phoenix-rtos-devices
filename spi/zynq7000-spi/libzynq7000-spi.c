/*
 * Phoenix-RTOS
 *
 * Zynq-7000 SPI controller (master mode only)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <spi.h>
#include <stdint.h>

#include <sys/interrupt.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <phoenix/arch/zynq7000.h>
#include <board_config.h>


/* SPI registers */
#define SPI_CR  0  /* Configuration register */
#define SPI_SR  1  /* Interrupt status register */
#define SPI_IER 2  /* Interrupt enable register */
#define SPI_IDR 3  /* Interrupt disable register */
#define SPI_IMR 4  /* Interrupt mask register */
#define SPI_ER  5  /* Enable controller register */
#define SPI_DR  6  /* Delay control register */
#define SPI_TXD 7  /* Transmit data register */
#define SPI_RXD 8  /* Receive data register */
#define SPI_SIC 9  /* Slave idle count register */
#define SPI_TXT 10 /* TX threshold register */
#define SPI_RXT 11 /* RX threshold register */


/* SPI definitions */
#define SPI_CLK_REF     200000000                    /* SPI reference clock (200 MHz) */
#define SPI_CLK_DIV_MIN 1                            /* Min clock divisor */
#define SPI_CLK_DIV_MAX 7                            /* Max clock divisor */
#define SPI_SCLK(div)   (SPI_CLK_REF / (2 << (div))) /* Returns SCLK for a given divisor */
#define SPI_SCLK_MIN    SPI_SCLK(SPI_CLK_DIV_MAX)    /* Min SCLK (781.25 kHz) */
#define SPI_SCLK_MAX    SPI_SCLK(SPI_CLK_DIV_MIN)    /* Max SCLK (50 MHz) */
#define SPI_FIFO_SIZE   128                          /* SPI TX and RX FIFO size */
#define SPI_SS_COUNT    3                            /* Max number of SPI slaves per controller */


/* MIO SPI pins configuration */
#define MIO_SCLK 0x22a0 /* SCLK - disable HSTL, LVCMOS18, SPI */
#define MIO_MOSI 0x22a0 /* MOSI - same as SCLK */
#define MIO_MISO 0x22a1 /* MISO - disable HSTL, LVCMOS18, SPI, disable output */
#define MIO_SS   0x32a0 /* SS   - disable HSTL, enable pull-up resistor, LVCMOS18, SPI */


typedef struct {
	const unsigned int id;    /* SPI controller ID */
	const unsigned int irq;   /* SPI controller IRQ */
	const addr_t paddr;       /* SPI controller base physical address */
	const struct {            /* SPI MIO pins configuration */
		int pin;              /* MIO pin */
		unsigned int cfg;     /* MIO pin configuration */
	} pins[SPI_SS_COUNT + 3]; /* SPI_SS_COUNT SS pins, SCLK, MOSI, MISO */

	volatile uint32_t *base; /* SPI registers base address */
	handle_t lock;           /* SPI IRQ mutex */
	handle_t cond;           /* SPI IRQ cond */
	handle_t inth;           /* SPI IRQ handle */
} spi_t;


/* SPI controllers */
static spi_t devs[] = {
	{
		.id = 0,             /* SPI0 */
		.irq = 58,           /* SPI0 IRQ */
		.paddr = 0xe0006000, /* SPI0 base physical address */
		.pins = {
			{ SPI0_SS0, MIO_SS },    /* SPI0 SS0 */
			{ SPI0_SS1, MIO_SS },    /* SPI0 SS1 */
			{ SPI0_SS2, MIO_SS },    /* SPI0 SS2 */
			{ SPI0_SCLK, MIO_SCLK }, /* SPI0 SCLK */
			{ SPI0_MOSI, MIO_MOSI }, /* SPI0 MOSI */
			{ SPI0_MISO, MIO_MISO }, /* SPI0 MISO */
		},
	},
	{
		.id = 1,             /* SPI1 */
		.irq = 81,           /* SPI1 IRQ */
		.paddr = 0xe0007000, /* SPI1 base physical address */
		.pins = {
			{ SPI1_SS0, MIO_SS },    /* SPI1 SS0 */
			{ SPI1_SS1, MIO_SS },    /* SPI1 SS1 */
			{ SPI1_SS2, MIO_SS },    /* SPI1 SS2 */
			{ SPI1_SCLK, MIO_SCLK }, /* SPI1 SCLK */
			{ SPI1_MOSI, MIO_MOSI }, /* SPI1 MOSI */
			{ SPI1_MISO, MIO_MISO }, /* SPI1 MISO */
		},
	},
};


static spi_t *spi_get(unsigned int dev)
{
	if (dev >= sizeof(devs) / sizeof(devs[0])) {
		return NULL;
	}

	return &devs[dev];
}


static int spi_isr(unsigned int n, void *arg)
{
	spi_t *spi = arg;
	unsigned int status;

	status = *(spi->base + SPI_SR);

	/* Clear all interrupts */
	*(spi->base + SPI_SR) = status;

	/* TX FIFO empty */
	if (status & (1 << 2)) {
		*(spi->base + SPI_IDR) = (1 << 2);
		return spi->cond;
	}

	return 0;
}


static unsigned char spi_getClkMode(spi_t *spi)
{
	unsigned char mode = 0;
	unsigned int cfg;

	cfg = *(spi->base + SPI_CR);

	if (cfg & (1 << 2)) {
		mode |= SPI_CPHA;
	}

	if (cfg & (1 << 1)) {
		mode |= SPI_CPOL;
	}

	return mode;
}


int spi_getMode(unsigned int dev, unsigned char *mode)
{
	spi_t *spi;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	if (mode == NULL) {
		return -EINVAL;
	}

	*mode = spi_getClkMode(spi);

	return EOK;
}


static void spi_setClkMode(spi_t *spi, unsigned char mode)
{
	unsigned int cfg;

	cfg = *(spi->base + SPI_CR);
	cfg &= ~((1 << 2) | (1 << 1));

	if (mode & SPI_CPHA) {
		cfg |= (1 << 2);
	}

	if (mode & SPI_CPOL) {
		cfg |= (1 << 1);
	}

	*(spi->base + SPI_CR) = cfg;
}


int spi_setMode(unsigned int dev, unsigned char mode)
{
	spi_t *spi;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	spi_setClkMode(spi, mode);

	return EOK;
}


static unsigned int spi_getClkSpeed(spi_t *spi)
{
	unsigned char divisor;
	unsigned int cfg;

	cfg = *(spi->base + SPI_CR);
	divisor = (cfg >> 3) & 7;

	return SPI_SCLK(divisor);
}


int spi_getSpeed(unsigned int dev, unsigned int *speed)
{
	spi_t *spi;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	if (speed == NULL) {
		return -EINVAL;
	}

	*speed = spi_getClkSpeed(spi);

	return EOK;
}


static void spi_setClkSpeed(spi_t *spi, unsigned char divisor)
{
	unsigned int cfg;

	cfg = *(spi->base + SPI_CR);
	cfg &= ~(7 << 3);
	cfg |= (divisor & 7) << 3;

	*(spi->base + SPI_CR) = cfg;
}


int spi_setSpeed(unsigned int dev, unsigned int speed)
{
	unsigned char divisor;
	spi_t *spi;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	for (divisor = SPI_CLK_DIV_MIN; (divisor < SPI_CLK_DIV_MAX) && (SPI_SCLK(divisor) > speed); divisor++)
		;

	spi_setClkSpeed(spi, divisor);

	return EOK;
}


static void spi_select(spi_t *spi, unsigned int ss, unsigned int state)
{
	unsigned int cfg;

	/* Check for external SS control */
	if (ss == SPI_SS_EXTERNAL) {
		return;
	}

	/* Raise SS line high */
	if (state > 0) {
		/* Deselect all */
		ss = 15;
	}
	/* Set SS line low */
	else {
		ss = (spi->pins[ss].pin - spi->id) % SPI_SS_COUNT;
		/* SS0 -> 0 */
		/* SS1 -> 1 */
		/* SS2 -> 3 */
		if (ss == 2) {
			ss++;
		}
	}

	cfg = *(spi->base + SPI_CR);
	cfg &= ~(15 << 10);
	cfg |= (ss << 10);

	*(spi->base + SPI_CR) = cfg;
}


int spi_xfer(unsigned int dev, unsigned int ss, const void *out, size_t olen, void *in, size_t ilen, size_t iskip)
{
	const unsigned char *obuff = out;
	unsigned char data, *ibuff = in;
	size_t n, len = max(olen, iskip + ilen);
	spi_t *spi;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	if ((ss != SPI_SS_EXTERNAL) && ((ss >= SPI_SS_COUNT) || (spi->pins[ss].pin < 0))) {
		return -EINVAL;
	}

	if (len == 0) {
		return EOK;
	}

	/* Select slave and enable controller */
	spi_select(spi, ss, 0);
	*(spi->base + SPI_ER) = 1;

	while (len > 0) {
		/* Write to TX FIFO */
		for (n = 0; (n < SPI_FIFO_SIZE) && (len > 0); len--, n++) {
			if ((obuff != NULL) && (olen > 0)) {
				data = *obuff++;
				olen--;
			}
			else {
				data = 0;
			}
			*(spi->base + SPI_TXD) = data;
		}

		/* Enable TX FIFO empty interrupt */
		*(spi->base + SPI_IER) = (1 << 2);

		/* Wait until TX FIFO is empty */
		/* spi->lock mutex is locked in spi_init() */
		while (!(*(spi->base + SPI_SR) & (1 << 2))) {
			condWait(spi->cond, spi->lock, 0);
		}

		/* Read from RX FIFO */
		while (n > 0) {
			/* RX FIFO not empty */
			if (*(spi->base + SPI_SR) & (1 << 4)) {
				data = *(spi->base + SPI_RXD) & 0xff;
				if (iskip > 0) {
					iskip--;
				}
				else if ((ibuff != NULL) && (ilen > 0)) {
					*ibuff++ = data;
					ilen--;
				}
				n--;
			}
		}
	}

	/* Disable controller and deselect slave */
	*(spi->base + SPI_ER) = 0;
	spi_select(spi, ss, 1);

	return EOK;
}


static int spi_setPin(int pin, unsigned int cfg)
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


static int spi_initPins(spi_t *spi)
{
	unsigned int i;
	int err;

	for (i = 0; i < sizeof(spi->pins) / sizeof(spi->pins[0]); i++) {
		/* Skip not configured pins */
		if (spi->pins[i].pin < 0) {
			continue;
		}

		err = spi_setPin(spi->pins[i].pin, spi->pins[i].cfg);
		if (err < 0) {
			return err;
		}
	}

	return EOK;
}


static int spi_initClk(spi_t *spi)
{
	platformctl_t pctl;
	int err;

	pctl.action = pctl_get;
	pctl.type = pctl_devclock;
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	switch (spi->id) {
		case 0:
			pctl.devclock.clkact0 = 1;
			break;

		case 1:
			pctl.devclock.clkact1 = 1;
			break;

		default:
			return -ENODEV;
	}

	/* Set reference clock to 200 MHz */
	pctl.action = pctl_set;
	pctl.devclock.dev = pctl_ctrl_spi_clk;
	pctl.devclock.srcsel = 0;   /* IO PLL source clock (1000 MHz) */
	pctl.devclock.divisor0 = 5; /* Divide by 5 */
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	/* Enable AMBA clock */
	pctl.type = pctl_ambaclock;
	pctl.ambaclock.dev = pctl_amba_spi0_clk + spi->id;
	pctl.ambaclock.state = 1;
	err = platformctl(&pctl);

	return err;
}


static int spi_reset(spi_t *spi)
{
	platformctl_t pctl;
	int err;

	pctl.action = pctl_get;
	pctl.type = pctl_devreset;
	pctl.devreset.dev = pctl_ctrl_spi_rst;
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	pctl.action = pctl_set;
	pctl.devreset.state |= (1 << (spi->id + 2)) | (1 << spi->id);
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	pctl.devreset.state &= ~((1 << (spi->id + 2)) | (1 << spi->id));
	err = platformctl(&pctl);

	return err;
}


int spi_init(unsigned int dev)
{
	spi_t *spi;
	int err;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	/* Reset controller */
	err = spi_reset(spi);
	if (err < 0) {
		return err;
	}

	/* Initialize clock */
	err = spi_initClk(spi);
	if (err < 0) {
		return err;
	}

	/* Configure pins */
	err = spi_initPins(spi);
	if (err < 0) {
		return err;
	}

	err = mutexCreate(&spi->lock);
	if (err < 0) {
		return err;
	}

	err = condCreate(&spi->cond);
	if (err < 0) {
		resourceDestroy(spi->lock);
		return err;
	}

	/* Map SPI registers */
	spi->base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, spi->paddr);
	if (spi->base == MAP_FAILED) {
		resourceDestroy(spi->cond);
		resourceDestroy(spi->lock);
		return -ENOMEM;
	}

	/* Manual SS, deselect all slaves, SPI master mode */
	*(spi->base + SPI_CR) = (1 << 14) | (15 << 10) | (1 << 0);

	/* SPI clock mode 0, SCLK 50 MHz */
	spi_setClkMode(spi, SPI_MODE0);
	spi_setClkSpeed(spi, SPI_CLK_DIV_MIN);

	/* Enable TX FIFO underflow and RX FIFO overflow interrupts */
	*(spi->base + SPI_IER) = (1 << 6) | (1 << 0);
	/* Disable other interrupts */
	*(spi->base + SPI_IDR) = (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);

	/* Lock IRQ mutex and attach IRQ handler */
	mutexLock(spi->lock);
	interrupt(spi->irq, spi_isr, spi, spi->cond, &spi->inth);

	return EOK;
}
