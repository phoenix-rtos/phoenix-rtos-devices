/*
 * Phoenix-RTOS
 *
 * Zynq-7000 AXI SPI controller (master mode only)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski, Aleksander Kaminski
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

#include "libzynq7000-xspi.h"


/* SPI registers */
#define SPI_SRR   16 /* Software reset register */
#define SPI_CR    24 /* Control register */
#define SPI_SR    25 /* Status register */
#define SPI_DTR   26 /* Data transfer register */
#define SPI_DRR   27 /* Data receive register */
#define SPI_SSR   28 /* Slave select register */
#define SPI_TFIFO 29 /* Trasnmit fifo ocupancy register */
#define SPI_RFIFO 30 /* Receive fifo ocupancy register */
#define SPI_DGIER 7  /* Global interrupt enable register */
#define SPI_ISR   8  /* Interrupt status register */
#define SPI_IER   10 /* Interrupt enable register */

/* SPI definitions */
#define SPI_FIFO_SIZE 256 /* SPI TX and RX FIFO size */


typedef struct {
	const unsigned int id;   /* SPI controller ID */
	const unsigned int irq;  /* SPI controller IRQ */
	const addr_t paddr;      /* SPI controller base physical address */
	volatile uint32_t *base; /* SPI registers base address */
	handle_t lock;           /* SPI IRQ mutex */
	handle_t cond;           /* SPI IRQ cond */
	handle_t inth;           /* SPI IRQ handle */
} spi_t;


/* SPI controllers */
static spi_t devs[] = {
	{
		.id = 0,             /* SPI0 */
		.irq = 64,           /* SPI0 IRQ */
		.paddr = 0x41e00000, /* SPI0 base physical address */
	}
};


static inline void spi_dmb(void)
{
	__asm__ volatile ("dmb");
}


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

	/* TX FIFO empty */
	if (*(spi->base + SPI_ISR) & (1 << 2)) {
		return 1;
	}

	/* Should never happen */
	return -1;
}


int spi_getMode(unsigned int dev, unsigned char *mode)
{
	unsigned int cfg;
	spi_t *spi;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	*mode &= ~(SPI_CPHA | SPI_CPOL);

	cfg = *(spi->base + SPI_CR);

	if (cfg & (1 << 4)) {
		*mode |= SPI_CPHA;
	}

	if (cfg & (1 << 3)) {
		*mode |= SPI_CPOL;
	}

	return EOK;
}


int spi_setMode(unsigned int dev, unsigned char mode)
{
	unsigned int cfg;
	spi_t *spi;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
	}

	cfg = *(spi->base + SPI_CR);
	cfg &= ~((1 << 4) | (1 << 3));

	if (mode & SPI_CPHA) {
		cfg |= (1 << 4);
	}

	if (mode & SPI_CPOL) {
		cfg |= (1 << 3);
	}

	*(spi->base + SPI_CR) = cfg;
	spi_dmb();

	return EOK;
}


int spi_getSpeed(unsigned int dev, unsigned int *speed)
{
	/* SW speed change is not supported by IP, we don't know the speed */
	return -ENOSYS;
}


int spi_setSpeed(unsigned int dev, unsigned int speed)
{
	/* SW speed change is not supported by IP, return OK for compatibility */
	return EOK;
}


static void spi_select(spi_t *spi, unsigned int ss, unsigned int state)
{
	/* Check for external SS control */
	if (ss != SPI_SS_EXTERNAL) {
		if (state != 0) {
			*(spi->base + SPI_SSR) |= (1 << ss);
		}
		else {
			*(spi->base + SPI_SSR) &= ~(1 << ss);
		}
	}
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

	if ((ss != SPI_SS_EXTERNAL) && ((ss >= SPI_SS_COUNT))) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	spi_select(spi, ss, 0);

	while (len > 0) {
		/* FIFO reset, master inhibit */
		*(spi->base + SPI_CR) |= (1 << 6) | (1 << 5) | (1 << 8);
		spi_dmb();

		/* Write to TX FIFO */
		for (n = 0; (n < SPI_FIFO_SIZE) && (len > 0); len--, n++) {
			if ((*(spi->base + SPI_SR) & (1 << 3)) != 0) {
				printf("Full\n");
				/* TX full */
				break;
			}

			if ((obuff != NULL) && (olen > 0)) {
				data = *(obuff++);
				olen--;
			}
			else {
				data = 0;
			}
			*(spi->base + SPI_DTR) = data;
			spi_dmb();
		}

		/* Clear status */
		*(spi->base + SPI_ISR) = *(spi->base + SPI_ISR);
		spi_dmb();

		/* Start master xfer */
		*(spi->base + SPI_CR) &= ~(1 << 8);
		spi_dmb();

		/* Wait until TX FIFO is empty */
		/* spi->lock mutex is locked in spi_init() */
		while (!(*(spi->base + SPI_ISR) & (1 << 2))) {
			condWait(spi->cond, spi->lock, 0);
		}

		/* Read from RX FIFO */
		while (n > 0) {
			data = *(spi->base + SPI_DRR) & 0xff;
			if (iskip > 0) {
				iskip--;
			}
			else if ((ibuff != NULL) && (ilen > 0)) {
				*(ibuff++) = data;
				ilen--;
			}
			n--;
		}
	}

	spi_select(spi, ss, 1);

	return EOK;
}


int spi_init(unsigned int dev)
{
	spi_t *spi;
	int err;

	spi = spi_get(dev);
	if (spi == NULL) {
		return -ENODEV;
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

	/* Reset controller */
	*(spi->base + SPI_SRR) = 0xa;
	spi_dmb();

	/* Manual SS, SPI master mode, mode 0 reset FIFOs */
	*(spi->base + SPI_CR) = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 2) | (1 << 1);
	spi_dmb();

	/* Disable all interrupts and enable global interrupt gate */
	*(spi->base + SPI_IER) = 1 << 2;
	*(spi->base + SPI_DGIER) = 1U << 31;
	spi_dmb();

	/* Lock IRQ mutex and attach IRQ handler */
	mutexLock(spi->lock);
	interrupt(spi->irq, spi_isr, spi, spi->cond, &spi->inth);

	return EOK;
}
