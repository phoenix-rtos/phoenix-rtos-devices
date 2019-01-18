/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI lib
 *
 * Copyright 2018 Phoenix Systems
 * Author: Daniel Sawka, Krystian Wasik
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <phoenix/arch/imx6ull.h>

#include "ecspi.h"


enum { rxdata = 0, txdata, conreg, configreg, intreg, dmareg, statreg, periodreg, testreg, msgdata = 16 };

typedef struct {
	volatile uint32_t *base;
	uint8_t chan_msk;

	handle_t inth;
	handle_t cond;
	handle_t irqlock;
} ecspi_t;

typedef struct {
	int pctl;
	char val;
} ecspi_pctl_t;


static const addr_t ecspi_addr[4] = { 0x2008000, 0x200C000, 0x2010000, 0x2014000 };
static const unsigned int ecspi_intr_number[4] = { 63, 64, 65, 66 };

ecspi_pctl_t ecspi_pctl_mux[4][7] = {
	{ { pctl_mux_csi_d7,      3 }, { pctl_mux_csi_d6,     3 }, { pctl_mux_csi_d4,    3 }, { pctl_mux_csi_d5,     3 },
	  { pctl_mux_lcd_d5,      8 }, { pctl_mux_lcd_d6,     8 }, { pctl_mux_lcd_d7,    8 } },
	{ { pctl_mux_csi_d3,      3 }, { pctl_mux_csi_d2,     3 }, { pctl_mux_csi_d0,    3 }, { pctl_mux_csi_d1,     3 },
	  { pctl_mux_lcd_hsync,   8 }, { pctl_mux_lcd_vsync,  8 }, { pctl_mux_lcd_rst,   8 } },
	{ { pctl_mux_uart2_rts,   8 }, { pctl_mux_uart2_cts,  8 }, { pctl_mux_uart2_rx,  8 }, { pctl_mux_uart2_tx,   8 },
	  { pctl_mux_nand_ale,    8 }, { pctl_mux_nand_re,    8 }, { pctl_mux_nand_we,   8 } },
	{ { pctl_mux_enet2_txclk, 3 }, { pctl_mux_enet2_txen, 3 }, { pctl_mux_enet2_tx1, 3 }, { pctl_mux_enet2_rxer, 3 },
	  { pctl_mux_nand_d1,     8 }, { pctl_mux_nand_d2,    8 }, { pctl_mux_nand_d3,   8 } }
};

ecspi_pctl_t ecspi_pctl_isel[4][4] = {
	{ { pctl_isel_ecspi1_miso, 1 }, { pctl_isel_ecspi1_mosi, 1 }, { pctl_isel_ecspi1_sclk, 1 }, { pctl_isel_ecspi1_ss0, 1 } },
	{ { pctl_isel_ecspi2_miso, 0 }, { pctl_isel_ecspi2_mosi, 1 }, { pctl_isel_ecspi2_sclk, 0 }, { pctl_isel_ecspi2_ss0, 0 } },
	{ { pctl_isel_ecspi3_miso, 0 }, { pctl_isel_ecspi3_mosi, 0 }, { pctl_isel_ecspi3_sclk, 0 }, { pctl_isel_ecspi3_ss0, 0 } },
	{ { pctl_isel_ecspi4_miso, 0 }, { pctl_isel_ecspi4_mosi, 0 }, { pctl_isel_ecspi4_sclk, 0 }, { pctl_isel_ecspi4_ss0, 0 } }
};

uint32_t ecspi_pctl_clk[4] = { pctl_clk_ecspi1, pctl_clk_ecspi2, pctl_clk_ecspi3, pctl_clk_ecspi4 };

static ecspi_t ecspi[4] = {0};


static int ecspi_irqHandler(unsigned int n, void *arg)
{
	ecspi_t *e = &ecspi[(int) arg];

	/* Disable Transfer Completed interrupt. */
	*(e->base + intreg) &= ~(1 << 7);

	return 1;
}


static void ecspi_setBurst(int dev_no, uint16_t burst)
{
	ecspi_t *e;

	if (burst % 32 == 1) {
		printf("ecspi: burst length of 32n + 1 is erroneous in hardware itself\n");
		return;
	}

	e = &ecspi[dev_no - 1];
	*(e->base + conreg) = (*(e->base + conreg) & ~(0xFFF << 20)) | ((burst - 1) << 20);
}


static void set_mux(int dev_no, uint8_t chan_msk)
{
	platformctl_t ctl;
	int i;

	ctl.action = pctl_set;
	ctl.type = pctl_iomux;

	ctl.iomux.sion = 0;

	for (i = 0; i < 7; i++) {
		/* Skip SS lines not enabled in chan_msk */
		if (i < 3 || (chan_msk & (1 << (i - 3)))) {
			ctl.iomux.mux = ecspi_pctl_mux[(dev_no - 1)][i].pctl;
			ctl.iomux.mode = ecspi_pctl_mux[(dev_no - 1)][i].val;
			platformctl(&ctl);
		}
	}

	ctl.action = pctl_set;
	ctl.type = pctl_ioisel;

	for (i = 0; i < 3; i++) {
		if (i < 3 || (chan_msk & (1 << (i - 3)))) {
			ctl.ioisel.isel = ecspi_pctl_isel[(dev_no - 1)][i].pctl;
			ctl.ioisel.daisy = ecspi_pctl_isel[(dev_no - 1)][i].val;
			platformctl(&ctl);
		}
	}
}


static void set_clk(int dev_no)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devclock;
	ctl.devclock.dev = ecspi_pctl_clk[dev_no - 1];
	ctl.devclock.state = 0x03;

	platformctl(&ctl);
}


static void ecspi_writeFifo(int dev_no, const uint8_t *out, uint32_t len)
{
	uint32_t word;
	ecspi_t *e = &ecspi[dev_no - 1];
	uint8_t fill_len = len % 4;

	if (fill_len > 0) {
		word = 0;

		while (fill_len > 0) {
			word = (word << 8) | *out++;
			fill_len--;
			len--;
		}

		*(e->base + txdata) = word;
	}

	while (len > 0) {
		word = out[3] | ((uint32_t) out[2] << 8) | ((uint32_t) out[1] << 16) | ((uint32_t) out[0] << 24);

		*(e->base + txdata) = word;

		len -= 4;
		out += 4;
	}
}


static void ecspi_readFifo(int dev_no, uint8_t *in, uint32_t len)
{
	uint32_t word;
	ecspi_t *e = &ecspi[dev_no - 1];
	uint8_t len_fill = len % 4;

	if (len_fill > 0) {
		word = *(e->base + rxdata);

		for (int i = len_fill; i >= 1; i--) {
			*in++ = (word >> ((i - 1) * 8)) & 0xFF;
		}

		len -= len_fill;
	}

	while (len > 0) {
		word = *(e->base + rxdata);

		*in++ = (word >> 24) & 0xFF;
		*in++ = (word >> 16) & 0xFF;
		*in++ = (word >> 8) & 0xFF;
		*in++ = (word) & 0xFF;

		len -= 4;
	}
}


int ecspi_setChannel(int dev_no, uint8_t chan)
{
	ecspi_t *e = &ecspi[dev_no - 1];

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	if (chan > 3) {
		return -2;
	}

	if (!(e->chan_msk & (1 << chan))) {
		return -3;
	}

	*(e->base + conreg) = (*(e->base + conreg) & ~(0x03 << 18)) | (chan << 18);

	return 0;
}


int ecspi_setMode(int dev_no, uint8_t chan, uint8_t mode)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	if (chan > 3) {
		return -2;
	}

	e = &ecspi[dev_no - 1];
	*(e->base + configreg) = (*(e->base + configreg) & ~(0x03)) | ((mode & 0x01) << chan) | ((mode & 0x02) << (chan + 4));

	return 0;
}


int ecspi_setClockDiv(int dev_no, uint8_t pre, uint8_t post)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	/* Default clock for ECSPI is 60 MHz. */
	e = &ecspi[dev_no - 1];
	*(e->base + conreg) = (*(e->base + conreg) & ~(0xFF << 8)) | ((pre & 0x0F) << 12) | ((post & 0x0F) << 8);

	return 0;
}


int ecspi_setCSDelay(int dev_no, uint8_t delay)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	e = &ecspi[dev_no - 1];
	*(e->base + periodreg) = (*(e->base + periodreg) & ~(0x3F << 16)) | ((delay & 0x3F) << 16);

	return 0;
}


int ecspi_exchange(int dev_no, const uint8_t *out, uint8_t *in, uint16_t len)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	if (len > (64*4)) {
		return -2; /* Not supported yet */
	}

	e = &ecspi[dev_no - 1];

	ecspi_setBurst(dev_no, len * 8);
	ecspi_writeFifo(dev_no, out, len);

	mutexLock(e->irqlock);

	/* Enable Transfer Completed interrupt. */
	*(e->base + intreg) |= (1 << 7);
	/* Begin transmission. */
	*(e->base + conreg) |= (1 << 2);

	while (!(*(e->base + statreg) & (1 << 7))) {
		condWait(e->cond, e->irqlock, 0);
	}

	/* Clear Transfer Completed bit. */
	*(e->base + statreg) |= (1 << 7);

	mutexUnlock(e->irqlock);

	ecspi_readFifo(dev_no, in, len);

	return 0;
}


addr_t ecspi_getTxFifoPAddr(int dev_no)
{
	return ecspi_addr[dev_no - 1] + txdata * 4;
}


addr_t ecspi_getRxFifoPAddr(int dev_no)
{
	return ecspi_addr[dev_no - 1] + rxdata * 4;
}


int ecspi_init(int dev_no, uint8_t chan_msk)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	e = &ecspi[dev_no - 1];
	e->chan_msk = chan_msk;

	if ((e->base = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, ecspi_addr[dev_no - 1])) == MAP_FAILED) {
		printf("ecspi: could not map ecspi%d paddr %p.\n", dev_no, (void*) ecspi_addr[dev_no - 1]);
		return -1;
	}

	set_mux(dev_no, chan_msk);
	set_clk(dev_no);

	mutexCreate(&e->irqlock);
	condCreate(&e->cond);

	interrupt(ecspi_intr_number[dev_no - 1], ecspi_irqHandler, (void *) (dev_no - 1), e->cond, &e->inth);

	/* Enable ECSPI. Defaults: 8-bit burst, multi burst, mode 0, all master, no cock division */
	*(e->base + conreg) = 0x007000F1;
	*(e->base + configreg) = 0x00000F00;

	return 0;
}
