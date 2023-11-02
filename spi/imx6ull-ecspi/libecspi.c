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
#include <stddef.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <phoenix/arch/imx6ull.h>

#include "imx6ull-ecspi.h"


#define BYTES_2_RXTHRESHOLD(LEN) (((LEN) + 3) / 4 - 1)
#define BITS_2_BYTES_ROUND_UP(LEN) (((LEN) + 7) / 8)
#define GET_BURST_IN_BYTES(ECSPI) (BITS_2_BYTES_ROUND_UP((*((ECSPI)->base + conreg) >> 20) + 1))


enum { rxdata = 0, txdata, conreg, configreg, intreg, dmareg, statreg, periodreg, testreg, msgdata = 16 };

typedef enum { mode_sync_exchange, mode_async_write, mode_async_exchange, mode_async_periodical } ecspi_mode_t;

typedef struct {
	volatile uint32_t *base;
	uint8_t chan_msk;
	ecspi_mode_t mode;

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
	{ { pctl_mux_lcd_d23,     2 }, { pctl_mux_lcd_d22,    2 }, { pctl_mux_lcd_d20,   2 }, { pctl_mux_lcd_d21,    2 },
	  { pctl_mux_lcd_d5,      8 }, { pctl_mux_lcd_d6,     8 }, { pctl_mux_lcd_d7,    8 } },
	{ { pctl_mux_csi_d3,      3 }, { pctl_mux_csi_d2,     3 }, { pctl_mux_csi_d0,    3 }, { pctl_mux_csi_d1,     3 },
	  { pctl_mux_lcd_hsync,   8 }, { pctl_mux_lcd_vsync,  8 }, { pctl_mux_lcd_rst,   8 } },
	{ { pctl_mux_uart2_rts,   8 }, { pctl_mux_uart2_cts,  8 }, { pctl_mux_uart2_rx,  8 }, { pctl_mux_uart2_tx,   8 },
	  { pctl_mux_nand_ale,    8 }, { pctl_mux_nand_re,    8 }, { pctl_mux_nand_we,   8 } },
	{ { pctl_mux_enet2_txclk, 3 }, { pctl_mux_enet2_txen, 3 }, { pctl_mux_enet2_tx1, 3 }, { pctl_mux_enet2_rxer, 3 },
	  { pctl_mux_nand_d1,     8 }, { pctl_mux_nand_d2,    8 }, { pctl_mux_nand_d3,   8 } }
};

ecspi_pctl_t ecspi_pctl_isel[4][4] = {
	{ { pctl_isel_ecspi1_miso, 0 }, { pctl_isel_ecspi1_mosi, 0 }, { pctl_isel_ecspi1_sclk, 0 }, { pctl_isel_ecspi1_ss0, 0 } },
	{ { pctl_isel_ecspi2_miso, 0 }, { pctl_isel_ecspi2_mosi, 1 }, { pctl_isel_ecspi2_sclk, 0 }, { pctl_isel_ecspi2_ss0, 0 } },
	{ { pctl_isel_ecspi3_miso, 0 }, { pctl_isel_ecspi3_mosi, 0 }, { pctl_isel_ecspi3_sclk, 0 }, { pctl_isel_ecspi3_ss0, 0 } },
	{ { pctl_isel_ecspi4_miso, 0 }, { pctl_isel_ecspi4_mosi, 0 }, { pctl_isel_ecspi4_sclk, 0 }, { pctl_isel_ecspi4_ss0, 0 } }
};

uint32_t ecspi_pctl_clk[4] = { pctl_clk_ecspi1, pctl_clk_ecspi2, pctl_clk_ecspi3, pctl_clk_ecspi4 };

static ecspi_t ecspi[4] = {0};


#define RESET_ECSPI(ECSPI) do { \
	uint32_t reg_backup[3]; \
	reg_backup[0] = *((ECSPI)->base + configreg); \
	reg_backup[1] = *((ECSPI)->base + intreg); \
	reg_backup[2] = *((ECSPI)->base + periodreg); \
	*((ECSPI)->base + conreg) &= ~(1 << 0); \
	*((ECSPI)->base + conreg) |= (1 << 0); \
	*((ECSPI)->base + configreg) = reg_backup[0]; \
	*((ECSPI)->base + intreg) = reg_backup[1]; \
	*((ECSPI)->base + periodreg) = reg_backup[2]; \
} while (0)


static void readFifo(int dev_no, uint8_t *in, size_t len);


static void writeFifo(int dev_no, const uint8_t *out, size_t len);


static int ecspi_irqHandler(unsigned int n, void *arg)
{
	(void) n;

	ecspi_t *e = &ecspi[(int) arg];

	if (e->mode != mode_sync_exchange) {
		return -1;
	}
	else {
		/* Disable Transfer Completed interrupt. */
		*(e->base + intreg) &= ~(1 << 7);
		return 1;
	}
}


static int ecspi_irqHandlerAsync(unsigned int n, void *arg)
{
	(void) n;

	size_t count;
	int res = -1;
	int res_writer;

	ecspi_ctx_t *ctx = arg;
	ecspi_t *e = &ecspi[ctx->dev_no - 1];

	if (e->mode == mode_sync_exchange) {
		return -1;
	}

	if (*(e->base + statreg) & (1 << 7)) {
		/* Clear Transfer Completed bit. */
		*(e->base + statreg) |= (1 << 7);

		if (e->mode == mode_async_write) {
			/* Disable Transfer Completed interrupt. */
			*(e->base + intreg) &= ~(1 << 7);
			/* Discarding rxfifo words. The only ways are: read all words from rxfifo, or reset ECSPI. */
			RESET_ECSPI(e);

			res = 1;
		}
		else if (e->mode == mode_async_exchange) {
			/* Disable Transfer Completed interrupt. */
			*(e->base + intreg) &= ~(1 << 7);

			ctx->rx_count += GET_BURST_IN_BYTES(e);
			*(e->base + conreg) |= (1 << 2);

			res = 1;
		}
		else if (e->mode == mode_async_periodical) {
			count = GET_BURST_IN_BYTES(e);

			readFifo(ctx->dev_no, ctx->in_periodical, count);
			res_writer = ctx->writer_proc(ctx->in_periodical, count, ctx->out_periodical);

			if (res_writer < 0) {
				/* Disable Transfer Completed interrupt. */
				*(e->base + intreg) &= ~(1 << 7);
				ecspi_setSSDelay(ctx->dev_no, ctx->prev_wait_states);
				/* The only way to reset the internal period counter is to reenable the ECSPI. */
				RESET_ECSPI(e);
				res = 1;
			} else {
				writeFifo(ctx->dev_no, ctx->out_periodical, count);
				*(e->base + conreg) |= (1 << 2);
				res = -1;
			}
		}
	}

	return res;
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

	for (i = 0; i < 4; i++) {
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


static void writeFifo(int dev_no, const uint8_t *out, size_t len)
{
	uint32_t word;
	ecspi_t *e = &ecspi[dev_no - 1];
	size_t fill_len = len % 4;

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


static void readFifo(int dev_no, uint8_t *in, size_t len)
{
	uint32_t word;
	ecspi_t *e = &ecspi[dev_no - 1];
	size_t len_fill = len % 4;

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


int ecspi_readFifo(ecspi_ctx_t *ctx, uint8_t *buf, size_t len)
{
	if (ctx->dev_no < 1 || ctx->dev_no > 4) {
		return -1;
	}

	if (len > (64 * 4)) {
		return -2;
	}

	readFifo(ctx->dev_no, buf, len);

	return len;
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


int ecspi_setSSDelay(int dev_no, uint16_t delay)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	e = &ecspi[dev_no - 1];
	*(e->base + periodreg) = (*(e->base + periodreg) & ~0x7FFF) | (delay & 0x7FFF);

	return 0;
}


int ecspi_exchange(int dev_no, const uint8_t *out, uint8_t *in, size_t len)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	if (len > (64 * 4) || len == 0) {
		return -2;
	}

	e = &ecspi[dev_no - 1];
	// Wait until the previous transaction has ended.
	while ((*(e->base + conreg) & (1 << 2)) || (*(e->base + testreg) & 0x7F) != 0) {
		;
	}

	e->mode = mode_sync_exchange;

	/* Single burst mode */
	*(e->base + configreg) &= ~(0xF << 8);

	ecspi_setBurst(dev_no, len * 8);
	writeFifo(dev_no, out, len);

	mutexLock(e->irqlock);

	/* Clear Transfer Completed bit. */
	*(e->base + statreg) |= (1 << 7);
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

	readFifo(dev_no, in, len);

	return 0;
}



int ecspi_exchangeBusy(int dev_no, const uint8_t *out, uint8_t *in, size_t len)
{
	ecspi_t *e;

	if (dev_no < 1 || dev_no > 4) {
		return -1;
	}

	if (len > (64 * 4) || len == 0) {
		return -2;
	}

	e = &ecspi[dev_no - 1];

	/* Wait until the previous transaction has completed. */
	while ((*(e->base + conreg) & (1 << 2)) || (*(e->base + testreg) & 0x7F) != 0) {
		;
	}

	ecspi_setBurst(dev_no, len * 8);
	writeFifo(dev_no, out, len);

	/* Clear Transfer Completed bit. */
	*(e->base + statreg) |= (1 << 7);
	/* Begin transmission. */
	*(e->base + conreg) |= (1 << 2);

	/* Wait until the transaction has completed. */
	while (!(*(e->base + statreg) & (1 << 7))) {
		;
	}

	/* Clear Transfer Completed bit. */
	*(e->base + statreg) |= (1 << 7);

	readFifo(dev_no, in, len);

	return 0;
}


int ecspi_registerContext(int dev_no, ecspi_ctx_t *ctx, handle_t cond)
{
	*ctx = (ecspi_ctx_t) {
		.dev_no = dev_no,
	};

	return interrupt(ecspi_intr_number[dev_no - 1], ecspi_irqHandlerAsync, (void *) ctx, cond, &ctx->inth);
}


int ecspi_exchangeAsync(ecspi_ctx_t *ctx, const uint8_t *out, size_t len)
{
	ecspi_t *e;
	uint16_t current_burst;
	uint8_t txfifo_word_cnt;
	uint8_t rxfifo_word_cnt;
	size_t written = 0;

	if (ctx->dev_no < 1 || ctx->dev_no > 4) {
		return -1;
	}

	if (len > (64 * 4)) {
		return -2;
	}

	e = &ecspi[ctx->dev_no - 1];
	txfifo_word_cnt = *(e->base + testreg) & 0x7F;

	if (!(*(e->base + conreg) & (1 << 2)) && txfifo_word_cnt == 0) {
		e->mode = mode_async_exchange;
		ctx->rx_count = 0;

		ecspi_setBurst(ctx->dev_no, len * 8);

		/* One burst mode */
		*(e->base + configreg) &= ~(0xF << 8);
		/* Clear Transfer Completed bit. */
		*(e->base + statreg) |= (1 << 7);
		/* Enable Transfer Completed interrupt. */
		*(e->base + intreg) |= (1 << 7);

		writeFifo(ctx->dev_no, out, len);
		*(e->base + conreg) |= (1 << 2);

		written = len;
	}
	else if (e->mode == mode_async_exchange) {
		current_burst = (*(e->base + conreg) >> 20) + 1;
		rxfifo_word_cnt = (*(e->base + testreg) >> 8) & 0x7F;

		if (current_burst == (len * 8) && (int) (len / 4) <= (64 - txfifo_word_cnt) && (int) (len / 4) < (64 - rxfifo_word_cnt)) {
			writeFifo(ctx->dev_no, out, len);
			*(e->base + conreg) |= (1 << 2);
			written = len;
		}
	}

	return written;
}


int ecspi_exchangePeriodically(ecspi_ctx_t *ctx, uint8_t *out, uint8_t *in, size_t len, unsigned int wait_states, ecspi_writerProc_t writer_proc)
{
	ecspi_t *e;
	uint8_t txfifo_word_cnt;
	size_t written = 0;

	if (ctx->dev_no < 1 || ctx->dev_no > 4) {
		return -1;
	}

	if (len > (64 * 4)) {
		return -2;
	}

	e = &ecspi[ctx->dev_no - 1];
	txfifo_word_cnt = *(e->base + testreg) & 0x7F;

	if (!(*(e->base + conreg) & (1 << 2)) && txfifo_word_cnt == 0 && !(*(e->base + intreg) & (1 << 7))) {
		e->mode = mode_async_periodical;
		ecspi_setBurst(ctx->dev_no, len * 8);
		ctx->prev_wait_states = (*(e->base + periodreg) & 0x7FFF);
		ecspi_setSSDelay(ctx->dev_no, wait_states);
		ctx->writer_proc = writer_proc;
		ctx->out_periodical = out;
		ctx->in_periodical = in;

		/* One burst mode */
		*(e->base + configreg) &= ~(0xF << 8);
		/* Clear Transfer Completed bit. */
		*(e->base + statreg) |= (1 << 7);
		/* Enable Transfer Completed interrupt. */
		*(e->base + intreg) |= (1 << 7);

		writeFifo(ctx->dev_no, out, len);
		*(e->base + conreg) |= (1 << 2);

		written = len;
	}

	return written;
}


int ecspi_writeAsync(ecspi_ctx_t *ctx, const uint8_t *out, size_t len)
{
	ecspi_t *e;
	uint16_t current_burst;
	uint8_t txfifo_word_cnt;
	size_t written = 0;

	if (ctx->dev_no < 1 || ctx->dev_no > 4) {
		return -1;
	}

	if (len > (64 * 4)) {
		return -2;
	}

	e = &ecspi[ctx->dev_no - 1];

	txfifo_word_cnt = *(e->base + testreg) & 0x7F;

	if (!(*(e->base + conreg) & (1 << 2)) && txfifo_word_cnt == 0) {
		e->mode = mode_async_write;

		/* Multiple (auto) burst mode */
		*(e->base + configreg) |= (0xF << 8);
		/* Clear Transfer Completed bit. */
		*(e->base + statreg) |= (1 << 7);
		/* Enable Transfer Completed interrupt. */
		*(e->base + intreg) |= (1 << 7);

		ecspi_setBurst(ctx->dev_no, len * 8);
		writeFifo(ctx->dev_no, out, len);
		*(e->base + conreg) |= (1 << 2);
		written = len;
	}
	else if (e->mode == mode_async_write) {
		current_burst = (*(e->base + conreg) >> 20) + 1;

		if (current_burst == (len * 8) && (int) (len / 4) <= (64 - txfifo_word_cnt)) {
			writeFifo(ctx->dev_no, out, len);
			*(e->base + conreg) |= (1 << 2);

			written = len;
		}
	}

	return written;
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
	e->mode = mode_sync_exchange;

	if ((e->base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, ecspi_addr[dev_no - 1])) == MAP_FAILED) {
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
