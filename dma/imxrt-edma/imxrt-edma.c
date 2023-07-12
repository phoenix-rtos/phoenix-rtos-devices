/*
 * Phoenix-RTOS
 *
 * i.MX RT eDMA driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Krystian Wasik
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <unistd.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#if defined(__CPU_IMXRT117X)
#include <phoenix/arch/imxrt1170.h>
#elif defined(__CPU_IMXRT106X)
#include <phoenix/arch/imxrt.h>
#else
#error "Unsupported CPU"
#endif

#include "edma.h"

#if defined(__CPU_IMXRT106X)
#define EDMA_BASE_ADDR ((void *)0x400e8000)
#define EDMA_CLK       pctl_clk_dma
#elif defined(__CPU_IMXRT117X)
#define EDMA_BASE_ADDR ((void *)0x40070000)
#define EDMA_CLK       pctl_clk_bus
#else
#error "Unsupported CPU"
#endif

#define DMA_MUX_BASE_ADDR EDMA_BASE_ADDR + 0x4000


struct edma_regs_s {
	uint32_t cr;
	uint32_t es;
	uint32_t reserved0;
	uint32_t erq;
	uint32_t reserved1;
	uint32_t eei;
	uint8_t ceei;
	uint8_t seei;
	uint8_t cerq;
	uint8_t serq;
	uint8_t cdne;
	uint8_t ssrt;
	uint8_t cerr;
	uint8_t cint;
	uint32_t reserved2;
	uint32_t _int;
	uint32_t reserved3;
	uint32_t err;
	uint32_t reserved4;
	uint32_t hrs;
	uint32_t reserved5[3];
	uint32_t ears;
	uint32_t reserved6[46];
	uint8_t dchpri3;
	uint8_t dchpri2;
	uint8_t dchpri1;
	uint8_t dchpri0;
	uint8_t dchpri7;
	uint8_t dchpri6;
	uint8_t dchpri5;
	uint8_t dchpri4;
	uint8_t dchpri11;
	uint8_t dchpri10;
	uint8_t dchpri9;
	uint8_t dchpri8;
	uint8_t dchpri15;
	uint8_t dchpri14;
	uint8_t dchpri13;
	uint8_t dchpri12;
	uint8_t dchpri19;
	uint8_t dchpri18;
	uint8_t dchpri17;
	uint8_t dchpri16;
	uint8_t dchpri23;
	uint8_t dchpri22;
	uint8_t dchpri21;
	uint8_t dchpri20;
	uint8_t dchpri27;
	uint8_t dchpri26;
	uint8_t dchpri25;
	uint8_t dchpri24;
	uint8_t dchpri31;
	uint8_t dchpri30;
	uint8_t dchpri29;
	uint8_t dchpri28;
	uint32_t reserved7[952];
	struct edma_tcd_s tcd[EDMA_NUM_OF_CHANNELS];
};

struct dmamux_regs_s {
	uint32_t chcfg[32];
};

static volatile struct dmamux_regs_s* dmamux_regs;
static volatile struct edma_regs_s* edma_regs;

void dmamux_set_source(uint8_t channel, uint8_t source)
{
	/*
	 * TODO: Should we verify that this is the only channel with this source?
	 * Note from the datasheet:
	 * "Setting multiple CHCFG registers with the same source value will result
	 * in unpredictable behavior."
	 */
	uint32_t tmp = dmamux_regs->chcfg[channel];
	tmp &= ~0x7f;
	tmp |= source & 0x7f;
	dmamux_regs->chcfg[channel] = tmp;
}

void dmamux_channel_enable(uint8_t channel)
{
	dmamux_regs->chcfg[channel] |= 1 << 31;
}

void dmamux_channel_disable(uint8_t channel)
{
	dmamux_regs->chcfg[channel] &= ~(1 << 31);
}

int dmamux_channel_is_enabled(uint8_t channel)
{
	return !!(dmamux_regs->chcfg[channel] & (1 << 31));
}

int edma_init(int (*error_isr)(unsigned int n, void *arg))
{
	int init = 1;
	int res;
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = EDMA_CLK;

	if ((res = platformctl(&pctl)) != 0) {
		return res;
	}

#if defined(__CPU_IMXRT106X)
	if (pctl.devclock.state == clk_state_run) {
#elif defined(__CPU_IMXRT117X)
	if (pctl.devclock.state == 1) {
#else
#error "Unsupported CPU"
	if (!0) { /* only for syntax correctness */
#endif
		/* someone else already started initialization - sleep 1 ms make sure it has finished */
		usleep(1000);
		init = 0;
	}

	if (init) {
		pctl.action = pctl_set;

#if defined(__CPU_IMXRT106X)
		pctl.devclock.state = clk_state_run;
#elif defined(__CPU_IMXRT117X)
		pctl.devclock.state = 1;
#else
#error "Unsupported CPU"
#endif

		if ((res = platformctl(&pctl)) != 0) {
			return res;
		}
	}

	edma_regs = EDMA_BASE_ADDR;
	dmamux_regs = DMA_MUX_BASE_ADDR;

	if (init) {
		/* Disable all channels, clear interrupt and error flags */
		edma_regs->erq = 0;
		edma_regs->_int = 0xffffffff;
		edma_regs->err = 0xffffffff;

		/*
		 * Use round robin for group and channel arbitration,
		 * enable minor loop mapping
		 */
		edma_regs->cr = (1 << 2) | (1 << 3) | (1 << 7);

		/* Set halt on error bit */
		edma_regs->cr |= (1 << 4);

		/* Stall channel activation in debug mode */
		edma_regs->cr |= 1 << 1;
	}

	/* Install error interrupts handler */
	unsigned handle;
	interrupt(EDMA_ERROR_IRQ, error_isr, NULL, 0, &handle);

	if (init) {
		/* Enable all error interrupts */
		edma_regs->eei = 0xffffffff;
	}

	return 0;
}

void edma_copy_tcd(volatile struct edma_tcd_s *dst,
	const volatile struct edma_tcd_s *src)
{
	/* Copy manually since memcpy can not be used for volatile memory */
	dst->saddr         = src->saddr;
	dst->soff          = src->soff;
	dst->attr          = src->attr;
	dst->nbytes_mlno   = src->nbytes_mlno;
	dst->slast         = src->slast;
	dst->daddr         = src->daddr;
	dst->doff          = src->doff;
	dst->citer_elinkno = src->citer_elinkno;
	dst->dlast_sga     = src->dlast_sga;
	dst->csr           = src->csr;
	dst->biter_elinkno = src->biter_elinkno;
}

int edma_install_tcd(const volatile struct edma_tcd_s* tcd, uint8_t channel)
{
	if (channel >= EDMA_NUM_OF_CHANNELS)
		return -1;

	/* Clear DONE bit first, otherwise ESG cannot be set */
	edma_regs->tcd[channel].csr = 0;

	edma_copy_tcd(&edma_regs->tcd[channel], tcd);

	return 0;
}

int edma_read_tcd(volatile struct edma_tcd_s *tcd, uint8_t channel)
{
	if (channel >= EDMA_NUM_OF_CHANNELS) {
		return -1;
	}

	edma_copy_tcd(tcd, &edma_regs->tcd[channel]);

	return 0;
}

/* Address must be 32-byte aligned */
#define TCD_REQUIRED_ALIGNMENT_MASK (0x1f)

int edma_initialize_tcd_ring(const struct edma_tcd_s* prototype,
	volatile struct edma_tcd_s** tcds, unsigned cnt,
	unsigned src_offset, unsigned dst_offset)
{
	for (int i = 0; i < cnt; i++) {

		/* Make sure TCD is properly aligned */
		if (((uint32_t)tcds[i] & TCD_REQUIRED_ALIGNMENT_MASK) != 0)
			return -1;

		/* Copy common part of TCD */
		edma_copy_tcd(tcds[i], prototype);

		/* Fill-in the rest */
		tcds[i]->dlast_sga = (uint32_t)tcds[(i + 1) % cnt];
		tcds[i]->saddr     = prototype->saddr + i*src_offset;
		tcds[i]->daddr     = prototype->daddr + i*dst_offset;
	}

	return 0;
}

/* Returns value of SSIZE/DSIZE bits for transfer size passed as a param */
uint8_t edma_get_tcd_attr_xsize(uint8_t num_of_bytes)
{
	switch (num_of_bytes) {
	default:
	case 1:  return 0x0;
	case 2:  return 0x1;
	case 4:  return 0x2;
	case 8:  return 0x3;
	case 32: return 0x5;
	}
}

int edma_is_hw_req_pending(unsigned channel)
{
	return edma_regs->hrs & (1 << channel);
}

void edma_channel_enable(unsigned channel)
{
	edma_regs->serq = channel | (1 << 6);
}

void edma_channel_disable(unsigned channel)
{
	edma_regs->cerq |= channel;
}

void edma_clear_interrupt(unsigned channel)
{
	edma_regs->cint |= channel;
}

void edma_clear_error(unsigned channel)
{
	edma_regs->cerr |= channel;
}

void edma_software_request_start(int channel)
{
	edma_regs->ssrt = channel & 0x1F;
}

uint32_t edma_error_status(unsigned channel)
{
	return edma_regs->es;
}

uint32_t edma_error_channel(void)
{
	return edma_regs->err;
}
