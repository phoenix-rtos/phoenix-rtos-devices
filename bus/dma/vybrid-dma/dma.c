/**
 * Enhanced Serial Audio Interface (ESAI) implementation for Vybrid
 *
 * Phoenix-RTOS
 * 
 * Operating system kernel
 *
 * @file esai.c
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Pawel Tryfon<pawel.tryfon@phoesys.com>
 * 
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <hal/MVF50GS10MK50.h>
#include <vm/if.h>
#include <main/if.h>

#include "dma.h"

#define DMA_NUM 2
static struct {
	DMA_Type* dmaReg[DMA_NUM];
	DMAMUX_Type* dmamuxReg[2*DMA_NUM];
	u16 channels[2*DMA_NUM];
	dma_interrupt_t dmaIrqTable[DMA_NUM][32];
} dma_g = {{NULL}};


int dma_irq(unsigned int n,cpu_context_t* ctx,void* data);
int dma_errIrq(unsigned int n,cpu_context_t* ctx,void* data);

void _dma_init(void)
{
	int i;
	DMA_Type* dma_bases[] = DMA_BASES;
	DMAMUX_Type* dmamux_bases[] = DMAMUX_BASES;
	for(i = 0;i < sizeof(dma_bases)/sizeof(DMA_Type*);i++) {
		if(vm_iomap((addr_t) dma_bases[i],sizeof(DMA_Type),PGHD_DEV_RW,(void**)&dma_g.dmaReg[i]) < 0) {
			main_printf(ATTR_ERROR,"dev: [_dma_init] Failed to initialize dma\n");
			return;
		}
		dma_g.channels[2*i] = 0xFF;
		dma_g.channels[2*i+1] = 0xFF;
	}
	for(i = 0;i < sizeof(dmamux_bases)/sizeof(DMAMUX_Type*);i++) {
		if(vm_iomap((addr_t) dmamux_bases[i],sizeof(DMAMUX_Type),PGHD_DEV_RW,(void**)&dma_g.dmamuxReg[i]) < 0) {
			main_printf(ATTR_ERROR,"dev: [_dma_init] Failed to initialize dma\n");
			return;
		}
	}

	dma_g.dmaReg[0]->CR = DMA_CR_GRP1PRI(1) |      // Channel Group 1 Priority (1 == higher (must be != group 0)) 
		DMA_CR_GRP0PRI(0);      // Channel Group 0 Priority (0 == lower (must be != group 1))
	dma_g.dmaReg[1]->CR = DMA_CR_GRP1PRI(1) |      // Channel Group 1 Priority (1 == higher (must be != group 0)) 
		DMA_CR_GRP0PRI(0);      // Channel Group 0 Priority (0 == lower (must be != group 1))
	hal_interruptsSetHandler(40,dma_irq,(void*)0); // DMA0 complete
	hal_interruptsSetHandler(41,dma_errIrq,(void*)0); // DMA0 error
	hal_interruptsSetHandler(42,dma_irq,(void*)1); // DMA1 complete
	hal_interruptsSetHandler(43,dma_errIrq,(void*)1); // DMA1 error
	dma_g.dmaReg[0]->SEEI = DMA_SEEI_SAEE_MASK;
}

int dma_irq(unsigned int n,cpu_context_t* ctx,void* data)
{
	u32 status;
	dma_handle_t h = (dma_handle_t)(unsigned int) data;
	status = dma_g.dmaReg[h]->INT;
	dma_g.dmaReg[h]->CINT = DMA_CINT_CAIR_MASK;
	dma_channel_t ch;
	while(status != 0) {
		ch = hal_cpuGetFirstBit(status);
		status &= ~((u32)1 << ch);
		if(dma_g.dmaIrqTable[h][ch] != NULL)
			dma_g.dmaIrqTable[h][ch](h,ch);
	}
	return 0;
}


int dma_errIrq(unsigned int n,cpu_context_t* ctx,void* data)
{
	dma_handle_t h = (dma_handle_t)(unsigned int) data;
	//u32 status = dma_g.dmaReg[h]->ES;
	assert(0);
	//status = dma_g.dmaReg[h]->ERR;
	dma_g.dmaReg[h]->CERR = DMA_CERR_CAEI_MASK;
	//main_printf(ATTR_ERROR,"dev: DMA%d error\n",h);
	//main_printf(ATTR_ERROR,"    DMA%d.ES = %x\n",h,dma_g.dmaReg[h]->ES);
	//main_printf(ATTR_ERROR,"    DMA%d.ERR = %x\n",h,status);
	return 0;
}

int dma_allocChannel(dma_handle_t h, u8 mux,dma_channel_t* ch)
{
	if(h >= DMA_NUM || mux > 1)
		return -EINVAL;
	if(dma_g.channels[2*h+mux] != 0) {
		*ch = hal_cpuGetFirstBit(dma_g.channels[2*h+mux]) + 16 * mux;
		dma_g.channels[2*h+mux] &= ~(1 << (*ch & 0xF));
		dma_g.dmaReg[h]->TCD[*ch].CSR = 0;
		return 0;
	} else
		return -EBUSY;
}

void* dma_getPtr(dma_handle_t h)
{
    if (h >= DMA_NUM)
        return NULL;
    return dma_g.dmaReg[h];
}

void* dma_getTcdPtr(dma_handle_t h, dma_channel_t ch)
{
    if (h >= DMA_NUM || ch >= 32)
        return NULL;
    return &dma_g.dmaReg[h]->TCD[ch];
}


int dma_enableSource(dma_handle_t h,dma_channel_t ch, u8 source)
{
	u8 mux;
	if(ch >= 32)
		return -EINVAL;
	dma_g.dmaReg[h]->SERQ = ch;
	mux = ch >> 4;
	dma_g.dmamuxReg[2*h+mux]->CHCFG[ch & 0xF] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(source);
	return 0;
}


int dma_getError(dma_handle_t h)
{
	return dma_g.dmaReg[h]->ES;
}


void dma_setTcdSaddr(dma_handle_t h,dma_channel_t ch,u32 addr)
{
	dma_g.dmaReg[h]->TCD[ch].SADDR = addr;
}


void dma_setTcdDaddr(dma_handle_t h,dma_channel_t ch,u32 addr)
{
	dma_g.dmaReg[h]->TCD[ch].DADDR = addr;
}


void dma_setTcdSoff(dma_handle_t h,dma_channel_t ch,u32 off)
{
	dma_g.dmaReg[h]->TCD[ch].SOFF = off;
}


void dma_setTcdDoff(dma_handle_t h,dma_channel_t ch,u32 off)
{
	dma_g.dmaReg[h]->TCD[ch].DOFF = off;
}


void dma_setTcdAccessSize(dma_handle_t h,dma_channel_t ch, u8 size)
{
	switch(size) {
		case 1: 
			size = 0;
			break;
		case 2:
			size = 1;
			break;
		case 4:
			size = 2;
			break;
		case 8:
			size = 3;
			break;
		default:
			size = 0;
			break;
	}
    dma_g.dmaReg[h]->TCD[ch].ATTR = DMA_ATTR_SMOD(0) |     // no loop
                       DMA_ATTR_SSIZE(size) | // 16-bit
                       DMA_ATTR_DMOD(0) |     // no loop
                       DMA_ATTR_DSIZE(size);  // 16-bit
}


void dma_setTcdSmodulo(dma_handle_t h,dma_channel_t ch, u8 smod)
{
    dma_g.dmaReg[h]->TCD[ch].ATTR = (dma_g.dmaReg[h]->TCD[ch].ATTR & ~DMA_ATTR_SMOD_MASK) | DMA_ATTR_SMOD(smod);
}


void dma_setTcdDmodulo(dma_handle_t h,dma_channel_t ch, u8 dmod)
{
    dma_g.dmaReg[h]->TCD[ch].ATTR = (dma_g.dmaReg[h]->TCD[ch].ATTR & ~DMA_ATTR_DMOD_MASK) | DMA_ATTR_DMOD(dmod);
}


void dma_setTcdMinorCount(dma_handle_t h,dma_channel_t ch, u32 size)
{
	dma_g.dmaReg[h]->TCD[ch].NBYTES_MLNO = size;
}


void dma_setTcdSlast(dma_handle_t h,dma_channel_t ch,u32 slast)
{
	dma_g.dmaReg[h]->TCD[ch].SLAST = slast;
}


void dma_setTcdDlast(dma_handle_t h,dma_channel_t ch,u32 dlast)
{
	dma_g.dmaReg[h]->TCD[ch].DLAST_SGA = dlast;
}


void dma_setTcdBiter(dma_handle_t h,dma_channel_t ch,u16 biter)
{
	/* Biter and citer should be equal at the beginning */
	dma_g.dmaReg[h]->TCD[ch].CITER_ELINKNO = biter & DMA_CITER_ELINKNO_CITER_MASK;
	dma_g.dmaReg[h]->TCD[ch].BITER_ELINKNO = biter & DMA_BITER_ELINKNO_BITER_MASK;
}

int dma_getCiter(dma_handle_t h,dma_channel_t ch)
{
    return dma_g.dmaReg[h]->TCD[ch].CITER_ELINKNO;
}

void dma_enableInterrupts(dma_handle_t h,dma_channel_t ch,dma_interrupt_t irq,u8 end,u8 half)
{
	u16 mask = 0;
	dma_g.dmaIrqTable[h][ch] = irq;
	if(end)
		mask |= DMA_CSR_INTMAJOR_MASK;
	if(half)
		mask |= DMA_CSR_INTHALF_MASK;
	dma_g.dmaReg[h]->TCD[ch].CSR |= mask;
}

void dma_debugPrintRegs(dma_handle_t h)
{
    main_printf(ATTR_DEBUG, "  DMA%d.CR = 0x%x\n",h , dma_g.dmaReg[h]->CR);
    main_printf(ATTR_DEBUG, "  DMA%d.ES = 0x%x\n",h , dma_g.dmaReg[h]->ES);
    main_printf(ATTR_DEBUG, "  DMA%d.ERQ = 0x%x\n",h , dma_g.dmaReg[h]->ERQ);
    main_printf(ATTR_DEBUG, "  DMA%d.EEI = 0x%x\n",h , dma_g.dmaReg[h]->EEI);
    main_printf(ATTR_DEBUG, "  DMA%d.INT = 0x%x\n",h , dma_g.dmaReg[h]->INT);
    main_printf(ATTR_DEBUG, "  DMA%d.ERR = 0x%x\n",h , dma_g.dmaReg[h]->ERR);
    main_printf(ATTR_DEBUG, "  DMA%d.HRS = 0x%x\n",h , dma_g.dmaReg[h]->HRS);
}
 

void dma_debugPrintChannel(dma_handle_t h,u8 ch)
{
    main_printf(ATTR_DEBUG, "  DMA channel %d:\n", ch);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].SADDR = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].SADDR);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].SOFF = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].SOFF);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].ATTR = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].ATTR);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].NBYTES_MLNO = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].NBYTES_MLNO);
    uint32_t slast = dma_g.dmaReg[h]->TCD[ch].SLAST;
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].SLAST = 0x%x (%d)\n", ch, slast, slast);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].DADDR = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].DADDR);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].DOFF = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].DOFF);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].CITER_ELINKNO = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].CITER_ELINKNO);
    uint32_t dlast = dma_g.dmaReg[h]->TCD[ch].DLAST_SGA;
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].DLAST_SGA = 0x%x (%d)\n", ch, dlast, dlast);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].CSR = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].CSR);
    main_printf(ATTR_DEBUG, "    DMA_TCD[%d].BITER_ELINKNO = 0x%x\n", ch, dma_g.dmaReg[h]->TCD[ch].BITER_ELINKNO);
}








