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

#ifndef _VYBRID_DMA_H_
#define _VYBRID_DMA_H_

typedef u8 dma_handle_t;
typedef u8 dma_channel_t;
typedef void (*dma_interrupt_t)(dma_handle_t h,dma_channel_t ch);

extern void* dma_getPtr(dma_handle_t h);
extern void* dma_getTcdPtr(dma_handle_t h, dma_channel_t ch);

extern void _dma_init(void);
extern int dma_allocChannel(dma_handle_t h, u8 mux,dma_channel_t* ch);
extern int dma_enableSource(dma_handle_t h,dma_channel_t ch, u8 source);
extern void dma_enableInterrupts(dma_handle_t h,dma_channel_t ch,dma_interrupt_t irq,u8 end,u8 half);
extern int dma_getError(dma_handle_t h);
extern void dma_setTcdSaddr(dma_handle_t h,dma_channel_t ch,u32 addr);
extern void dma_setTcdDaddr(dma_handle_t h,dma_channel_t ch,u32 addr);
extern void dma_setTcdSoff(dma_handle_t h,dma_channel_t ch,u32 off);
extern void dma_setTcdDoff(dma_handle_t h,dma_channel_t ch,u32 off);
extern void dma_setTcdAccessSize(dma_handle_t h,dma_channel_t ch, u8 size);
extern void dma_setTcdSmodulo(dma_handle_t h,dma_channel_t ch, u8 smod);
extern void dma_setTcdDmodulo(dma_handle_t h,dma_channel_t ch, u8 dmod);
extern void dma_setTcdMinorCount(dma_handle_t h,dma_channel_t ch, u32 size);
extern void dma_setTcdSlast(dma_handle_t h,dma_channel_t ch,u32 slast);
extern void dma_setTcdDlast(dma_handle_t h,dma_channel_t ch,u32 slast);
extern void dma_setTcdBiter(dma_handle_t h,dma_channel_t ch,u16 biter);
extern void dma_debugPrintRegs(dma_handle_t h);
extern void dma_debugPrintChannel(dma_handle_t h, u8 ch);
extern int dma_getCiter(dma_handle_t h,dma_channel_t ch);

#endif
