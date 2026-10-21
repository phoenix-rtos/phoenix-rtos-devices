/*
 * Phoenix-RTOS
 *
 * STM32 xPDMA driver register and setup definitions
 *
 * Copyright 2020-2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Daniel Sawka, Aleksander Kaminski, Jacek Maksymowicz, Mateusz Karcz
 *
 * %LICENSE%
 */


#ifndef _XPDMA_DEFS_H_
#define _XPDMA_DEFS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__CPU_STM32N6)
#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>

#define DMA_CTRL_GPDMA1 0
#define GPDMA_BASE      ((void *)0x50021000U)
#define DMA_CTRL_HPDMA1 1
#define HPDMA_BASE      ((void *)0x58020000U)
#define DMA_DEFAULT     DMA_CTRL_HPDMA1
#define GPDMA1_MAX_IRQ  gpdma1_ch15_irq
#elif defined(__CPU_STM32U3)
#include <phoenix/arch/armv8m/stm32/u3/stm32u3.h>

#define DMA_CTRL_GPDMA1 0
#define GPDMA_BASE      ((void *)0x50020000U)
#define DMA_DEFAULT     DMA_CTRL_GPDMA1
#define GPDMA1_MAX_IRQ  gpdma1_ch11_irq
#else
#error "Unknown platform"
#endif


enum dma_reqs {
#if defined(__CPU_STM32N6)
	dma_req_jpeg_rx = 0,
	dma_req_jpeg_tx = 1,
	dma_req_xspi1 = 2,
	dma_req_xspi2 = 3,
	dma_req_xspi3 = 4,
	dma_req_fmc_txrx = 5,
	dma_req_fmc_bch = 6,
	dma_req_adc1 = 7,
	dma_req_adc2 = 8,
	dma_req_cryp_in = 9,
	dma_req_cryp_out = 10,
	dma_req_saes_out = 11,
	dma_req_saes_in = 12,
	dma_req_hash_in = 13,
	dma_req_tim1_cc1 = 14,
	dma_req_tim1_cc2 = 15,
	dma_req_tim1_cc3 = 16,
	dma_req_tim1_cc4 = 17,
	dma_req_tim1_upd = 18,
	dma_req_tim1_trg = 19,
	dma_req_tim1_com = 20,
	dma_req_tim2_cc1 = 21,
	dma_req_tim2_cc2 = 22,
	dma_req_tim2_cc3 = 23,
	dma_req_tim2_cc4 = 24,
	dma_req_tim2_upd = 25,
	dma_req_tim2_trg = 26,
	dma_req_tim3_cc1 = 27,
	dma_req_tim3_cc2 = 28,
	dma_req_tim3_cc3 = 29,
	dma_req_tim3_cc4 = 30,
	dma_req_tim3_upd = 31,
	dma_req_tim3_trg = 32,
	dma_req_tim4_cc1 = 33,
	dma_req_tim4_cc2 = 34,
	dma_req_tim4_cc3 = 35,
	dma_req_tim4_cc4 = 36,
	dma_req_tim4_upd = 37,
	dma_req_tim4_trg = 38,
	dma_req_tim5_cc1 = 39,
	dma_req_tim5_cc2 = 40,
	dma_req_tim5_cc3 = 41,
	dma_req_tim5_cc4 = 42,
	dma_req_tim5_upd = 43,
	dma_req_tim5_trg = 44,
	dma_req_tim6_upd = 45,
	dma_req_tim7_upd = 46,
	dma_req_tim8_cc1 = 47,
	dma_req_tim8_cc2 = 48,
	dma_req_tim8_cc3 = 49,
	dma_req_tim8_cc4 = 50,
	dma_req_tim8_upd = 51,
	dma_req_tim8_trg = 52,
	dma_req_tim8_com = 53,
	dma_req_tim15_cc1 = 56,
	dma_req_tim15_cc2 = 57,
	dma_req_tim15_upd = 58,
	dma_req_tim15_trg = 59,
	dma_req_tim15_com = 60,
	dma_req_tim16_cc1 = 61,
	dma_req_tim16_upd = 62,
	dma_req_tim16_com = 63,
	dma_req_tim17_cc1 = 64,
	dma_req_tim17_upd = 65,
	dma_req_tim17_com = 66,
	dma_req_tim18_cc1 = 67,
	dma_req_tim18_upd = 68,
	dma_req_tim18_com = 69,
	dma_req_lptim1_ic1 = 70,
	dma_req_lptim1_ic2 = 71,
	dma_req_lptim1_ue = 72,
	dma_req_lptim2_ic1 = 73,
	dma_req_lptim2_ic2 = 74,
	dma_req_lptim2_ue = 75,
	dma_req_lptim3_ic1 = 76,
	dma_req_lptim3_ic2 = 77,
	dma_req_lptim3_ue = 78,
	dma_req_spi1_rx = 79,
	dma_req_spi1_tx = 80,
	dma_req_spi2_rx = 81,
	dma_req_spi2_tx = 82,
	dma_req_spi3_rx = 83,
	dma_req_spi3_tx = 84,
	dma_req_spi4_rx = 85,
	dma_req_spi4_tx = 86,
	dma_req_spi5_rx = 87,
	dma_req_spi5_tx = 88,
	dma_req_spi6_rx = 89,
	dma_req_spi6_tx = 90,
	dma_req_sai1_a = 91,
	dma_req_sai1_b = 92,
	dma_req_sai2_a = 93,
	dma_req_sai2_b = 94,
	dma_req_i2c1_rx = 95,
	dma_req_i2c1_tx = 96,
	dma_req_i2c2_rx = 97,
	dma_req_i2c2_tx = 98,
	dma_req_i2c3_rx = 99,
	dma_req_i2c3_tx = 100,
	dma_req_i2c4_rx = 101,
	dma_req_i2c4_tx = 102,
	dma_req_i3c1_rx = 103,
	dma_req_i3c1_tx = 104,
	dma_req_i3c2_rx = 105,
	dma_req_i3c2_tx = 106,
	dma_req_usart1_rx = 107,
	dma_req_usart1_tx = 108,
	dma_req_usart2_rx = 109,
	dma_req_usart2_tx = 110,
	dma_req_usart3_rx = 111,
	dma_req_usart3_tx = 112,
	dma_req_uart4_rx = 113,
	dma_req_uart4_tx = 114,
	dma_req_uart5_rx = 115,
	dma_req_uart5_tx = 116,
	dma_req_usart6_rx = 117,
	dma_req_usart6_tx = 118,
	dma_req_uart7_rx = 119,
	dma_req_uart7_tx = 120,
	dma_req_uart8_rx = 121,
	dma_req_uart8_tx = 122,
	dma_req_uart9_rx = 123,
	dma_req_uart9_tx = 124,
	dma_req_usart10_rx = 125,
	dma_req_usart10_tx = 126,
	dma_req_lpuart1_rx = 127,
	dma_req_lpuart1_tx = 128,
	dma_req_spdifrx_cs = 129,
	dma_req_spdifrx_dt = 130,
	dma_req_adf1_flt0 = 131,
	dma_req_mdf1_flt0 = 132,
	dma_req_mdf1_flt1 = 133,
	dma_req_mdf1_flt2 = 134,
	dma_req_mdf1_flt3 = 135,
	dma_req_mdf1_flt4 = 136,
	dma_req_mdf1_flt5 = 137,
	dma_req_ucpd1_tx = 138,
	dma_req_ucpd1_rx = 139,
	dma_req_dcmi = 140,
	dma_req_i3c1_tc = 141,
	dma_req_i3c1_rs = 142,
	dma_req_i3c2_tc = 143,
	dma_req_i3c2_rs = 144,
#elif defined(__CPU_STM32U3)
	dma_req_adc1 = 0,
	dma_req_adc2,
	dma_req_dac1,
	dma_req_dac2,
	dma_req_tim6_upd,
	dma_req_tim7_upd,
	dma_req_spi1_rx,
	dma_req_spi1_tx,
	dma_req_spi2_rx,
	dma_req_spi2_tx,
	dma_req_spi3_rx,
	dma_req_spi3_tx,
	dma_req_i2c1_rx,
	dma_req_i2c1_tx,
	dma_req_i2c1_evc,
	dma_req_i2c2_rx,
	dma_req_i2c2_tx,
	dma_req_i2c2_evc,
	dma_req_i2c3_rx,
	dma_req_i2c3_tx,
	dma_req_i2c3_evc,
	dma_req_i2c4_rx,
	dma_req_i2c4_tx,
	dma_req_i2c4_evc,
	dma_req_usart1_rx,
	dma_req_usart1_tx,
	dma_req_usart2_rx,
	dma_req_usart2_tx,
	dma_req_usart3_rx,
	dma_req_usart3_tx,
	dma_req_uart4_rx,
	dma_req_uart4_tx,
	dma_req_uart5_rx,
	dma_req_uart5_tx,
	dma_req_lpuart1_rx,
	dma_req_lpuart1_tx,
	dma_req_sai1_a,
	dma_req_sai1_b,
	dma_req_octospi1 = 40,
	dma_req_tim1_cc1 = 42,
	dma_req_tim1_cc2,
	dma_req_tim1_cc3,
	dma_req_tim1_cc4,
	dma_req_tim1_upd,
	dma_req_tim1_trig,
	dma_req_tim1_com,
	dma_req_i3c1_rx,
	dma_req_i3c1_tx,
	dma_req_i3c1_tc,
	dma_req_i3c1_rs,
	dma_req_tim2_cc1 = 56,
	dma_req_tim2_cc2,
	dma_req_tim2_cc3,
	dma_req_tim2_cc4,
	dma_req_tim2_upd,
	dma_req_tim3_cc1,
	dma_req_tim3_cc2,
	dma_req_tim3_cc3,
	dma_req_tim3_cc4,
	dma_req_tim3_upd,
	dma_req_tim3_trig,
	dma_req_tim4_cc1,
	dma_req_tim4_cc2,
	dma_req_tim4_cc3,
	dma_req_tim4_cc4,
	dma_req_tim4_upd,
	dma_req_i3c2_rx,
	dma_req_i3c2_tx,
	dma_req_i3c2_tc,
	dma_req_i3c2_rs,
	dma_req_spi4_rx,
	dma_req_spi4_tx,
	dma_req_tim15_cc1,
	dma_req_tim15_upd,
	dma_req_tim15_trig,
	dma_req_tim15_com,
	dma_req_tim16_cc1,
	dma_req_tim16_upd,
	dma_req_tim17_cc1,
	dma_req_tim17_upd,
	dma_req_aes_in = 87,
	dma_req_aes_out,
	dma_req_hash_in,
	dma_req_tim8_cc1 = 91,
	dma_req_tim8_cc2,
	dma_req_tim8_cc3,
	dma_req_tim8_cc4,
	dma_req_tim8_upd,
	dma_req_tim8_trig,
	dma_req_tim8_com,
	dma_req_adf1_flt0,
	dma_req_saes_in = 103,
	dma_req_saes_out,
	dma_req_lptim1_ic1,
	dma_req_lptim1_ic2,
	dma_req_lptim1_ue,
	dma_req_lptim2_ic1,
	dma_req_lptim2_ic2,
	dma_req_lptim2_ue,
	dma_req_lptim3_ic1,
	dma_req_lptim3_ic2,
	dma_req_lptim3_ue,
#endif
	dma_req_sw_trig = 254,
	dma_req_invalid = 255,
};


enum xpdma_regs {
	xpdma_seccfgr = 0x0,
	xpdma_privcfgr,
	xpdma_rcfglockr,
	xpdma_misr,
	xpdma_smisr,
};


#define CHAN_OFFS(chan) (0x14 + ((chan) * 0x20))
enum xpdma_cx_regs {
	xpdma_cxlbar = 0x0,
	hpdma_cxcidcfgr, /* HPDMA instances only */
	hpdma_cxsemcr,   /* HPDMA instances only */
	xpdma_cxfcr,
	xpdma_cxsr,
	xpdma_cxcr,
	xpdma_cxtr1 = 0x10,
	xpdma_cxtr2,
	xpdma_cxbr1,
	xpdma_cxsar,
	xpdma_cxdar,
	xpdma_cxtr3, /* 2D DMA only */
	xpdma_cxbr2, /* 2D DMA only */
	xpdma_cxllr = 0x1f,
};


/* CxTR1 register is slightly weird - the upper 16 bits mostly refer to the destination and lower 16 bits mostly refer to the source.
 * However, bits 29:26 and 13:10 refer to the transfer as a whole (byte exchange, padding and alignment). */
#define XPDMA_CXTR1_INDIVIDUAL_BITS 0xc3ffUL
#define XPDMA_CXTR1_COMMON_BITS     0x3c003c00UL
#define XPDMA_CXTR1_PAM_MASK        (3 << 11)           /* Mask for padding/alignment mode */
#define XPDMA_CXTR1_PAM_PACK        (2 << 11)           /* Padding/alignment set to FIFO queued packing */
#define XPDMA_CXTR1_SRC_EL_SIZE(x)  (((x) >> 0) & 0x3)  /* Log2 of element size for source */
#define XPDMA_CXTR1_DST_EL_SIZE(x)  (((x) >> 16) & 0x3) /* Log2 of element size for destination */

#define XPDMA_CXTR2_TCEM_BLOCK   (0 << 30) /* Generate TC event on block completion, HT on half block */
#define XPDMA_CXTR2_TCEM_RBLOCK  (1 << 30) /* Generate TC event on repeated block completion, HT on half repeated block */
#define XPDMA_CXTR2_TCEM_EACH_LL (2 << 30) /* Generate TC event on each linked-list item completion, HT on half of each list item */
#define XPDMA_CXTR2_TCEM_LAST_LL (3 << 30) /* Generate TC event on last linked-list item completion, HT on half of last list item */
#define XPDMA_CXTR2_TCEM_MASK    (3 << 30)

#define XPDMA_CXTR2_PF_NORMAL (0 << 12) /* XPDMA_CxBR1.BNDT[15:0] controls block size */
#define XPDMA_CXTR2_PF_CTRL   (1 << 12) /* Peripheral controls block size and can complete transfer early */
#define XPDMA_CXTR2_PER_BURST (0 << 11) /* Hardware requests one burst to be transferred */
#define XPDMA_CXTR2_PER_BLOCK (1 << 11) /* Hardware requests one block to be transferred */
#define XPDMA_CXTR2_PER2MEM   (0 << 10) /* Hardware request signals that data is available from source port */
#define XPDMA_CXTR2_MEM2PER   (1 << 10) /* Hardware request signals that data can be written to destination port */
#define XPDMA_CXTR2_MEM2MEM   (1 << 9)  /* Channel is software requested, ignore hardware requests */

#define XPDMA_TOF      (1 << 14) /* Trigger overflow */
#define XPDMA_SUSPF    (1 << 13) /* Channel suspended */
#define XPDMA_USEF     (1 << 12) /* User setting error */
#define XPDMA_ULEF     (1 << 11) /* Bus error when updating from linked-list */
#define XPDMA_DTEF     (1 << 10) /* Bus error when transferring data */
#define XPDMA_HTF      (1 << 9)  /* Half-transfer complete */
#define XPDMA_TCF      (1 << 8)  /* Transfer complete */
#define XPDMA_ALLFLAGS ((XPDMA_TOF << 1) - XPDMA_TCF)
#define XPDMA_ERRFLAGS (XPDMA_USEF | XPDMA_ULEF | XPDMA_DTEF)
#define XPDMA_IDLEF    (1 << 0) /* Channel idle */

#define XPDMA_CXLLR_UT1 (1 << 31)
#define XPDMA_CXLLR_UT2 (1 << 30)
#define XPDMA_CXLLR_UB1 (1 << 29)
#define XPDMA_CXLLR_USA (1 << 28)
#define XPDMA_CXLLR_UDA (1 << 27)
#define XPDMA_CXLLR_UT3 (1 << 26) /* 2D DMA only */
#define XPDMA_CXLLR_UB2 (1 << 25) /* 2D DMA only */
#define XPDMA_CXLLR_ULL (1 << 16)

#define XPDMA_LISTBUF_LEN 16 /* Maximum length of linked list in memory in words */

static const struct dma_setup {
	volatile uint32_t *base;
	int pctl;
	bool isHPDMA;
} dma_setup[LIBDMA_NUM_CONTROLLERS] = {
	[DMA_CTRL_GPDMA1] = {
		.base = GPDMA_BASE,
		.pctl = pctl_gpdma1,
		.isHPDMA = false,
	},
#if defined(DMA_CTRL_HPDMA1)
	[DMA_CTRL_HPDMA1] = {
		.base = HPDMA_BASE,
		.pctl = pctl_hpdma1,
		.isHPDMA = true,
	},
#endif
};


static inline volatile uint32_t *libdma_channelBase(unsigned int dma, unsigned int chn)
{
	return dma_setup[dma].base + CHAN_OFFS(chn);
}


#endif /* _XPDMA_DEFS_H_ */
