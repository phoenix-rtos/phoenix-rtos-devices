#include <sys/mman.h>
#include <sys/platform.h>

#include <phoenix/arch/imx6ull.h>

#include "../ad7779.h"

#define SAI_RCR3_RCE_BIT  (1 << 16)
#define SAI_RCSR_RE_BIT   (1 << 31)
#define SAI_RCSR_SR_BIT   (1 << 24)
#define SAI_RCSR_FRDE_BIT (1 << 0)

typedef volatile struct {
	uint32_t TCSR;
	uint32_t TCR1;
	uint32_t TCR2;
	uint32_t TCR3;
	uint32_t TCR4;
	uint32_t TCR5;
	uint32_t reserved0[2];
	uint32_t TDR0;
	uint32_t reserved1[7];
	uint32_t TFR0;
	uint32_t reserved2[7];
	uint32_t TMR;
	uint32_t reserved3[7];
	uint32_t RCSR;
	uint32_t RCR1;
	uint32_t RCR2;
	uint32_t RCR3;
	uint32_t RCR4;
	uint32_t RCR5;
	uint32_t reserved4[2];
	uint32_t RDR0;
	uint32_t reserved5[7];
	uint32_t RFR0;
	uint32_t reserved6[7];
	uint32_t RMR;
	uint32_t reserved7[7];
	uint32_t MCR;
} sai_t;

static struct {
	uint32_t watermark;
	sai_t *reg;
	addr_t paddr;
} sai_common = {
	.watermark = 4,
	.reg = NULL,
	.paddr = 0x2030000, /* SAI1 */
};


void sai_free(void)
{
	if (!sai_common.reg)
		return;

	munmap((void *)sai_common.reg, _PAGE_SIZE);
	sai_common.reg = NULL;
}


int sai_init(void)
{
	platformctl_t pctl = { 0 };

	sai_common.reg = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, sai_common.paddr);
	if (sai_common.reg == MAP_FAILED) {
		sai_common.reg = NULL;
		return -1;
	}

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = pctl_clk_sai3;
	pctl.devclock.state = 3;
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_lcd_d10;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 1; /* ALT1 (SAI3_RX_SYNC) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_lcd_d11;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 1; /* ALT1 (SAI3_RX_BCLK) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_lcd_d14;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 1; /* ALT1 (SAI3_RX_DATA) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai3_rx;
	pctl.ioisel.daisy = 1; /* Select LCD_DATA14 pad */
	platformctl(&pctl);

	/* Initialize SAI */
	sai_common.reg->RCR1 = sai_common.watermark;
	sai_common.reg->RCR2 = 0x0; /* External bit clock (slave mode) */
	sai_common.reg->RCR3 |= SAI_RCR3_RCE_BIT;
	sai_common.reg->RCR4 = 0x00070018;         /* FRSZ=7, SYWD=0, MF=1, FSE=1, FSP=0, FSD=0 */
	sai_common.reg->RCR5 = 0x1f1f1f00;         /* WNW=31, WOW=31, FBT=31 */
	sai_common.reg->RMR = 0x0;                 /* No words masked */
	sai_common.reg->RCSR |= SAI_RCSR_FRDE_BIT; /* FIFO Request DMA Enable */

	return 0;
}

void sai_rx_enable(void)
{
	sai_common.reg->RCSR |= SAI_RCSR_RE_BIT;
}

void sai_rx_disable(void)
{
	sai_common.reg->RCSR &= ~(SAI_RCSR_RE_BIT);
}

addr_t sai_fifo_rx_ptr(void)
{
	return (addr_t)(&(((sai_t *)sai_common.paddr)->RDR0));
}

uint32_t sai_fifo_watermark(void)
{
	return sai_common.watermark;
}
