#include <sys/mman.h>
#include <sys/platform.h>

#include <phoenix/arch/imxrt.h>

#include "../ad7779.h"

#define SAI_RCR3_RCE_BIT  (1 << 16)
#define SAI_RCSR_RE_BIT   (1 << 31)
#define SAI_RCSR_SR_BIT   (1 << 24)
#define SAI_RCSR_FRDE_BIT (1 << 0)

typedef volatile struct {
	uint32_t VERID;         //0x00
	uint32_t PARAM;         //0x04
	uint32_t TCSR;          //0x08
	uint32_t TCR1;          //0x0C
	uint32_t TCR2;          //0x10
	uint32_t TCR3;          //0x14
	uint32_t TCR4;          //0x18
	uint32_t TCR5;          //0x1C
	uint32_t TDR0;          //0x20
	uint32_t TDR1;          //0x24
	uint32_t TDR2;          //0x28
	uint32_t TDR3;          //0x2C
	uint32_t reserved0[4];  //0x30-0x3C
	uint32_t TFR0;          //0x40
	uint32_t TFR1;          //0x44
	uint32_t TFR2;          //0x48
	uint32_t TFR3;          //0x4C
	uint32_t reserved1[4];  //0x50-0x5C
	uint32_t TMR;           //0x60
	uint32_t reserved2[9];  //0x64-0x84
	uint32_t RCSR;          //0x88
	uint32_t RCR1;          //0x8C
	uint32_t RCR2;          //0x90
	uint32_t RCR3;          //0x94
	uint32_t RCR4;          //0x98
	uint32_t RCR5;          //0x9C
	uint32_t RDR0;          //0xA0
	uint32_t RDR1;          //0xA4
	uint32_t RDR2;          //0xA8
	uint32_t RDR3;          //0xAC
	uint32_t reserved3[4];  //0xB0-0xBC
	uint32_t RFR0;          //0xC0
	uint32_t RFR1;          //0xC4
	uint32_t RFR2;          //0xC8
	uint32_t RFR3;          //0xCC
	uint32_t reserved4[4];  //0xD0-0xDC
	uint32_t RMR;           //0xE0
} sai_t;

static struct {
	uint32_t watermark;
	sai_t *reg;
	addr_t paddr;
} sai_common = {
	.watermark = 8,
	.reg = NULL,
	.paddr = 0x40384000, /* SAI1 */
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
	pctl.devclock.dev = pctl_clk_sai1;
	pctl.devclock.state = 3;
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_14;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_SYNC) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_sync;
	pctl.ioisel.daisy = 2; /* Select GPIO_B0_14 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_15;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_BCLK) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_bclk;
	pctl.ioisel.daisy = 2; /* Select GPIO_B0_15 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b1_00;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA0) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data0;
	pctl.ioisel.daisy = 2; /* Select GPIO_B1_00 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_10;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA1) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data1;
	pctl.ioisel.daisy = 1; /* Select GPIO_B0_10 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_11;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA2) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data2;
	pctl.ioisel.daisy = 1; /* Select GPIO_B0_11 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_12;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA3) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data3;
	pctl.ioisel.daisy = 1; /* Select GPIO_B0_12 pad */
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
	sai_common.reg->RCSR &= ~(SAI_RCSR_SR_BIT);
}


addr_t sai_fifo_rx_ptr(void)
{
	return (addr_t)(&(((sai_t *)sai_common.paddr)->RDR0));
}

uint32_t sai_fifo_watermark(void)
{
	return sai_common.watermark;
}
