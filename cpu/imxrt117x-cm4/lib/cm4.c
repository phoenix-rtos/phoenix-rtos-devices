/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 basic core control
 *
 * Copyright 2017-2019, 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stddef.h>

#include "cm4.h"

// clang-format off
enum { stk_ctrl = 0, stk_load, stk_val, stk_calib };


enum { scb_cpuid = 0, scb_icsr, scb_vtor, scb_aircr, scb_scr, scb_ccr, scb_shp0, scb_shp1,
	scb_shp2, scb_shcsr, scb_cfsr, scb_hfsr, scb_dfsr, scb_mmfar, scb_bfar, scb_afsr, scb_pfr0,
	scb_pfr1, scb_dfr, scb_afr, scb_mmfr0, scb_mmfr1, scb_mmfr2, scb_mmf3, scb_isar0, scb_isar1,
	scb_isar2, scb_isar3, scb_isar4, /* reserved */ scb_clidr = 30, scb_ctr, scb_ccsidr, scb_csselr,
	scb_cpacr, /* 93 reserved */ scb_stir = 128, /* 15 reserved */ scb_mvfr0 = 144, scb_mvfr1,
	scb_mvfr2, /* reserved */ scb_iciallu = 148, /* reserved */ scb_icimvau = 150, scb_scimvac,
	scb_dcisw, scb_dccmvau, scb_dccmvac, scb_dccsw, scb_dccimvac, scb_dccisw, /* 6 reserved */
	scb_itcmcr = 164, scb_dtcmcr, scb_ahbpcr, scb_cacr, scb_ahbscr, /* reserved */ scb_abfsr = 170 };


enum { nvic_iser = 0, nvic_icer = 32, nvic_ispr = 64, nvic_icpr = 96, nvic_iabr = 128,
	nvic_ip = 256, nvic_stir = 896 };
// clang-format on


static struct {
	volatile unsigned int jiffies;

	volatile uint32_t *scb;
	volatile uint32_t *nvic;
	volatile uint32_t *iomuxc;
	volatile uint32_t *iomux_snvs;
	volatile uint32_t *iomux_lpsr;
	volatile uint32_t *stk;

	unsigned int cpuclk;
} cm4_common;


/* IOMUX */


static volatile uint32_t *cm4_IOmuxGetReg(int mux)
{
	if (mux < pctl_mux_gpio_emc_b1_00 || mux > pctl_mux_gpio_lpsr_15)
		return NULL;

	if (mux < pctl_mux_wakeup)
		return cm4_common.iomuxc + 4 + mux - pctl_mux_gpio_emc_b1_00;

	if (mux < pctl_mux_gpio_lpsr_00)
		return cm4_common.iomux_snvs + mux - pctl_mux_wakeup;

	return cm4_common.iomux_lpsr + mux - pctl_mux_gpio_lpsr_00;
}


int cm4_setIOmux(int mux, char sion, char mode)
{
	volatile uint32_t *reg;

	if ((reg = cm4_IOmuxGetReg(mux)) == NULL)
		return -1;

	(*reg) = (!!sion << 4) | (mode & 0xf);

	return 0;
}


int cm4_getIOmux(int mux, char *sion, char *mode)
{
	uint32_t t;
	volatile uint32_t *reg;

	if ((reg = cm4_IOmuxGetReg(mux)) == NULL)
		return -1;

	t = (*reg);
	*sion = !!(t & (1 << 4));
	*mode = t & 0xf;

	return 0;
}


static volatile uint32_t *cm4_IOpadGetReg(int pad)
{
	if (pad < pctl_pad_gpio_emc_b1_00 || pad > pctl_pad_gpio_lpsr_15)
		return NULL;

	if (pad < pctl_pad_test_mode)
		return cm4_common.iomuxc + pad + 149 - pctl_pad_gpio_emc_b1_00;

	if (pad < pctl_pad_gpio_lpsr_00)
		return cm4_common.iomux_snvs + pad + 14 - pctl_pad_test_mode;

	return cm4_common.iomux_lpsr + pad + 16 - pctl_pad_gpio_lpsr_00;
}


int cm4_setIOpad(int pad, char sre, char dse, char pue, char pus, char ode, char apc)
{
	uint32_t t;
	volatile uint32_t *reg;
	char pull;

	if ((reg = cm4_IOpadGetReg(pad)) == NULL)
		return -1;

	if (pad >= pctl_pad_gpio_emc_b1_00 && pad <= pctl_pad_gpio_disp_b2_15) {
		/* Fields have slightly diffrent meaning... */
		if (!pue)
			pull = 3;
		else if (pus)
			pull = 1;
		else
			pull = 2;

		t = *reg & ~0x1e;
		t |= (!!dse << 1) | (pull << 2) | (!!ode << 4);
	}
	else {
		t = *reg & ~0x1f;
		t |= (!!sre) | (!!dse << 1) | (!!pue << 2) | (!!pus << 3);

		if (pad >= pctl_pad_test_mode && pad <= pctl_pad_gpio_snvs_09) {
			t &= ~(1 << 6);
			t |= !!ode << 6;
		}
		else {
			t &= ~(1 << 5);
			t |= !!ode << 5;
		}
	}

	/* APC field is not documented. Leave it alone for now. */
	//t &= ~(0xf << 28);
	//t |= (apc & 0xf) << 28;

	(*reg) = t;

	return 0;
}


int cm4_getIOpad(int pad, char *sre, char *dse, char *pue, char *pus, char *ode, char *apc)
{
	uint32_t t;
	char pull;
	volatile uint32_t *reg;

	if ((reg = cm4_IOpadGetReg(pad)) == NULL)
		return -1;

	t = (*reg);

	if (pad >= pctl_pad_gpio_emc_b1_00 && pad <= pctl_pad_gpio_disp_b2_15) {
		pull = (t >> 2) & 3;

		if (pull == 3) {
			*pue = 0;
		}
		else {
			*pue = 1;
			if (pull & 1)
				*pus = 1;
			else
				*pus = 0;
		}

		*ode = (t >> 4) & 1;
		/* sre field does not apply, leave it alone */
	}
	else {
		*sre = t & 1;
		*pue = (t >> 2) & 1;
		*pus = (t >> 3) & 1;

		if (pad >= pctl_pad_test_mode && pad <= pctl_pad_gpio_snvs_09)
			*ode = (t >> 6) & 1;
		else
			*ode = (t >> 5) & 1;
	}

	*dse = (t >> 1) & 1;
	*apc = (t >> 28) & 0xf;

	return 0;
}


static volatile uint32_t *cm4_IOiselGetReg(int isel, uint32_t *mask)
{
	if (isel < pctl_isel_flexcan1_rx || isel > pctl_isel_sai4_txsync)
		return NULL;

	switch (isel) {
		case pctl_isel_flexcan1_rx:
		case pctl_isel_ccm_enet_qos_ref_clk:
		case pctl_isel_enet_ipg_clk_rmii:
		case pctl_isel_enet_1g_ipg_clk_rmii:
		case pctl_isel_enet_1g_mac0_mdio:
		case pctl_isel_enet_1g_mac0_rxclk:
		case pctl_isel_enet_1g_mac0_rxdata_0:
		case pctl_isel_enet_1g_mac0_rxdata_1:
		case pctl_isel_enet_1g_mac0_rxdata_2:
		case pctl_isel_enet_1g_mac0_rxdata_3:
		case pctl_isel_enet_1g_mac0_rxen:
		case enet_qos_phy_rxer:
		case pctl_isel_flexspi1_dqs_fa:
		case pctl_isel_lpuart1_rxd:
		case pctl_isel_lpuart1_txd:
		case pctl_isel_qtimer1_tmr0:
		case pctl_isel_qtimer1_tmr1:
		case pctl_isel_qtimer2_tmr0:
		case pctl_isel_qtimer2_tmr1:
		case pctl_isel_qtimer3_tmr0:
		case pctl_isel_qtimer3_tmr1:
		case pctl_isel_qtimer4_tmr0:
		case pctl_isel_qtimer4_tmr1:
		case pctl_isel_sdio_slv_clk_sd:
		case pctl_isel_sdio_slv_cmd_di:
		case pctl_isel_sdio_slv_dat0_do:
		case pctl_isel_slv_dat1_irq:
		case pctl_isel_sdio_slv_dat2_rw:
		case pctl_isel_sdio_slv_dat3_cs:
		case pctl_isel_spdif_in1:
		case pctl_isel_can3_canrx:
		case pctl_isel_lpuart12_rxd:
		case pctl_isel_lpuart12_txd:
			(*mask) = 0x3;
			break;

		default:
			(*mask) = 0x1;
			break;
	}

	if (isel >= pctl_isel_can3_canrx)
		return cm4_common.iomux_lpsr + 32 + isel - pctl_isel_can3_canrx;

	return cm4_common.iomuxc + 294 + isel - pctl_isel_flexcan1_rx;
}


int cm4_setIOisel(int isel, char daisy)
{
	volatile uint32_t *reg;
	uint32_t mask;

	if ((reg = cm4_IOiselGetReg(isel, &mask)) == NULL)
		return -1;

	(*reg) = daisy & mask;

	return 0;
}


int cm4_getIOisel(int isel, char *daisy)
{
	volatile uint32_t *reg;
	uint32_t mask;

	if ((reg = cm4_IOiselGetReg(isel, &mask)) == NULL)
		return -1;

	*daisy = (*reg) & mask;

	return 0;
}


/* SCB */


void cm4_scbSetPriorityGrouping(uint32_t group)
{
	uint32_t t;

	/* Get register value and clear bits to set */
	t = *(cm4_common.scb + scb_aircr) & ~0xffff0700;

	/* Store new value */
	*(cm4_common.scb + scb_aircr) = t | 0x5fa0000 | ((group & 7) << 8);
}


uint32_t cm4_scbGetPriorityGrouping(void)
{
	return (*(cm4_common.scb + scb_aircr) & 0x700) >> 8;
}


void cm4_scbSetPriority(int8_t excpn, uint32_t priority)
{
	volatile uint8_t *ptr;

	ptr = &((uint8_t *)(cm4_common.scb + scb_shp0))[excpn - 4];

	*ptr = (priority << 4) & 0x0ff;
}


uint32_t cm4_scbGetPriority(int8_t excpn)
{
	volatile uint8_t *ptr;

	ptr = &((uint8_t *)(cm4_common.scb + scb_shp0))[excpn - 4];

	return *ptr >> 4;
}


/* NVIC (Nested Vectored Interrupt Controller */
/* NVIC is private for each core, we can use it freely */


void cm4_nvicSetIRQ(int8_t irqn, uint8_t state)
{
	volatile uint32_t *ptr = cm4_common.nvic + ((uint8_t)irqn >> 5) + (state ? nvic_iser: nvic_icer);
	*ptr |= 1 << (irqn & 0x1f);
}


uint32_t cm4_nvicGetPendingIRQ(int8_t irqn)
{
	volatile uint32_t *ptr = cm4_common.nvic + ((uint8_t)irqn >> 5) + nvic_ispr;
	return !!(*ptr & (1 << (irqn & 0x1f)));
}


void cm4_nvicSetPendingIRQ(int8_t irqn, uint8_t state)
{
	volatile uint32_t *ptr = cm4_common.nvic + ((uint8_t)irqn >> 5) + (state ? nvic_ispr: nvic_icpr);
	*ptr |= 1 << (irqn & 0x1f);
}


uint32_t cm4_nvicGetActive(int8_t irqn)
{
	volatile uint32_t *ptr = cm4_common.nvic + ((uint8_t)irqn >> 5) + nvic_iabr;
	return !!(*ptr & (1 << (irqn & 0x1f)));
}


void cm4_nvicSetPriority(int8_t irqn, uint32_t priority)
{
	volatile uint8_t *ptr;

	ptr = (uint8_t *)(cm4_common.nvic + irqn + nvic_ip);

	*ptr = (priority << 4) & 0x0ff;
}


uint8_t cm4_nvicGetPriority(int8_t irqn)
{
	volatile uint8_t *ptr;

	ptr = (uint8_t *)(cm4_common.nvic + irqn + nvic_ip);

	return *ptr >> 4;
}


/* SysTick */


int cm4_systickInit(uint32_t interval)
{
	uint64_t load = ((uint64_t) interval * cm4_common.cpuclk) / 1000000;
	if (load > 0x00ffffff)
		return -1;

	*(cm4_common.stk + stk_load) = (uint32_t)load;
	*(cm4_common.stk + stk_ctrl) = 0x7;

	return 0;
}


void cm4_systickSet(uint8_t state)
{
	state = !state;

	*(cm4_common.stk + stk_ctrl) &= ~state;
	*(cm4_common.stk + stk_ctrl) |= !state;
}


uint32_t cm4_systickGet(void)
{
	uint32_t cb;

	cb = ((*(cm4_common.stk + stk_load) - *(cm4_common.stk + stk_val)) * 1000) / *(cm4_common.stk + stk_load);

	/* Add 1000 us if there's systick pending */
	if (*(cm4_common.scb + scb_icsr) & (1 << 26))
		cb += 1000;

	return cb;
}


/* Default handlers */


void __attribute__((weak)) _exceptionHandler(int n)
{
	for (;;)
		;
}


void __attribute__((weak)) _systickHandler(void)
{
	++cm4_common.jiffies;
}


void __attribute__((weak)) _syscallHandler(int n)
{
	(void)n;
}


void __attribute__((weak)) _pendsvHandler(void)
{

}


unsigned int cm4_getJiffies(void)
{
	return cm4_common.jiffies;
}


void cm4_init(void)
{
	cm4_common.scb = (void *)0xe000ed00;
	cm4_common.nvic = (void *)0xe000e100;
	cm4_common.iomux_snvs = (void *)0x40c94000;
	cm4_common.iomux_lpsr = (void *)0x40c08000;
	cm4_common.iomuxc = (void *)0x400e8000;
	cm4_common.stk = (void *)0xe000e010;

	cm4_common.cpuclk = 240 * 1000 * 1000;

	/* Enable UsageFault, BusFault and MemManage exceptions */
	*(cm4_common.scb + scb_shcsr) |= (1 << 16) | (1 << 17) | (1 << 18);

	cm4_scbSetPriorityGrouping(3);
	cm4_systickInit(1000);
}
