/*
 * Phoenix-RTOS
 *
 * STM32N6 reset and clock controller driver
 *
 * Copyright 2017, 2018, 2020, 2025 Phoenix Systems
 * Author: Aleksander Kaminski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/interrupt.h>
#include <sys/threads.h>
#include <sys/pwman.h>
#include <sys/platform.h>

#include "stm32n6_regs.h"
#include "common.h"
#include "rcc.h"
#include "rtc.h"

#define MAX_CLOCK_CHOICES 8
static const uint8_t rcc_clksels[pctl_ipclks_count][MAX_CLOCK_CHOICES] = {
	[pctl_ipclk_adf1sel] = { clkid_hclk, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_timg },
	[pctl_ipclk_adc12sel] = { clkid_hclk, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_timg },
	[pctl_ipclk_dcmippsel] = { clkid_pclk5, clkid_per, clkid_ic17, clkid_hsi_div },
	[pctl_ipclk_eth1ptpsel] = { clkid_hclk, clkid_per, clkid_ic13, clkid_hse },
	[pctl_ipclk_eth1clksel] = { clkid_hclk, clkid_per, clkid_ic12, clkid_hse },
	[pctl_ipclk_eth1sel] = {},
	[pctl_ipclk_eth1refclksel] = {},
	[pctl_ipclk_fdcansel] = { clkid_pclk1, clkid_per, clkid_ic19, clkid_hse },
	[pctl_ipclk_fmcsel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_i2c1sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i2c2sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i2c3sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i2c4sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i3c1sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i3c2sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_ltdcsel] = { clkid_pclk5, clkid_per, clkid_ic16, clkid_hsi_div },
	[pctl_ipclk_mco1sel] = { clkid_hsi_div, clkid_lse, clkid_msi, clkid_lsi, clkid_hse, clkid_ic5, clkid_ic10, clkid_sysa },
	[pctl_ipclk_mco2sel] = { clkid_hsi_div, clkid_lse, clkid_msi, clkid_lsi, clkid_hse, clkid_ic15, clkid_ic20, clkid_sysb },
	[pctl_ipclk_mdf1sel] = { clkid_hclk, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_timg },
	[pctl_ipclk_xspi1sel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_xspi2sel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_xspi3sel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_otgphy1sel] = { clkid_hse, clkid_per, clkid_ic15, clkid_hse_div2_osc },
	[pctl_ipclk_otgphy1ckrefsel] = {},
	[pctl_ipclk_otgphy2sel] = { clkid_hse, clkid_per, clkid_ic15, clkid_hse_div2_osc },
	[pctl_ipclk_otgphy2ckrefsel] = {},
	[pctl_ipclk_persel] = { clkid_hsi, clkid_msi, clkid_hse, clkid_ic19, clkid_ic5, clkid_ic10, clkid_ic15, clkid_ic20 },
	[pctl_ipclk_pssisel] = { clkid_hclk, clkid_per, clkid_ic20, clkid_hsi_div },
	[pctl_ipclk_rtcsel] = { 0, clkid_lse, clkid_lsi, clkid_hse_rtc },
	[pctl_ipclk_sai1sel] = { clkid_pclk2, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_spdif_symb },
	[pctl_ipclk_sai2sel] = { clkid_pclk2, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_spdif_symb },
	[pctl_ipclk_sdmmc1sel] = { clkid_hclk, clkid_per, clkid_ic4, clkid_ic5 },
	[pctl_ipclk_sdmmc2sel] = { clkid_hclk, clkid_per, clkid_ic4, clkid_ic5 },
	[pctl_ipclk_spdifrx1sel] = { clkid_pclk1, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi1sel] = { clkid_pclk2, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi2sel] = { clkid_pclk1, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi3sel] = { clkid_pclk1, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi4sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_msi, clkid_hsi_div, clkid_hse },
	[pctl_ipclk_spi5sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_msi, clkid_hsi_div, clkid_hse },
	[pctl_ipclk_spi6sel] = { clkid_pclk4, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_lptim1sel] = { clkid_pclk1, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim2sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim3sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim4sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim5sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_usart1sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart2sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart3sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart4sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart5sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart6sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart7sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart8sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart9sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart10sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_lpuart1sel] = { clkid_pclk4, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },

};

struct {
	volatile unsigned int *base;
	volatile unsigned int *pwr;

	handle_t lock;
} rcc_common;


int rcc_setClksel(enum ipclks ipclk, enum clock_ids clkID)
{
	if ((ipclk < 0) || (ipclk > pctl_ipclks_count) || (clkID == clkid_none)) {
		return -EINVAL;
	}

	size_t setting;
	for (setting = 0; setting < MAX_CLOCK_CHOICES; setting++) {
		enum clock_ids settingID = (enum clock_ids)rcc_clksels[ipclk][setting];
		if (settingID == clkID) {
			break;
		}
	}

	if (setting > MAX_CLOCK_CHOICES) {
		return -EINVAL;
	}

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_ipclk,
		.ipclk = {
			.ipclk = ipclk,
			.setting = setting,
		}
	};

	return platformctl(&pctl);
}


uint32_t clockdef_getRegHW(uint32_t offset)
{
	return *(rcc_common.base + offset);
}


int clockdef_getClockHW(
		clockdef_clkID_t id,
		uint32_t *base_out,
		uint64_t *nom,
		uint64_t *denom,
		clockdef_clkID_t *prev_out)
{
	(void)base_out;
	if (id >= clkid_pll1 && id <= clkid_pll4) {
		uint32_t v = *(rcc_common.base + rcc_pll1cfgr1 + 4 * (id - clkid_pll1));
		uint32_t src = (v >> 28) & 0x3;
		uint32_t bypass = (v >> 27) & 0x1;
		if (bypass == 0) {
			uint32_t div = (v >> 20) & 0x3f;
			uint32_t mul = (v >> 8) & 0xfff;
			v = *(rcc_common.base + rcc_pll1cfgr3 + 4 * (id - clkid_pll1));
			div *= (v >> 27) & 0x7;
			div *= (v >> 24) & 0x7;
			if ((v & 0xc) == 0xc) {
				/* Fractional divider active */
				uint32_t frac = *(rcc_common.base + rcc_pll1cfgr2 + 4 * (id - clkid_pll1));
				frac &= (1 << 24) - 1;
				*nom *= (((uint64_t)mul) << 24) + frac;
				*denom *= ((uint64_t)div) << 24;
			}
			else {
				*nom *= mul;
				*denom *= div;
			}
		}

		static const enum clock_ids src_to_id[4] = { clkid_hsi, clkid_msi, clkid_hse, clkid_i2s_in };
		*prev_out = src_to_id[src];
		return EOK;
	}

	if (id >= clkid_ic1 && id <= clkid_ic20) {
		uint32_t v = *(rcc_common.base + rcc_ic1cfgr + (id - clkid_ic1));
		uint32_t pll = (v >> 28) & 0x3;
		uint32_t divider = ((v >> 16) & 0xff) + 1;
		*denom *= divider;
		*prev_out = clkid_pll1 + pll;
		return EOK;
	}

	return -ENOENT;
}


void pwr_lockFromIRQ(uint32_t previous)
{
	/* Disable writing to backup domain if it was disabled previously */
	uint32_t tmp = *(rcc_common.pwr + pwr_dbpcr) & ~1u;
	*(rcc_common.pwr + pwr_dbpcr) = tmp | (previous & 1);
}


uint32_t pwr_unlockFromIRQ(void)
{
	uint32_t previous = *(rcc_common.pwr + pwr_dbpcr);
	/* Enable writing to backup domain */
	*(rcc_common.pwr + pwr_dbpcr) = previous | 1;
	return previous & 1;
}


void pwr_lock(void)
{
	mutexLock(rcc_common.lock);
	pwr_lockFromIRQ(0);
	mutexUnlock(rcc_common.lock);
}


void pwr_unlock(void)
{
	mutexLock(rcc_common.lock);
	(void)pwr_unlockFromIRQ();
	mutexUnlock(rcc_common.lock);
}


int rcc_init(void)
{
	rcc_common.base = RCC_BASE;
	rcc_common.pwr = PWR_BASE;

	mutexCreate(&rcc_common.lock);

	return 0;
}
