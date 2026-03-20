/*
 * Phoenix-RTOS
 *
 * stm32l4-multi
 *
 * Clock selection for STM32N6
 *
 * Copyright 2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Jacek Maksymowicz, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RCC_STM32N6_H_
#define _RCC_STM32N6_H_


#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>


#define RCC_MAX_CLOCK_CHOICES 8


enum clock_ids {
	clkid_none = 0,
	clkid_lsi,
	clkid_lse,
	clkid_msi,
	clkid_hsi,
	clkid_hsi_div,
	clkid_hse,
	clkid_hse_div2_osc,
	clkid_hse_rtc,
	clkid_sysa,
	clkid_sysb,
	clkid_sysc,
	clkid_sysd,
	clkid_timg,
	clkid_hclk,
	clkid_pclk1,
	clkid_pclk2,
	clkid_pclk4,
	clkid_pclk5,
	clkid_per,
	clkid_i2s_in,
	clkid_jtag_tck,
	clkid_spdif_symb,
	clkid_pll1,
	clkid_pll2,
	clkid_pll3,
	clkid_pll4,
	clkid_ic1,
	clkid_ic2,
	clkid_ic3,
	clkid_ic4,
	clkid_ic5,
	clkid_ic6,
	clkid_ic7,
	clkid_ic8,
	clkid_ic9,
	clkid_ic10,
	clkid_ic11,
	clkid_ic12,
	clkid_ic13,
	clkid_ic14,
	clkid_ic15,
	clkid_ic16,
	clkid_ic17,
	clkid_ic18,
	clkid_ic19,
	clkid_ic20,
	clkids_count,
};

#endif /* _RCC_STM32N6_H_ */
