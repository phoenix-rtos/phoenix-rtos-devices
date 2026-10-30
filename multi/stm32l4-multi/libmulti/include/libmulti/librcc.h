/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4/N6/U3 reset and clock controller driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Aleksander Kaminski, Mateusz Karcz, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef LIBRCC_H_
#define LIBRCC_H_


#include <stdint.h>


#if defined(__CPU_STM32N6)
#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>

#define LIBRCC_MAX_CLOCK_CHOICES 8
#define LIBRCC_HAS_CLOCKDEF      1

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
#elif defined(__CPU_STM32U3)
#include <phoenix/arch/armv8m/stm32/u3/stm32u3.h>

#define LIBRCC_MAX_CLOCK_CHOICES 4
#define LIBRCC_HAS_CLOCKDEF      1

enum clock_ids {
	clkid_none = 0,
	clkid_lsi,
	clkid_lse,
	clkid_hse,
	clkid_hsi16,
	clkid_msis,
	clkid_msik,
	clkid_hsi48,
	clkid_hse_rtc,
	clkid_sysclk,
	clkid_hclk,
	clkid_hclk_div,
	clkid_pclk1,
	clkid_pclk2,
	clkid_pclk3,
	clkid_iclk,
	clkid_usb1,
	clkid_audioclk,
	clkid_sai1,
	clkids_count,
};
#endif


int librcc_init(void);


#ifdef LIBRCC_HAS_CLOCKDEF
typedef enum clock_ids librcc_clkid_t;


int librcc_setClksel(enum ipclks ipclk, librcc_clkid_t clkID);


int librcc_getClock(librcc_clkid_t id, uint64_t *output);
#endif


#endif
