/*
 * Phoenix-RTOS
 *
 * stm32l4-multi
 *
 * Clock selection for STM32U3
 *
 * Copyright 2026 Apator Metrix
 * Author: Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RCC_STM32U3_H_
#define _RCC_STM32U3_H_


#include <phoenix/arch/armv8m/stm32/u3/stm32u3.h>


#define RCC_MAX_CLOCK_CHOICES 4


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
