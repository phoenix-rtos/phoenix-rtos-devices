/*
 * Phoenix-RTOS
 *
 * STM32U3 reset and clock controller driver
 *
 * Copyright 2026 Apator Metrix
 * Author: Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>

#include "../stm32u3_regs.h"
#include "../common.h"
#include "../rcc.h"


static const uint8_t rcc_clksels[pctl_ipclks_count][RCC_MAX_CLOCK_CHOICES] = {
	[pctl_ipclk_usart1sel] = { clkid_pclk2, clkid_hsi16 },
	[pctl_ipclk_usart3sel] = { clkid_pclk1, clkid_hsi16 },
	[pctl_ipclk_uart4sel] = { clkid_pclk1, clkid_hsi16 },
	[pctl_ipclk_uart5sel] = { clkid_pclk1, clkid_hsi16 },
	[pctl_ipclk_i3c1sel] = { clkid_pclk1, clkid_msik },
	[pctl_ipclk_i2c1sel] = { clkid_pclk1, clkid_msik },
	[pctl_ipclk_i2c2sel] = { clkid_pclk1, clkid_msik },
	[pctl_ipclk_i3c2sel] = { clkid_pclk2, clkid_msik },
	[pctl_ipclk_spi2sel] = { clkid_pclk1, clkid_msik },
	[pctl_ipclk_lptim2sel] = { clkid_pclk1, clkid_lsi, clkid_hsi16, clkid_lse },
	[pctl_ipclk_spi1sel] = { clkid_pclk2, clkid_msik },
	[pctl_ipclk_systicksel] = { clkid_hclk_div, clkid_lsi, clkid_lse },
	[pctl_ipclk_fdcansel] = { clkid_sysclk, clkid_msik },
	[pctl_ipclk_iclksel] = { clkid_hsi48, clkid_msik, clkid_hse, clkid_sysclk },
	[pctl_ipclk_adf1sel] = { clkid_hclk, clkid_audioclk, clkid_msik, clkid_sai1 },
	[pctl_ipclk_spi3sel] = { clkid_pclk1, clkid_msik },
	[pctl_ipclk_sai1sel] = { clkid_msik, clkid_audioclk, clkid_hse },
	[pctl_ipclk_spi4sel] = { clkid_pclk1, clkid_msik },
	[pctl_ipclk_i2c4sel] = { clkid_pclk1, clkid_msik },
	[pctl_ipclk_rngsel] = { clkid_hsi48, clkid_msik },
	[pctl_ipclk_adcdacsel] = { clkid_hclk, clkid_hse, clkid_msik },
	[pctl_ipclk_dac1shsel] = { clkid_lse, clkid_lsi },
	[pctl_ipclk_octospisel] = { clkid_sysclk, clkid_msik },
	[pctl_ipclk_usart2sel] = { clkid_pclk1, clkid_hsi16 },
	[pctl_ipclk_lpuart1sel] = { clkid_pclk3, clkid_hsi16, clkid_lse, clkid_msik },
	[pctl_ipclk_i2c3sel] = { clkid_pclk3, clkid_msik },
	[pctl_ipclk_lptim34sel] = { clkid_msik, clkid_lsi, clkid_hsi16, clkid_lse },
	[pctl_ipclk_lptim1sel] = { clkid_msik, clkid_lsi, clkid_hsi16, clkid_lse },
};


#define HZ(n)  (n##U)
#define KHZ(n) (HZ(n) * 1000U)
#define MHZ(n) (KHZ(n) * 1000U)

#define CLOCK_LITERAL(x) \
	{ \
		.type = clockdef_type_literal, .literal = (x) \
	}


#define RCC_BIASED_FIELD(reg, sh, len, bias) \
	{ \
		.offset = (reg), .shift = (sh), .mask = (1 << (len)) - 1, .val_offset = (bias) \
	}
#define RCC_FIELD(reg, sh, len) RCC_BIASED_FIELD(reg, sh, len, 0)

#define RCC_CSR_LSIPREDIV  RCC_FIELD(rcc_csr, 2, 1)
#define RCC_ICSCR1_MSIKSEL RCC_FIELD(rcc_icscr1, 28, 1)
#define RCC_ICSCR1_MSISSEL RCC_FIELD(rcc_icscr1, 31, 1)
#define RCC_CFGR1_SW       RCC_FIELD(rcc_cfgr1, 0, 2)
#define RCC_CFGR2_HPRE     RCC_BIASED_FIELD(rcc_cfgr2, 0, 4, -7)
#define RCC_CFGR2_PPRE1    RCC_BIASED_FIELD(rcc_cfgr2, 4, 3, -3)
#define RCC_CFGR2_PPRE2    RCC_BIASED_FIELD(rcc_cfgr2, 8, 3, -3)
#define RCC_CFGR2_PPRE3    RCC_BIASED_FIELD(rcc_cfgr3, 4, 3, -3)
#define RCC_CCIPR1_ICLKSEL RCC_FIELD(rcc_ccipr1, 26, 2)
#define RCC_CCIPR1_USB1SEL RCC_FIELD(rcc_ccipr1, 28, 1)
#define RCC_CCIPR2_SAI1SEL RCC_FIELD(rcc_ccipr2, 5, 2)


static const clockdef_literal_t lsiSources[] = { KHZ(32), HZ(250) };
static const clockdef_literal_t msiSources[] = { MHZ(96), MHZ(24) };

static const clockdef_clkID_t sysclkSources[] = { clkid_msis, clkid_hsi16, clkid_hse };
static const clockdef_clkID_t iclkSources[] = { clkid_hsi48, clkid_msik, clkid_hse, clkid_sysclk };
static const clockdef_clkID_t sai1Sources[] = { clkid_msik, clkid_audioclk, clkid_hse };


static const clockdef_obj_t clockObjects[] = {
	[clkid_lsi] = {
		.base = {
			.type = clockdef_type_register_lookup,
			.reg_lookup = {
				.r = RCC_CSR_LSIPREDIV,
				.vals = lsiSources,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_lse] = {
		.base = CLOCK_LITERAL(HZ(32768)),
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_hse] = {
		.base = CLOCK_LITERAL(MHZ(48)),
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_hsi16] = {
		.base = CLOCK_LITERAL(MHZ(16)),
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_msis] = {
		.base = {
			.type = clockdef_type_register_lookup,
			.reg_lookup = {
				.r = RCC_ICSCR1_MSISSEL,
				.vals = msiSources,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_msik] = {
		.base = {
			.type = clockdef_type_register_lookup,
			.reg_lookup = {
				.r = RCC_ICSCR1_MSIKSEL,
				.vals = msiSources,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_hsi48] = {
		.base = CLOCK_LITERAL(MHZ(48)),
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_hse_rtc] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hse,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(32),
	},
	[clkid_sysclk] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = RCC_CFGR1_SW,
				.vals = sysclkSources,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_hclk] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_sysclk,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = RCC_CFGR2_HPRE,
		},
	},
	[clkid_hclk_div] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hclk,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(8),
	},
	[clkid_pclk1] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hclk,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = RCC_CFGR2_PPRE1,
		},
	},
	[clkid_pclk2] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hclk,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = RCC_CFGR2_PPRE2,
		},
	},
	[clkid_pclk3] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hclk,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = RCC_CFGR2_PPRE3,
		},
	},
	[clkid_iclk] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = RCC_CCIPR1_ICLKSEL,
				.vals = iclkSources,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_usb1] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_iclk,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = RCC_CCIPR1_USB1SEL,
		},
	},
	[clkid_audioclk] = {
		.base = CLOCK_LITERAL(KHZ(24576)),
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_sai1] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = RCC_CCIPR2_SAI1SEL,
				.vals = sai1Sources,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
};


static const size_t clockObjects_size = sizeof(clockObjects) / sizeof(clockObjects[0]);


const clockdef_obj_t *clockdef_getObject(clockdef_clkID_t id)
{
	if ((id > clkid_none) && (id < clockObjects_size)) {
		return clockObjects + id;
	}

	return NULL;
}


uint32_t clockdef_getRegHW(uint32_t offset)
{
	return *(rcc_common.base + offset);
}


const enum clock_ids *rcc_getClkselOptions(enum ipclks ipclk)
{
	return rcc_clksels[ipclk];
}


int clockdef_getClockHW(
		clockdef_clkID_t id,
		uint32_t *base_out,
		uint64_t *nom,
		uint64_t *denom,
		clockdef_clkID_t *prev_out)
{
	/* clockdef_getObject() handles all valid clock IDs on this platform.
	 * This function will only get called for invalid IDs. */
	(void)id;
	(void)base_out;
	(void)nom;
	(void)denom;
	(void)prev_out;
	return -ENOENT;
}


void pwr_lockFromIRQ(uint32_t previous)
{
	/* Disable writing to backup domain if it was disabled previously */
	uint32_t tmp = *(rcc_common.pwr + pwr_dbpr) & ~1u;
	*(rcc_common.pwr + pwr_dbpr) = tmp | (previous & 1);
}


uint32_t pwr_unlockFromIRQ(void)
{
	uint32_t previous = *(rcc_common.pwr + pwr_dbpr);
	/* Enable writing to backup domain */
	*(rcc_common.pwr + pwr_dbpr) = previous | 1;
	return previous & 1;
}
