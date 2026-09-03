/*
 * Phoenix-RTOS
 *
 * STM32L4 reset and clock controller driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Aleksander Kaminski, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "../common.h"
#include "../rcc.h"


/* clang-format off */
enum { pwr_cr1 = 0, pwr_cr2, pwr_cr3, pwr_cr4, pwr_sr1, pwr_sr2, pwr_scr, pwr_pucra, pwr_pdcra, pwr_pucrb,
	pwr_pdcrb, pwr_pucrc, pwr_pdcrc, pwr_pucrd, pwr_pdcrd, pwr_pucre, pwr_pdcre, pwr_pucrf, pwr_pdcrf,
	pwr_pucrg, pwr_pdcrg, pwr_pucrh, pwr_pdcrh, pwr_pucri, pwr_pdcri };
/* clang-format on */


void pwr_lockFromIRQ(uint32_t previous)
{
	/* Disable writing to backup domain if it was disabled previously */
	uint32_t tmp = *(rcc_common.pwr + pwr_cr1) & ~(1u << 8);
	*(rcc_common.pwr + pwr_cr1) = tmp | ((previous & 1) << 8);
}


uint32_t pwr_unlockFromIRQ(void)
{
	uint32_t previous = *(rcc_common.pwr + pwr_cr1);
	/* Enable writing to backup domain */
	*(rcc_common.pwr + pwr_cr1) = previous | (1u << 8);
	return (previous >> 8) & 1;
}
