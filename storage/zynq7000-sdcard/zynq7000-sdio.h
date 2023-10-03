/*
 * Phoenix-RTOS
 *
 * Zynq-7000 SDIO peripheral initialization header
 *
 * Copyright 2023 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _ZYNQ7000_SDIO_H_
#define _ZYNQ7000_SDIO_H_

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

/* Number of SD Host Controllers available. */
#define PLATFORM_SDIO_N_HOSTS 1

/* Structure with platform-specific information that the SD Host Controller may need */
typedef struct {
	/* Frequency of reference clock that was selected (Hz) */
	uint32_t refclkFrequency;
	/* Physical address of the SD Host Controller register bank */
	addr_t regBankPhys;
	/* Number of the interrupt that will be used by this controller */
	int interruptNum;
	/* Does this slot have a physical Card Detect switch connected */
	bool isCDPinSupported;
	/* Does this slot have a physical Write Protect switch connected */
	bool isWPPinSupported;
} sdio_platformInfo_t;


static inline void sdio_dataBarrier(void)
{
	__asm__ volatile("dmb");
}


int sdio_platformConfigure(unsigned int slot, sdio_platformInfo_t *infoOut);

#endif /* _ZYNQ7000_SDIO_H_ */
