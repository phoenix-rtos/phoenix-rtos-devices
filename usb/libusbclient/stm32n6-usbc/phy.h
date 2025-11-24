/*
 * Phoenix-RTOS
 *
 * phy - depends on the platform
 *
 * Copyright 2025 Phoenix Systems
 * Author: Olaf Czerwinski, Radoslaw Szewczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _STM32_PHY_H_
#define _STM32_PHY_H_

#include <stdlib.h>
#include "registers.h"


#define PHY_IRQ otg1_irq


#define USBPHYC_CR_FSEL_24MHZ (0x2 << 4)
/* page 3865 per rm0486 - only rf_w1 bits valid for device mode */
#define GINTSTSWrMsk 0xD8F0FC0A


volatile uint32_t *phy_getOtgBase(void);


uint32_t phy_getIrq(void);


void phy_setClock(void);


int phy_init(void);


void phy_reset(void);

uint32_t phy_getHclkFreq(void);


#endif
