#ifndef _STM32_PHY_H_
#define _STM32_PHY_H_

#include <stdlib.h>
#include "registers.h"
#include "client.h"

#define USBPHYC_CR_FSEL_24MHZ (0x2U << 4)

volatile uint32_t *phy_getOtgBase(void);

uint32_t phy_getIrq(void);

int phy_setClock(void);

int phy_mapRegs(void);
void phy_config(usb_dc_t *dc);
int phy_usbss_init(void);
int phy_clk_reset(void);
int phy_clear_config(void);

uint32_t phy_getHclkFreq(void);

#endif
