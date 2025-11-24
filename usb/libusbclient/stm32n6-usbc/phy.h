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


#ifndef _PHY_H_
#define _PHY_H_

#include <stdlib.h>


/* BASE ADRESSES */
#define PHY_ADDR_OTG     0x58040000
#define PHY_ADDR_USBPHYC 0x5803FC00
#define RCC_BASE_ADDR    0x56028000

#define PHY_IRQ otg1_irq

/* REGISTER OFFSETS */
#define USBPHYC_CR (0x000 / 4)

#define GOTGCTL (0x000 / 4)
#define GAHBCFG (0x008 / 4)
#define GUSBCFG (0x00C / 4)
#define GRSTCTL (0x010 / 4)
#define GINTSTS (0x014 / 4)
#define GINTMSK (0x018 / 4)
#define GCCFG   (0x038 / 4)
#define DCFG    (0x800 / 4)
#define DCTL    (0x804 / 4)
#define DSTS    (0x808 / 4)

/* RC OFFSETS */
#define RCC_AHB5RSTSR 648   // 0xA20 / 4
#define RCC_AHB5RSTCR 1160  // 0x1220 / 4

#define USBPHYC_CR_FSEL_24MHZ (0x2 << 4)


/* Function returns buffer which is a multiple of USB_BUFFER_SIZE.
 * Otherwise it returns null.
 */
void *usbclient_allocBuff(uint32_t size);


/* Function cleans the whole memory assigned to endpoints and setup memory */
void usbclient_buffDestroy(void *addrs, uint32_t size);

/* Function sets the clock for USB Controller */
void phy_setClock(void);

/* Function returns ID of USB controller interrupt */
uint32_t phy_getIrq(void);

/* Returns pointer to OTG register base */
volatile uint32_t *phy_getOtgBase(void);


/* Function returns physical address of USB controller */
void *phy_getBase(uint32_t size);


/* Function initializes pins and clocks associated with USB controller */
int phy_init(void);


/* Function resets physical layer of USB controller */
void phy_reset(void);


#endif
