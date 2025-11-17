/*
 * Phoenix-RTOS
 *
 * phy - depends on the platform
 *
 * Copyright 2025 Phoenix Systems
 * Author: Olaf Czerwinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _PHY_H_
#define _PHY_H_

#include <stdlib.h>

/* Function returns buffer which is a multiple of USB_BUFFER_SIZE.
 * Otherwise it returns null.                                               */
void *usbclient_allocBuff(uint32_t size);


/* Function cleans the whole memory assigned to endpoints and setup memory. */
void usbclient_buffDestroy(void *addrs, uint32_t size);

/* Function sets the clock for USB Controller */
void phy_setClock(void);

/* Function returns ID of USB controller interrupt.                         */
uint32_t phy_getIrq(void);


/* Function returns physical address of USB controller.                     */
void *phy_getBase(uint32_t size);


/* Function initializes pins and clocks associated with USB controller.     */
void phy_init(void);


/* Function resets physical layer of USB controller.                        */
void phy_reset(void);


#endif