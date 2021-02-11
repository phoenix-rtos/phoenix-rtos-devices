/*
 * Phoenix-RTOS
 *
 * phy - depends on the platform
 *
 * Copyright 2020-2021 Phoenix Systems
 * Author: Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _PHY_H_
#define _PHY_H_

#include <stdlib.h>


void *usbclient_allocBuff(uint32_t size);


void usbclient_buffDestory(void *addrs, uint32_t size);


void phy_setClock(void);


uint32_t phy_getIrq(void);


void *phy_getBase(uint32_t size);


void phy_reset(void);


void phy_init(void);


#endif
