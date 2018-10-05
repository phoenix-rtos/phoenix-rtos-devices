/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/phy.h
 *
 * Copyright 2018 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_USB_HOST_PHY_
#define _IMX6ULL_USB_HOST_PHY_

extern void phy_dumpRegisters(FILE *stream);


extern void phy_config(void);


extern void phy_reset(void);


extern void phy_init(void);

#endif
