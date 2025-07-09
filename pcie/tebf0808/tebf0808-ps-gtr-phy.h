/******************************************************************************
 * Copyright (C) 2010-2020 Xilinx, Inc.  All rights reserved.
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

/*
 * Phoenix-RTOS
 *
 * PCI Express PS PHY initialisation code for TEBF0808 baseboard + TE0807 SOM
 * Code is derived from Trenz Electronic example design.
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#ifndef TEBF0808_PS_GTR_PHY_H
#define TEBF0808_PS_GTR_PHY_H

/**
 * Initialise PS GTR PHY
 *
 * Return values:
 * - 0: Operation succeed
 * - others values: Operation failed
 */
int tebf0808_pciePsGtrPhyInit(void);

#endif
