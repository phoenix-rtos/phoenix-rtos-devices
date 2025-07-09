/*
 * Phoenix-RTOS
 *
 * TEBF0808 evaluation board PCI Express reference clock initialisation
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#ifndef TEBF0808_PCIE_REFCLK_H
#define TEBF0808_PCIE_REFCLK_H

/**
 * Initialise PCI Express reference clock on TEBF0808 board
 *
 * Return values:
 * - 0: Operation succeed
 * - others values: Operation failed
 */
int tebf0808_pcieRefClkInit(void);

#endif
