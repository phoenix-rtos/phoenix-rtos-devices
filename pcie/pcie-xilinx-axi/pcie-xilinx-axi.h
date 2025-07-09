/*
 * Phoenix-RTOS
 *
 * PCI Express Xilinx AXI bridge driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#ifndef PCIE_XILINX_AXI_H
#define PCIE_XILINX_AXI_H

/**
 * Initialise PCI Express Xilinx PCIE AXI driver
 *
 * Return values:
 * - 0: Operation succeed
 * - others values: Operation failed
 */
int pcie_xilinx_axi_init(void);

#endif
