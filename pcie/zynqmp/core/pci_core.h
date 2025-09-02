/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */


#ifndef PCI_CORE_H
#define PCI_CORE_H

#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include <sys/platform.h>
#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include "pci_utils.h"

typedef int irqreturn_t;
typedef irqreturn_t (*irq_handler_t)(int, void *);

#define PCI_ROOT_COMPLEX_IRQ_NO 123 /* Connected to PL-PS interrupt, for current bitstrem PL-PS irq no. 2 */

#define PCI_BAR0    0x10
#define PCI_CAP_PTR 0x34

#define PCI_MAX_BAR_COUNT 6

#define PCI_BAR0_ADD 0x520000000
#define PCI_BAR1_ADD 0x530000000
#define PCI_BAR2_ADD 0x540000000
#define PCI_BAR3_ADD 0x550000000
#define PCI_BAR4_ADD 0x560000000
#define PCI_BAR5_ADD 0x570000000

/* In current bitstream there is no 32 bit addresses */
/* Not implemented*/
#define PCI_IO_BAR_BA    0
#define PCI_IO_BAR_LIMIT 0

/* Not implemented */
#define PCI_32NP_BAR_BA    0
#define PCI_32NP_BAR_LIMIT 0

/* Not implemented */
#define PCI_32P_BAR_BA    0
#define PCI_32P_BAR_LIMIT 0

#define PCI_64P_BAR_BA    PCI_BAR0_ADD
#define PCI_64P_BAR_LIMIT PCI_BAR5_ADD

/* PCIe bridge (type 1 header) offsets */
#define PCI_BRIDGE_VID            0x00
#define PCI_BRIDGE_CMD            0x04
#define PCI_BRIDGE_REV            0x08
#define PCI_BRIDGE_CACHE_LINE     0x0C
#define PCI_BRIDGE_BAR0           0x10
#define PCI_BRIDGE_BAR1           0x14
#define PCI_BRIDGE_PRIM_BUS_NUM   0x18
#define PCI_BRIDGE_IO_BASE        0x1C
#define PCI_BRIDGE_MEM_BASE       0x20
#define PCI_BRIDGE_MEM_PREF_BASE  0x24
#define PCI_BRIDGE_MEM_PREF_UPPER 0x28
#define PCI_BRIDGE_MEM_PREF_LIM   0x2C
#define PCI_BRIDGE_IO_UPPER       0x30
#define PCI_BRIDGE_CAPABILITY     0x34
#define PCI_BRIDGE_ROM_BASE       0x38
#define PCI_BRIDGE_IRQ_LINE       0x3C

/* Used for a work-around since default plo does not provide some configurations */
#define AXI_HPM1_FPD_REG 0x00FD615000
#define AXI_WIDTH_CFG_32B  0x0
#define AXI_WIDTH_CFG_64B  0x1
#define AXI_WIDTH_CFG_128B 0x2

/* This struct is used to create map of PCIe fabric */
typedef struct pci_dev {
	uint16_t vid;
	uint16_t pid;
	uint16_t vendor;
	uint16_t device;

	struct device dev;
	uint32_t irq;
	irq_handler_t private_handler;
	void *handler_data;

	struct pci_dev *parent; /* Pointer to parent PCIe device */
	struct pci_dev *next;   /* Pointer to sibling - same depth */
	struct pci_dev *child;  /* Children of this device -
	if this is an endpoint set to NULL */
	/* ECAM addressing of the device */
	uint8_t bus_no;
	uint8_t dev_no;
	uint8_t func_no;

	/* BAR settings */
	uintptr_t bar[PCI_MAX_BAR_COUNT];
	size_t size[PCI_MAX_BAR_COUNT];
} pci_dev_t;

/* Set up AXI to work with PCIe root complex IP core */
void pci_prepareAXI(void);
/*  */
void pci_resetAXIandRC(uint32_t *gpioBase);
/* */
void pci_readBridgeInfoReg(uint32_t *pcie);

/* Display PCIe fabric tree */
void pci_printTree(pci_dev_t *root, int depth);
/* */
pci_dev_t *pci_getDevFromTree(pci_dev_t *root, uint32_t vid, uint32_t pid);
/* Display BAR config */
void pci_printBARs(uint32_t *pcie, pci_dev_t *device);
/* Display capabilities */
void print_capabilities(uint32_t *pcie, pci_dev_t *device);

/* We want to check topology of the bus */
void pci_enumerateRoot(uint32_t *pcie, pci_dev_t *root, uint8_t bus, int depth);
/* Scaning selected function of the device */
void pci_scanFunc(uint32_t *pcie, pci_dev_t *device, int depth);
/* Configure bridge */
void pci_configureBridge(uint32_t *pcie, pci_dev_t *device, int depth);
/* Maps physical address ranges to root complex */
void pci_mapBARsPhysAddToBridge(uint32_t *pcie, pci_dev_t *device);
/* Configure capabilities of the bridge - currently set timeout for invalid TLPs */
void pci_configureCapBridge(uint32_t *pcie, pci_dev_t *device);
/* Enable PCIe bridge */
void pci_bridgeEnable(uint32_t *pcie, pci_dev_t *device);
/* Configure endpoint */
void pci_configureEndpoint(uint32_t *pcie, pci_dev_t *device);
/* Configure endpoint BARs */
void pci_configureEndpointBARs(uint32_t *pcie, pci_dev_t *device);
/* Function will enable endpoint passed by device */
void pci_enableEndpoint(uint32_t *pcie, pci_dev_t *device);
/* Enable root complex to pass AXI accesses to BARs */
void pci_enableRootComplex(uint32_t *pcie);
/* */
void pci_dumpBAR(pci_dev_t *device, int bar_number);
/* Generic PCI interrupt handler */
int pci_irqHandler(unsigned int no, void *data);

/* Will return valid base address for I/O BAR */
uintptr_t pci_getNextIORange(size_t bar_length);
/* Will return valid physical base address for 32-bit non-prefetchble BAR*/
uintptr_t pci_getNext32RangeNonPref(size_t bar_length);
/* Will return valid physical base address for 32-bit prefetchble BAR*/
uintptr_t pci_getNext32Range(size_t bar_length);
/* Will return valid physical base address for 64-bit prefetchble BAR*/
uintptr_t pci_getNext64Range(size_t bar_length);

#endif
