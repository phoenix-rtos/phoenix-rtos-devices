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
#include "pci_utils.h"

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

/* This struct is used to create map of PCIe fabric */
typedef struct pci_dev {
	uint16_t vid;
	uint16_t pid;

	struct pci_dev *parent; /* Pointer to parent PCIe device */
	struct pci_dev *next;   /* Pointer to sibling - same depth */
	struct pci_dev *child;  /* Children of this device -
	if this is an endpoint set to NULL */
	/* ECAM addressing of the device */
	uint8_t bus;
	uint8_t dev;
	uint8_t func;

	/* BAR settings */
	uintptr_t bar[PCI_MAX_BAR_COUNT];
	size_t size[PCI_MAX_BAR_COUNT];
} pci_dev_t;

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

/* Will return valid base address for I/O BAR */
uintptr_t pci_getNextIORange(size_t bar_length);
/* Will return valid physical base address for 32-bit non-prefetchble BAR*/
uintptr_t pci_getNext32RangeNonPref(size_t bar_length);
/* Will return valid physical base address for 32-bit prefetchble BAR*/
uintptr_t pci_getNext32Range(size_t bar_length);
/* Will return valid physical base address for 64-bit prefetchble BAR*/
uintptr_t pci_getNext64Range(size_t bar_length);

#endif
