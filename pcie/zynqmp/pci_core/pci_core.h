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
#include <sys/rb.h>
#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include "pci_utils.h"

typedef int irqreturn_t;
typedef irqreturn_t (*irq_handler_t)(int, void *);

typedef int pci_power_t;

#define PCI_FPGA_GPIO_ADD                 0xa0020000
#define PCI_ROOT_COMPLEX_BASE_ADD         0x500000000
#define PCI_ROOT_COMPLEX_IRQ_NO           123 /* Connected to PL-PS interrupt, for current bitstrem PL-PS irq no. 2 */
#define PCI_IRQ_HANDLER_THREAD_PRIO       1
#define PCI_IRQ_HANDELR_THREAD_STACK_SIZE 4096

#define PCI_BAR0    0x10
#define PCI_CAP_PTR 0x34

#define PCI_MAX_BAR_COUNT 6
#define PCI_CONTEXT_IRQ_BUF_SIZE 16

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

/* Address ranges for MSI */
/* Keep in mid that those will be intercepted by root complex */
/* so those will never end up on interconnect, nor will they */
/* be translated by any IOMMU mechanism. This address has to */
/* be 4KiB alligned. */
/* TBH this can stay as 0 for now. AFAIK this does not matter */
/* since it will never end up on the bus */
/* For now we will leave here 8KiB address range for MSI, */
/* this allows for 4K MSI interrupt sources (this is limit for Xlinix RC) */
#define PCI_MSI_BA    0x0ul
#define PCI_MSI_LIMIT 0x1000ul

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

/* Capability structure IDs */
#define PCI_CAP_MSI_ID 0x05

/* Used for a work-around since default plo does not provide some configurations */
#define AXI_HPM1_FPD_REG   0x00FD615000
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
	uint32_t msi_payload;
	rbnode_t msi_payload_rbnode;

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

	pci_power_t current_state;
} pci_dev_t;

typedef enum {
	INTA,
	INTB,
	INTC,
	INTD
} pci_intx_line;

typedef enum {
	PCI_INTX_IRQ,
	PCI_MSI_IRQ,
	PCI_MSIX_IRQ
} pci_interrupt_type;

/* */
typedef struct pci_irq_info {
	uint16_t requester_id;
	pci_interrupt_type type;
	union {
		struct{
			uint32_t msi_address;
			uint32_t msi_payload;
		};
		pci_intx_line intx_line;
	};
} pci_irq_info_t;

/* */
typedef struct pci_context {
	void *fpga_gpio;
	void *pcie;

	pci_dev_t device_root;

	uint32_t rc_status_reg;
	pci_irq_info_t irq_queue[PCI_CONTEXT_IRQ_BUF_SIZE];
	int irq_queue_rd_ptr;
	int irq_queue_wr_ptr;

	rbtree_t msi_rb_tree;
} pci_context_t;

/* Init PCI context */
int pci_initContext(pci_context_t *context);
/* Set up AXI to work with PCIe root complex IP core */
void pci_prepareAXI(void);
/* Reset AXI and root complex */
void pci_resetAXIandRC(pci_context_t *context);
/* */
void pci_readBridgeInfoReg(uint32_t *pcie);
/* */
void pci_irqHandlerThread(void *);
/* */
int pci_irqCallback(unsigned int irq_no, void *arg);

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
/* Configure capabilities of the bridg9 */
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
/* */
void pci_setMaxPayloadSize(uint32_t *pcie, pci_dev_t *device);
/* */
void pci_enableTimeoutTLP(uint32_t *pcie, pci_dev_t *dev);
/* */
void pci_disableCompletionTimeout(uint32_t *pcie, pci_dev_t *device);
/* Find offset from BDF ecam addresing at which capability with ID resides, if fail return 0 */
uint16_t pci_findCapability(uint32_t *pcie, pci_dev_t *dev, uint32_t id);
/* Enable MSI for given device */
int pci_enableMsiInterrupt(pci_context_t *context, pci_dev_t *device);

/* Will return valid base address for I/O BAR */
uintptr_t pci_getNextIORange(size_t bar_length);
/* Will return valid physical base address for 32-bit non-prefetchble BAR*/
uintptr_t pci_getNext32RangeNonPref(size_t bar_length);
/* Will return valid physical base address for 32-bit prefetchble BAR*/
uintptr_t pci_getNext32Range(size_t bar_length);
/* Will return valid physical base address for 64-bit prefetchble BAR*/
uintptr_t pci_getNext64Range(size_t bar_length);
/* */
unsigned long pci_getNextMSIadd(void);
/* Generate new ID for the device to use as MSI payload */
uint32_t pci_getNextMsiPayload(void);

#endif
