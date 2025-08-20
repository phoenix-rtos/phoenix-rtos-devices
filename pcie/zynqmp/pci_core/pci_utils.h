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


#ifndef PCI_UTILS_H
#define PCI_UTILS_H

#include <stdint.h>
#include <stdatomic.h>

struct device {
	void *dirver_data;
};

/* Adding read/write memory barriers */
#define rmb() atomic_thread_fence(memory_order_acquire)
#define wmb() atomic_thread_fence(memory_order_release)

#define ECAM_BUS_SHIFT  20
#define ECAM_DEV_SHIFT  15
#define ECAM_FUNC_SHIFT 12

#define PCI_VENDOR_ID     0x00
#define PCI_DEVICE_ID     0x02
#define PCI_COMMAND       0x04
#define PCI_CMD_IO        0x01
#define PCI_CMD_MEM       0x02
#define PCI_CMD_MASTER    0x04
#define PCI_STATUS        0x06
#define PCI_CLASSCODE     0x08
#define PCI_HEADER_TYPE   0x0e
#define PCI_HT_MULTI_FUNC 0x80
#define PCI_BIST          0x0f


#define PCI_PRIMARY_BUS     0x18
#define PCI_SECONDARY_BUS   0x19
#define PCI_SUBORDINATE_BUS 0x1a


/* Write 32-bit register at base + offset */
void writeReg(uint32_t *base, uint32_t offset, uint32_t value);
/* Read 32-bit register at base + offset */
uint32_t readReg(uint32_t *base, uint32_t offset);
/* */
void writeRegMsk(uint32_t *base, uint32_t offset, uint32_t clr, uint32_t set);
/* Returns pointer to register of PCIe device */
volatile uint32_t *ecamRegPtr(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg);
/* Write word to PCIe device configuration header */
void ecamWrite32(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val);
/* Read word from PCIe device configuration header */
uint32_t ecamRead32(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);
/* Read half-word -//-*/
uint16_t ecamRead16(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);
/* Read byte -//- */
uint8_t ecamRead8(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off);

#endif
