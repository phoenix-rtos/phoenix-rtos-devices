/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala, Mikolaj Matalowski
 *
 * %LICENSE%
 */


#include "pci_utils.h"


/* Write 32-bit register at base + offset */
void writeReg(uint32_t *base, uint32_t offset, uint32_t value)
{
	*((volatile uint32_t *)((char *)base + offset)) = value;
}


/* Read 32-bit register at base + offset */
uint32_t readReg(uint32_t *base, uint32_t offset)
{
	return *((volatile uint32_t *)((char *)base + offset));
}


/* */
void writeRegMsk(uint32_t *base, uint32_t offset, uint32_t clr, uint32_t set)
{
	uint32_t value = readReg(base, offset);
	value &= ~clr;
	value |= set;
	writeReg(base, offset, value);
}


/* Returns pointer to register of PCIe device */
volatile uint32_t *ecamRegPtr(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg)
{
	uintptr_t cfg_space_offset = ((uintptr_t)bus << ECAM_BUS_SHIFT) |
			((uintptr_t)dev << ECAM_DEV_SHIFT) |
			((uintptr_t)fn << ECAM_FUNC_SHIFT) |
			(reg & 0xfffu);

	return (volatile uint32_t *)((uintptr_t)pcie + cfg_space_offset);
}


/* Write word to PCIe device configuration header */
void ecamWrite32(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	*ecamRegPtr(pcie, bus, dev, fn, off & ~0x3u) = val;
	wmb();
}


/* Read word from PCIe device configuration header */
uint32_t ecamRead32(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	rmb();
	uint32_t ret = *ecamRegPtr(pcie, bus, dev, fn, off & ~0x3u);
	return ret;
}


/* Read half-word -//-*/
uint16_t ecamRead16(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = ecamRead32(pcie, bus, dev, fn, off);
	if (off & 2) {
		return value_u32 >> 16;
	}
	else {
		return value_u32 & 0xffff;
	}
}


/* Read byte -//- */
uint8_t ecamRead8(uintptr_t pcie, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = ecamRead32(pcie, bus, dev, fn, off);
	return (value_u32 >> ((off & 3) * 8)) & 0xff;
}
