#include "phoenix_wrappers.h"
#include <sys/interrupt.h>

uint32_t *phoenix_wrapper_pci_base = NULL;

int pci_enable_device(struct pci_dev *dev)
{
	pci_enableEndpoint(phoenix_wrapper_pci_base, dev);
	return 0;
}

void pci_set_master(struct pci_dev *dev)
{
	(void)dev;
	/* Place holder - pci_enableEndpoint alread will set master */
	return;
}

int pci_request_regions(struct pci_dev *dev, const char *name)
{
	(void)name;
	(void)dev;
	/* Place holder - we don't need to claim BARs */
	return 0;
}

size_t pci_resource_len(struct pci_dev *dev, int n)
{
	return dev->size[n];
}

void *pci_iomap(struct pci_dev *dev, int n, size_t dummy)
{
	(void)dummy;
	return (void *)dev->bar[n];
}


void pci_iounmap(struct pci_dev *dev, void *addr)
{
	int i;

	for (i = 0; i < 6; i++) {
		if ((void *)dev->bar[i] == addr) {
			break;
		}
	}

	munmap(addr, dev->size[i]);
}


void pci_release_regions(struct pci_dev *dev)
{
	(void)dev;
	return;
}


int pcie_capability_read_word(pci_dev_t *device, uint32_t off, uint16_t *ret)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* Check if there is capabilities list */
	uint16_t status = ecamRead16((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return -1;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, ptr);
		uint8_t next = ecamRead8((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, ptr + 1);

		if (0x10 == cap_id) {
			*ret = ecamRead16((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, ptr + off);
			return 0;
		}

		if (next == 0) {
			break;
		}
		ptr = next;
	}

	return -1;
}

int pcie_capability_clear_word(pci_dev_t *device, uint32_t off, u16 mask)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* Check if there is capabilities list */
	uint16_t status = ecamRead16((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return -1;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, ptr);
		uint8_t next = ecamRead8((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, ptr + 1);

		if (0x10 == cap_id) {
			u8 a = off % 4;
			off -= a;
			u32 reg = ecamRead32((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, ptr + off);
			if (a == 0) {
				reg &= mask | (0xFFFF << 16);
			}
			else {
				reg &= (mask << 16) | 0xFFFF;
			}
			ecamWrite32((uintptr_t)phoenix_wrapper_pci_base, bus, dev, fun, ptr + off, reg);
			return 0;
		}

		if (next == 0) {
			break;
		}
		ptr = next;
	}

	return -1;
}

void pci_set_drvdata(struct pci_dev *dev, void *data)
{
	dev->dev.dirver_data = data;
}

void *pci_get_drvdata(struct pci_dev *dev){
	return dev->dev.dirver_data;
}

int pci_set_power_state(struct pci_dev *dev, uint32_t state){
	/* For now do nothing, keep device in full-power L0 state */
	/* ID of power managment capability structure is 0x01 */
	return 0;
}

void pci_disable_device(struct pci_dev *dev){
	/* Ignore for now */
	return;
}

void pci_disable_msi(struct pci_dev *dev){
	/* For now since root complex keeps interrupts as disabled, disabling interrupts on the side of the device is sufficent */
    /* ID of MSI-X capability structure is 0x11 */
	return;
}

void free_irq(uint32_t irq, void *p)
{
    /* Clear stuff */
    return;
}

int pci_enable_msi(struct pci_dev *dev)
{
	return 0;
}

int request_irq(struct pci_dev *dev, irq_handler_t callback, int flags, const char *driver_name, void *arg)
{
	/* Set private interrupt handler, called from generic PCIe interrupt handler */
	dev->private_handler = callback;
	dev->handler_data = arg;
	return 0;
}
