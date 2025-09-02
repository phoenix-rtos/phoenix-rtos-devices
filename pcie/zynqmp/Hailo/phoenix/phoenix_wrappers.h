#ifndef PHOENIX_WRAPPERS
#define PHOENIX_WRAPPERS

#include "../../core/pci_core.h"
#include "phoenix_comp.h"

#define PCI_EXP_DEVCTL              0x08   /* Device Control */
#define PCI_EXP_DEVCTL_READRQ       0x7000 /* Max_Read_Request_Size */
#define PCI_EXP_DEVCTL_READRQ_128B  0x0000 /* 128 Bytes */
#define PCI_EXP_DEVCTL_READRQ_256B  0x1000 /* 256 Bytes */
#define PCI_EXP_DEVCTL_READRQ_512B  0x2000 /* 512 Bytes */
#define PCI_EXP_DEVCTL_READRQ_1024B 0x3000 /* 1024 Bytes */
#define PCI_EXP_DEVCTL_READRQ_2048B 0x4000 /* 2048 Bytes */
#define PCI_EXP_DEVCTL_READRQ_4096B 0x5000 /* 4096 Bytes */

extern uint32_t *phoenix_wrapper_pci_base;

int pci_enable_device(struct pci_dev *dev);

void pci_set_master(struct pci_dev *dev);

int pci_request_regions(struct pci_dev *dev, const char *name);

size_t pci_resource_len(struct pci_dev *dev, int n);

void *pci_iomap(struct pci_dev *dev, int n, size_t dummy);

void pci_iounmap(struct pci_dev *dev, void *addr);

void pci_release_regions(struct pci_dev *dev);

int pcie_capability_read_word(pci_dev_t *device, uint32_t off, uint16_t *ret);

int pcie_capability_clear_word(pci_dev_t *device, uint32_t off, u16 mask);

void pci_set_drvdata(struct pci_dev *dev, void *data);

void *pci_get_drvdata(struct pci_dev *dev);

int pci_set_power_state(struct pci_dev *dev, uint32_t state);

void pci_disable_device(struct pci_dev *dev);

void pci_disable_msi(struct pci_dev *dev);

void free_irq(uint32_t irq, void *p);

int pci_enable_msi(struct pci_dev *dev);

int request_irq(struct pci_dev *dev, irq_handler_t callback, int flags, const char * driver_name, void *arg);

#endif
