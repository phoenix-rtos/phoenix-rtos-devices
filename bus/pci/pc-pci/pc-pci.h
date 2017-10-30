/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * PCI enumerator
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2006 Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_PCI_H_
#define _DEV_PCI_H_


#define PCI_ANY            0
#define PCI_VENDOR_INTEL   0x8086

typedef struct _pci_id_t {
	u16 vendor;
	u16 device;
	u16 subvendor;
	u16 subdevice;
	u16 cl;
} pci_id_t;


typedef struct _pci_resource_t {
	u32 base;
	u32 limit;
	u8 flags;
} pci_resource_t;


typedef struct _pci_device_t {
	unsigned int usage;
	u8 b; /* bus */
	u8 d; /* device */
	u8 f; /* function */ 
	u16 device;
	u16 vendor;
	u16 status;
	u16 command;
	u16 cl; /* class and subclass */
	u8 progif;
	u8 revision;
	u8 irq;
	u8 type;
	pci_resource_t resources[6];
  
	struct _pci_device_t *next;
	struct _pci_device_t *prev;
} pci_device_t;


/* Function allocates PCI device described by id structure */
extern int dev_pciAlloc(const pci_id_t *id, pci_device_t **adev);
extern void dev_setBusmaster(pci_device_t *dev, u8 enable);

/* Function enumerates PCI bus */
extern void _pci_init(void);

#endif
