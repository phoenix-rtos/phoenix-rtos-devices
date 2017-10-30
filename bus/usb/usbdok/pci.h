/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003
 *
 * PCI related routines
 */

#ifndef _PCI_H_
#define _PCI_H_

#include "low.h"


#define PCI_VENDOR_ID_INTEL   0x8086

#define PCI_ANY_ID                     0


/* Find modes */
#define PCI_FIND_DEVICE  0
#define PCI_FIND_CLASS   1


typedef struct _pci_dev_t {
  u8  irq;
  u32 base;
  u16 handle;
} pci_dev_t;


typedef struct _pci_id_t {
  u16 vendor;
  u16 device;
  u32 class;
} pci_id_t;


/* Function finds the device given by PCI identifier */
extern pci_dev_t *pci_find_device(pci_id_t *id, u16 n, int mode, pci_dev_t *pdev);


/* Function writes configuration space word */
extern int pci_confwritel(pci_dev_t *pdev, u16 offs, u32 v);


/* Function reads configuration space word */
extern int pci_confreadl(pci_dev_t *pdev, u16 offs, u32 *v);


#endif
