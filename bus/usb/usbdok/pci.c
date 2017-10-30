/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003
 *
 * PCI related routines implementation
 */

#include "low.h"
#include "io.h"
#include "pci.h"


/* Function finds the device given by PCI device identifier */
pci_dev_t *pci_find_device(pci_id_t *id, u16 n, int mode, pci_dev_t *pdev)
{
  int k;
  u16 d, v, handle;
  u8 irq;
  u32 base, c;

  /* Check the PCI BIOS */
  asm {
    mov ax, 0b101h
    int 1ah
  }
  if (_AH)
    return NULL;

  /* Find the device given by id */
  if (mode == PCI_FIND_DEVICE) {
    d = id->device;
    v = id->vendor;
    asm {
      mov cx, d
      mov dx, v
      mov si, n
      mov ax, 0b102h
      int 1ah
      mov [handle], bx
    }
    if (_AH)
      return NULL;
  }
  else {
    c = id->class;
    asm {
      mov ecx, c
      mov si, n
      mov ax, 0b103h
      int 1ah
      mov [handle], bx
    }
    if (_AH)
      return NULL;
  }

  pdev->handle = handle;

  /* Get the device revision */
  asm {
    mov bx, [handle]
    mov di, 8
    mov ax, 0b108h
    int 1ah
    mov irq, cl
  }
  if (_AH)
    return NULL;

  /* Get the device IRQ */
  asm {
    mov bx, [handle]
    mov di, 3ch
    mov ax, 0b108h
    int 1ah
    mov irq, cl
  }
  if (_AH)
    return NULL;

  pdev->irq = irq;

  /* Get the device base address */
  for (k = 0; k < 6; k++) {
    asm {
      mov bx, [handle]
      mov di, k
      shl di, 2
      add di, 10h
      mov ax, 0b10ah
      int 1ah
      mov base, ecx
    }
    if (_AH)
      return NULL;

    if (!k || (base & 1))
      pdev->base = base & 0xfffffffe;
  }
  return pdev;
}


/* Function writes configuration space word */
int pci_confwritel(pci_dev_t *pdev, u16 offs, u32 v)
{
  u16 handle = pdev->handle;

  asm {
    mov bx, [handle]
    mov di, offs
    mov ax, 0b10dh
    mov ecx, v;
    int 1ah
  }
  if (_AH)
    return -1;
  
  return 0;
}


/* Function reads configuration space word */
int pci_confreadl(pci_dev_t *pdev, u16 offs, u32 *v)
{
  u16 handle = pdev->handle;
  u32 d;

  asm {
    mov bx, [handle]
    mov di, offs
    mov ax, 0b10ah
    int 1ah
    mov d, ecx
  }
  if (_AH)
    return -1;

  *v = d;
  return 0;
}

