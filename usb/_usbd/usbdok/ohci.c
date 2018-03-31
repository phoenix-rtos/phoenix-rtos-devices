/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2010
 *
 * OHCI driver implementation
 */

#include "low.h"
#include "io.h"
#include "pci.h"
#include "usb.h"
#include "ohci.h"


/* Detection automa delays */
#define DELAY_DETECT   1
#define DELAY_RESET    50
#define DELAY_ENABLE   1
#define DELAY_CONFIG   1


static struct {
  u16 base;
  u8 irq;
  u16 oldisr[2];
  u8 hcca[256 + 128 + 2 + 2 + 4 + 116];
  volatile u8 *hccaptr;
  u8 heap[512 + 128];
  void *freeptr;
} ohci;


static u8 gdt[] = {
  0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // gdtr
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // null descriptor
  0xff, 0xff, 0x00, 0x00, 0x00, 0x92, 0xcf, 0x00  // data descriptor
};


void _ohci_unlimitfs(void)
{
  asm {
    xor eax, eax
    mov ax, ds
    shl eax, 4
    lea ebx, gdt
    add eax, ebx
    mov [ebx + 2], eax
    lgdt [ebx]
    mov eax, cr0
    or eax, 1
    mov cr0, eax
    mov bx, 8
    and eax, 0fffffffeh
    mov cr0, eax
  }
  return;
}


void ohci_outl(u16 reg, u32 v)
{
  u16 base = ohci.base;

  asm {
    mov ebx, v
    mov ax, base
    mov di, reg
    push ds    
    mov ds, ax
    mov [di], ebx
    pop ds
  }
  return;
}


u32 ohci_inl(u16 reg)
{
  u16 base = ohci.base;
  u32 v;

  asm {
    mov ax, base
    mov di, reg
    push ds
    mov ds, ax 
    mov ebx, [di]
    pop ds
    mov v, ebx
  }
  return v;
}


void *ohci_alloc(void)
{
  void *p = ohci.freeptr;

  if (p != NULL)
    ohci.freeptr = *(void **)ohci.freeptr;

  return p;
}


void ohci_free(void *p)
{
  if (p == NULL)
    return;

  *(void **)p = ohci.freeptr;
  ohci.freeptr = p;

  return;
}


/* Function suspend executions for t miliseconds */
void ohci_sleep(u16 t)
{
  u16 ct;

  for (; t; t -= (t < 32768 ? t : 32768))
    for (ct = (ohci_inl(HCFMNUMBER) & 0xffff);
      ((ohci_inl(HCFMNUMBER) - ct) & 0xffff) < (t < 32768 ? t : 32768); );

  return;
}


/* Function checks given hub port */
int ohci_checkhub(unsigned int k)
{
  u32 status;

  status = ohci_inl(HCRHPORTSTATUS + k * 4);

#ifdef _TRACE
  io_printf("ohci_check_hub: port %d status %x %x\n", k, (u16)(status >> 16), (u16)status);
#endif

  /* Check port */
  if (!(status & 1))
    return 0;
  if (status & 2)
    return 1;

  /* Check if status changed */
/*
  if (!(status & 0x10000))
    return 0;
*/

  ohci_sleep(DELAY_DETECT);
  if (!(status & 1))
    return 0;

  /* start port reset */
  ohci_outl(HCRHPORTSTATUS + k * 4, 0x10010);

  /* port reset done */
  ohci_sleep(DELAY_RESET);
  ohci_outl(HCRHPORTSTATUS + k * 4, 0x100000);

  /* port enable */
  ohci_sleep(DELAY_ENABLE);
  ohci_outl(HCRHPORTSTATUS + k * 4, 0x2);

  /* device configure */
  ohci_sleep(DELAY_CONFIG);

  if ((ohci_inl(HCRHPORTSTATUS + k * 4) & 0x10000) ||
    (usb_insertion(k) < 0)) {
    ohci_outl(HCRHPORTSTATUS + k * 4, 0x1);
    return 0;
  }

  return 0;
}


void ohci_interrupt(void)
{
  u32 status;

  while ((status = ohci_inl(HCINTERRUPTSTATUS)) != 0) {
    io_printf("ohci_interrupt: status %x %x\n", (u16)(status >> 16), (u16)status);
    ohci_outl(HCINTERRUPTSTATUS, status);
  }

  low_intack();
  return;
}


/* Functions transmits data over USB */
int ohci_transmit(urb_t *urb, u8 pid, u32 buffer, u16 len)
{
  u16 t;
  u32 mps, size;
  u32 addr;
  int res;
  u16 origlen = len;
  ohci_ed_t *ed;
  ohci_td_t *td, *last = NULL;

  if ((ed = (ohci_ed_t *)ohci_alloc()) == NULL)
    return ERR_USB_MEM;

  /* Prepare trasmission descriptor chain (MOD) */
  size = mps = (urb->ep ? 64 : urb->mtu);
  urb->dev &= 0x7f;
  urb->ep &= 0x0f;
  urb->toggle &= 1;

  if (urb->ep)
    size = 4096;

  ed->flags = ((mps & 0x7ff) << 16) | ((u32)urb->ep << 7) | urb->dev;
  ed->tail = 0;
  ed->head = urb->toggle << 1;
  ed->next = 0;

#ifdef _TRACE
  io_printf("LEN %d %d\n", origlen, len);
#endif

  for (last = NULL;;) {
    if ((td = (ohci_td_t *)ohci_alloc()) == NULL) {
      while (ed->head & ~0xf) {
        td = get_offs(ed->head & ~0xf);
        ed->head = td->next;
        ohci_free(td);
      }
      ohci_free(ed);
      return ERR_USB_MEM;
    }

    if (last == NULL)
      ed->head |= get_addr(td);
    else
      last->next = get_addr(td);

    /* DelayInterrupt = 000b | DataToggle = ep->toggle */
    td->flags = (0L << 21) | (0L << 24);

    last = td;

    if (pid == pidOUT)
      td->flags |= 1L << 19;
    else if (pid == pidIN)
      td->flags |= 2L << 19;

    td->cbp = len ? buffer : 0;
    td->be = buffer + (len > size ? size : len) - 1;
    td->next = 0;

    buffer += size;
    len -= (len > size ? size : len);

    if (!len)
      break;
  }

  /* Launch transfer and wait for completion */
  ohci_outl(HCINTERRUPTSTATUS, 2);
  ohci_outl(urb->ep ? HCBULKHEADED : HCCONTROLHEADED, get_addr(ed));
  ohci_outl(HCCONTROL, ohci_inl(HCCONTROL) | 0x30L);
  ohci_outl(HCCOMMANDSTATUS, 0x6);

  t = (u16)ohci_inl(HCFMNUMBER);
  res = 0;

  while (ed->head & ~0xf) {
    if (((u16)ohci_inl(HCFMNUMBER) - t) > USB_TIMEOUT) {
      res = ERR_USB_TIMEOUT;
      break;
    }

    if (ed->head & 1) {
      res = ERR_USB_TRANSMIT;
      break;
    }
  }

  ohci_outl(HCCONTROL, ohci_inl(HCCONTROL) & ~0x30);
  t = (u16)ohci_inl(HCFMNUMBER);

  len = origlen;

  addr = *(u32 *)(ohci.hccaptr + 0x84) & ~0xf;
  if (addr) {
    while (addr) {
      td = (ohci_td_t *)get_offs(addr);

#ifdef _TRACE
  if (td->flags >> 28)
    io_printf("ERROR %x\n", (u16)(td->flags >> 28));
#endif

      if (res == ERR_USB_TRANSMIT) {
        switch (td->flags >> 28) {
          case 0x9:
            res = 0;
            break;
          case 0x4:
            res = ERR_USB_STALL;
            break;
        }
      }

      len -= td->cbp ? td->be + 1 - td->cbp : 0;
      addr = td->next;
      ohci_free(td);
    }
    *(u32 *)(ohci.hccaptr + 0x84) = 0;
    ohci_outl(HCINTERRUPTSTATUS, 2);
  }

  while ((t == ohci_inl(HCFMNUMBER)) || (ohci_inl(HCDONEHEAD) & ~0xf));

  ohci_outl(HCCONTROLHEADED, 0x0);
  ohci_outl(HCCONTROLCURRENTED, 0x0);
  ohci_outl(HCBULKHEADED, 0x0);
  ohci_outl(HCBULKCURRENTED, 0x0);

  urb->toggle = (ed->head >> 1) & 1;

  addr = *(u32 *)(ohci.hccaptr + 0x84) & ~0xf;

  for (addr = *(u32 *)(ohci.hccaptr + 0x84) & ~0xf; addr; ) {
    td = (ohci_td_t *)get_offs(addr);

#ifdef _TRACE
  if (td->flags >> 28)
    io_printf("ERROR %x\n", (u16)(td->flags >> 28));
#endif

      if (res == ERR_USB_TRANSMIT) {
        switch (td->flags >> 28) {
          case 0x9:
            res = 0;
            break;
          case 0x4:
            res = ERR_USB_STALL;
            break;
        }
      }

    len -= td->cbp ? td->be + 1 - td->cbp : 0;
    addr = td->next;
    ohci_free(td);
  }
  *(u32 *)(ohci.hccaptr + 0x84) = 0;
  ohci_outl(HCINTERRUPTSTATUS, 2);

  for (addr = ed->head & ~0xf; addr; ) {
    td = (ohci_td_t *)get_offs(addr);

    len -= td->be + 1 - td->cbp;
    addr = td->next;
    ohci_free(td);
  }
  ohci_free(ed);

#ifdef _TRACE
  io_printf("RES %d\n", (res < 0) ? res : (pid == pidIN) ? len : 0);
#endif

  if (res < 0)
    return res;

  return (pid == pidIN) ? len : 0;
}


/* Function initializes OHCI driver */
int ohci_init(hcd_t *hcd)
{
  pci_id_t pci_id;
  pci_dev_t pdev;
  u32 k, fminterval, hccontrol;
  u8 *p;

  /* Find first OHCI controller */
  pci_id.vendor = 0x1022;
  pci_id.device = 0x2094;
  pci_id.class = 0x0c0300;

  if (pci_find_device(&pci_id, 0, PCI_FIND_DEVICE, &pdev) == NULL)
    return ERR_USB_UNKDEV;

  io_printf(" OHCI HC found\n");

  ohci.irq = pdev.irq;

  /* Reconfigure OHCI and disable SMM driver */
  ohci.base = 0xd000;
  if (pdev.base != ((u32)ohci.base << 4)) {
    asm cli;
    _ohci_unlimitfs();
    _EBX = pdev.base;

    asm {
      xor ax, ax
      mov fs, ax
      mov eax, fs:[ebx + HCCONTROL]
      mov hccontrol, eax
      test eax, 0x100
      jz init2
      mov dword ptr fs:[ebx + HCINTERRUPTENABLE], 0x40000000
      mov dword ptr fs:[ebx + HCCOMMANDSTATUS], 0x8

    init1:
      test dword ptr fs:[ebx + HCCONTROL], 0x100
      jnz init1
    init2:
      sti
    }

    if (hccontrol & 0x100)
      io_printf(" SMM driver disabled\n");

    pdev.base = (u32)ohci.base << 4;
    pci_confwritel(&pdev, 16, pdev.base);

    io_printf(" base set to 0x%x0\n", ohci.base);
  }

  /* Reset and run OHCI */
  ohci_outl(HCCONTROL, ohci_inl(HCCONTROL) & 0x000L);
  ohci_inl(HCCONTROL);

  /* Reset HC */
  ohci_outl(HCCOMMANDSTATUS, 1L);
  while (ohci_inl(HCCOMMANDSTATUS) & 1L);

  /* Prepare HCCA */
  ohci.hccaptr = get_offs((u32)(get_addr(ohci.hcca) + 255) & ~0xff);
  for (k = 0; k < 252; k++)
    ohci.hccaptr[k] = 0;

  ohci_outl(HCCONTROLHEADED, 0x0);
  ohci_outl(HCCONTROLCURRENTED, 0x0);
  ohci_outl(HCBULKHEADED, 0x0);
  ohci_outl(HCBULKCURRENTED, 0x0);

  ohci_outl(HCHCCA, get_addr(ohci.hccaptr));
  ohci_outl(HCINTERRUPTDISABLE, 0x80000000L);
  ohci_outl(HCINTERRUPTSTATUS, ~0L);

  /* fminterval = (1L << 31) | ((0x7fffL & ((6L * (0x2edfL - 210L)) / 7L)) << 16) | 0x2edfL; */
  fminterval = (ohci_inl(HCFMINTERVAL) & 0x80000000L) ^ 0x80000000L | 0x27782edfL;

  ohci_outl(HCFMINTERVAL, fminterval);
  ohci_outl(HCPERIODICSTART, 0x3e67L);

  if ((ohci_inl(HCFMINTERVAL) & 0x3fff0000L) == 0L) {
    io_printf("HCFMINTERVAL set error %x %x !\n", ohci_inl(HCFMINTERVAL));
    return -1;
  }

  /* Go to operational mode */
  ohci_outl(HCCONTROL, (ohci_inl(HCCONTROL) & 0x600L) | 0x80L | 0x03L);
  ohci_inl(HCCONTROL);

  /* NoPowerSwitching | unmask power on all ports | SetGlobalPower */
  ohci_outl(HCRHDESCRIPTORA, ohci_inl(HCRHDESCRIPTORA) | 0x200L);
  ohci_outl(HCRHDESCRIPTORB, 0);
  ohci_outl(HCRHSTATUS, 0x00010000L);

  ohci_sleep((ohci_inl(HCRHDESCRIPTORA) >> 23) & ~0x1L);

  /* low_isr_get(ohci.irq, &ohci.oldisr[0], &ohci.oldisr[1]);
  low_register_irq(ohci.irq, ohci_interrupt); */

  /* Prepare heap */
  ohci.freeptr = (void *)((u16)(ohci.heap + 0xf) & ~0xf);
  for (p = ohci.freeptr; p < (ohci.heap + sizeof(ohci.heap) - 15); p += 16)
    *(void **)p = (void *)(p + 16);

  hcd->sleep = ohci_sleep;
  hcd->checkhub = ohci_checkhub;
  hcd->transmit = ohci_transmit;
  hcd->done = ohci_done;

  return 0;
}


/* Function shutdowns OHCI driver */
void ohci_done(void)
{
  ohci_outl(HCCONTROL, ohci_inl(HCCONTROL) & 0x000L);
  ohci_inl(HCCONTROL);

  /* Reset HC */
  ohci_outl(HCCOMMANDSTATUS, 1L);
  while (ohci_inl(HCCOMMANDSTATUS) & 1L);
  
  ohci_outl(HCCONTROLHEADED, 0x0);
  ohci_outl(HCCONTROLCURRENTED, 0x0);
  ohci_outl(HCBULKHEADED, 0x0);
  ohci_outl(HCBULKCURRENTED, 0x0);

  ohci_outl(HCHCCA, 0);
  ohci_outl(HCINTERRUPTDISABLE, 0x80000000L);

  /* low_isr_set(ohci.irq, &ohci.oldisr[0], &ohci.oldisr[1]); */
}
