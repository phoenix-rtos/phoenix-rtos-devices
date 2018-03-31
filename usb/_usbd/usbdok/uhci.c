/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005, 2010
 *
 * UHCI driver implementation
 */

#include "low.h"
#include "io.h"
#include "pci.h"
#include "usb.h"
#include "uhci.h"


#define TDS  1024

/* Detection automa delays */
#define DELAY_DETECT   1
#define DELAY_RESET    50
#define DELAY_ENABLE   1
#define DELAY_CONFIG   1


static struct {
  u16          base;
  u32          *fl;
  uhci_td_t    *td;
  uhci_qh_t    *qh;
  volatile int compl;
  u8           heap[8192 + (TDS + 1) * sizeof(uhci_td_t)];
} uhci;


/* Function suspend executions for t miliseconds */
void uhci_sleep(u16 t)
{
  u16 ct;

  for (;t;t -= (t < 1024 ? t : 1024))
    for (ct = (inw(uhci.base + FRNUM) & 0x7ff);
         ((inw(uhci.base + FRNUM) - ct) & 0x7ff) < (t < 1024 ? t : 1024););
  return;
}


/* Function checks given hub port */
int uhci_checkhub(unsigned int k)
{
  u16 offs[2] = { PORTSC1, PORTSC2 };
  volatile u16 portsc;
  int res;

  portsc = inw(uhci.base + offs[k]);

#ifdef _TRACE
  io_printf("uhci_check_hub: test portsc=%x\n", portsc);
#endif

  /* Check port */
  if (!(portsc & 1))
    return 0;

  /* Check if status changed */
  if (!(portsc & 2))
    return 1;
  
  uhci_sleep(DELAY_DETECT);
  if (!(portsc & 1))
    return 0;

  /* start port reset */
  outw(uhci.base + offs[k], 0x0200);

  /* port reset done */
  uhci_sleep(DELAY_RESET);
  outw(uhci.base + offs[k], 0x0000);

  /* port enable */
  uhci_sleep(DELAY_ENABLE);
  outw(uhci.base + offs[k], 0x0004);

  /* device configure */
  uhci_sleep(DELAY_CONFIG);

  if (usb_insertion(k) < 0) {
    outw(uhci.base + offs[k], 0x0000);
    return 0;
  }
  outw(uhci.base + offs[k], 0x0006);

  return 0;
}


void uhci_interrupt(void)
{
  u16 status;
  u32 k;

  status = inw(uhci.base + USBSTS);

  /* UHCI halted */
  if (status & 0x20) {
    uhci.qh->td = tdTerm;
    outw(uhci.base + USBCMD, 0x81);
    uhci.compl = ERR_USB_TRANSMIT;
  }

  /* Transmission error occured */
  else if (status & 2) {
    uhci.qh->td = tdTerm;
    uhci.compl = ERR_USB_TRANSMIT;

    /* Check STALL condition */
    for (k = 1;;k++) {
      if (uhci.td[k].f0 & tdStall) {
        uhci.compl = ERR_USB_STALL;
        break;
      }
      if (uhci.td[k].link == 1)
        break;
    }
  }

  /* Interrupt on complete */
  else if (status & 1) {

    /* Test if transmission complete */
    if (!uhci.compl) {
      for (k = 1;;k++) {
        if (uhci.td[k].f0 & tdActive) {
          break;
        }

        if ((((uhci.td[k].f0 & 0x7ff) != (uhci.td[k].f1 >> 21)) &&
             ((uhci.td[k].f1 & 0xff) == pidIN)) || (uhci.td[k].link == 1)) {
          uhci.qh->td = tdTerm;
          uhci.compl = 1;
          break;
        }
      }
    }
  }

#ifdef _DEBUG
  else {
    io_printf("FATAL: Uknown port state %x!\n", status);
  }
#endif

  outw(uhci.base + USBSTS, status & 0x3f);
  low_intack();
  return;
}


/* Functions transmits data over USB */
int uhci_transmit(urb_t *urb, u8 pid, u32 buffer, u16 len)
{
  u32 k;
  u16 u, st[2], t;
  int res;

  uhci.compl = 0;

  /* Prepare trasmission descriptor chain */
  u = urb->ep ? 64 : urb->mtu;
  urb->dev &= 0x7f;
  urb->ep &= 0x0f;
  urb->toggle &= 1;

  for (k = 1;; k++) {
    uhci.td[k].f0 = (tdC_ERR * (u32)3) | tdShort | tdActive | (u32)(((len > u ? u : len) - 1) & 0x7ff);
    uhci.td[k].f1 = (tdEndPt * (u32)urb->ep) | (tdDevice * (u32)urb->dev) | pid |
                    (tdMaxLen * (u32)(((len > u ? u : len) - 1) & 0x7ff)) | (tdToggle * (u32)urb->toggle);
    uhci.td[k].buffer = buffer;
    urb->toggle = (1 - urb->toggle);
    buffer += u;
    len -= (len > u ? u : len);
    if (k > 1)
      uhci.td[k - 1].link = get_addr(&uhci.td[k]) | tdDepth;
    if (!len)
      break;
  }
  uhci.td[k].link = tdTerm;
  uhci.td[k].f0 |= tdIOC;

  /* Launch transfer and wait for completion */
  uhci.qh->td = get_addr(&uhci.td[1]);

  t = 0;
  st[0] = inw(uhci.base + FRNUM);
  for (;;) {
    st[1] = st[0];
    st[0] = inw(uhci.base + FRNUM);
    t += ((st[0] - st[1]) & 0x7ff);

    if (t > USB_TIMEOUT) {
      uhci.compl = ERR_USB_TIMEOUT;
      break;
    }

    if (uhci.compl)
      break;
  }

  if (uhci.compl < 0)  
    return uhci.compl;

  /* Calculate received data length */
  u = 0;
  if (pid == pidIN) {
    for (k = 1;;k++) {
      u += ((uhci.td[k].f0 + 1) & 0x7ff);
      if ((uhci.td[k].link == 1) ||
          ((uhci.td[k].f0 & 0x7ff) != (uhci.td[k].f1 >> 21)))
        break;
    }
  }

  return u;
}


/* Function prepares UHCI data */
int uhci_prepare_data(void)
{
  u32 fladdr;
  u32 k;

  /* Prepare an empty frame list */
  uhci.fl = (u32 *)uhci.heap;
  fladdr = get_addr(uhci.fl);
  if (fladdr & 0xfff) {
    uhci.fl = (u32 *)((u16)uhci.fl + (0x1000 - (fladdr & 0xfff)));
    fladdr &= ~(0xfff);
    fladdr += 0x1000;
  }
  uhci.qh = (uhci_qh_t *)((u16)uhci.fl + 0x1000);
  uhci.td = (uhci_td_t *)((u16)uhci.qh + sizeof(uhci_td_t));

  for (k = 0; k < 1024; k++)
    uhci.fl[k] = get_addr(uhci.qh) | tdQH;

  /* Prepare empty td for hub checking */
  uhci.td->link = get_addr(uhci.qh) | tdQH;
  uhci.td->f0 = tdIOC;
  uhci.td->f1 = 0;
  uhci.td->buffer = NULL;

  uhci.qh->link = tdTerm;
  uhci.qh->td = tdTerm;

  return 0;
}


/* Function initializes UHCI driver */
int uhci_init(hcd_t *hcd)
{
  pci_id_t pci_id;
  pci_dev_t pdev;
  u32 k;

  /* Find first UHCI controller */
  pci_id.vendor = PCI_VENDOR_ID_INTEL;
  pci_id.device = 0x2444;
  pci_id.class = 0x0c0300;  
  if (pci_find_device(&pci_id, 1, PCI_FIND_CLASS, &pdev) == NULL)
    return -1;

  io_printf(" UHCI HC found\n");

  uhci.base = (u16)pdev.base;
  uhci_prepare_data();

  low_register_irq(pdev.irq, uhci_interrupt);

  /* Reset UHCI */
  outw(uhci.base + USBCMD, 0x04);
  for (k = 0; k < 0xffffff; k++);
  outw(uhci.base + USBCMD, 0x00);

  /* Start UHCI */
  outl(uhci.base + FRBASEADD, get_addr(uhci.fl));
  outw(uhci.base + USBINTR, 0x08 | 0x04 | 0x01);
  outw(uhci.base + USBCMD, 0x81);

  hcd->sleep = uhci_sleep;
  hcd->checkhub = uhci_checkhub;
  hcd->transmit = uhci_transmit;
  hcd->done = uhci_done;

  uhci_sleep(20);
  return 0;
}


/* Function shutdowns UHCI driver */
void uhci_done(void)
{
  outw(uhci.base + USBCMD, 0);
  for (;;) {
    if (inw(uhci.base + USBSTS) & 0x20)
      break;
  }
  outw(uhci.base + PORTSC1, 0);
  outw(uhci.base + PORTSC2, 0);
}
