/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2010
 *
 * OHCI definitions
 */

#ifndef _OHCI_H_
#define _OHCI_H_

#include "low.h"
#include "usb.h"


typedef struct _ohci_ed_t {
  volatile u32 flags;
  volatile u32 tail;
  volatile u32 head;
  volatile u32 next;
} ohci_ed_t;


typedef struct _ohci_td_t {
  volatile u32 flags;
  volatile u32 cbp;
  volatile u32 next;
  volatile u32 be;
} ohci_td_t;


/* Host controller registers */
enum ohci_registers_t {
  HCREVISION = 0x0,
  HCCONTROL = 0x4,
  HCCOMMANDSTATUS = 0x8,
  HCINTERRUPTSTATUS = 0xc,
  HCINTERRUPTENABLE = 0x10,
  HCINTERRUPTDISABLE = 0x14,

  HCHCCA = 0x18,
  HCPERIODCURRENTED = 0x1c,
  HCCONTROLHEADED = 0x20,
  HCCONTROLCURRENTED = 0x24,
  HCBULKHEADED = 0x28,
  HCBULKCURRENTED = 0x2c,
  HCDONEHEAD = 0x30,

  HCFMINTERVAL = 0x34,
  HCFMREMAINING = 0x38,
  HCFMNUMBER = 0x3c,
  HCPERIODICSTART = 0x40,
  HCLSTHRESHOLD = 0x44,

  HCRHDESCRIPTORA = 0x48,
  HCRHDESCRIPTORB = 0x4c,
  HCRHSTATUS = 0x50,
  HCRHPORTSTATUS = 0x54
};


/* Function suspend executions for t miliseconds */
extern void ohci_sleep(u16 t);


/* Function checks given hub port */
extern int ohci_checkhub(unsigned int k);


/* Function transmits data over USB */
extern int ohci_transmit(urb_t *urb, u8 pid, u32 buffer, u16 len);


/* Function initializes OHCI driver */
extern int ohci_init(hcd_t *hcd);


/* Function stops OHCI driver */
extern void ohci_done(void);


#endif
