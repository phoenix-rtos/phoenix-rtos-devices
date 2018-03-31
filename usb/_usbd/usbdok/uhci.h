/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005, 2010
 *
 * UHCI definitions
 */

#ifndef _UHCI_H_
#define _UHCI_H_

#include "low.h"
#include "usb.h"


typedef struct _uhci_td_t {
  volatile u32 link;
  volatile u32 f0;
  volatile u32 f1;
  volatile u32 buffer;
} uhci_td_t;


typedef struct _uhci_qh_t {
  volatile u32 link;
  volatile u32 td;
} uhci_qh_t;


/* Host controller registers */
enum uhci_registers_t {
  USBCMD    = 0x00,
  USBSTS    = 0x02,
  USBINTR   = 0x04,
  FRNUM     = 0x06,
  FRBASEADD = 0x08,
  SOFMOD    = 0x0c,
  PORTSC1   = 0x10,
  PORTSC2   = 0x12
};


/* Transaction descriptor flags */
#define tdDepth    4
#define tdQH       2
#define tdTerm     1
#define tdShort    0x20000000
#define tdC_ERR    0x08000000
#define tdLS       0x04000000
#define tdIOS      0x02000000
#define tdIOC      0x01000000
#define tdActive   0x00800000
#define tdStall    0x00400000
#define tdBuffErr  0x00200000
#define tdBabble   0x00100000
#define tdNAK      0x00080000
#define tdCRCErr   0x00040000
#define tdBitStuff 0x00020000
#define tdMaxLen   0x00200000
#define tdToggle   0x00080000
#define tdEndPt    0x00008000
#define tdDevice   0x00000100


/* Function suspend executions for t miliseconds */
extern void uhci_sleep(u16 t);


/* Function checks given hub port */
extern int uhci_check_hub(unsigned int k);


/* Function transmits data over USB */
extern int uhci_transmit(urb_t *urb, u8 pid, u32 buffer, u16 len);


/* Function initializes UHCI driver */
extern int uhci_init(hcd_t *hcd);


/* Function stops UHCI driver */
extern void uhci_done(void);


#endif
