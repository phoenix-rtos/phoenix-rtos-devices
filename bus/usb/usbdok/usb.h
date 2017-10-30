/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005, 2010
 *
 * USB stack
 */

#ifndef _USB_H_
#define _USB_H_

#include "low.h"


/* Communication timeout */
#define USB_TIMEOUT       2000


#define ERR_USB_HOST       -1
#define ERR_USB_TRANSMIT   -2
#define ERR_USB_UNKDEV     -3
#define ERR_USB_PORT       -4
#define ERR_USB_BUSY       -5
#define ERR_USB_STALL      -6
#define ERR_USB_IDLE       -7
#define ERR_USB_TIMEOUT    -8
#define ERR_USB_MEM        -9

#define ERR_UMASS_IO       -34


enum uhci_pid_t {
  pidSETUP = 0x2d,
  pidIN    = 0x69,
  pidOUT   = 0xe1
};


typedef struct _urb_t {
  u8  dev;
  u8  ep;
  u8  toggle;
  u16 mtu;
} urb_t;


typedef struct _ctlreq_t {
  u8  bmRequestType;
  u8  bRequest;
  u16 wValue;
  u16 wIndex;
  u16 wLength;
} ctlreq_t;


/* Common control requests */
#define GET_STATUS         0
#define CLEAR_FEATURE      1
#define SET_FEATURE        3
#define SET_ADDRESS        5
#define GET_DESCRIPTOR     6
#define SET_DESCRIPTOR     7
#define GET_CONFIGURATION  8
#define SET_CONFIGURATION  9
#define GET_INTERFACE      10
#define SET_INTERFACE      11
#define SYNCH_FRAME        12


/* Descriptors */
#define DESCR_DEVICE                    1
#define DESCR_CONFIGURATION             2
#define DESCR_STRING                    3
#define DESCR_INTERFACE                 4
#define DESCR_ENDPOINT                  5
#define DESCR_DEVICE_QUALIFIER          6
#define DESCR_OTHER_SPEED_CONFIGURATION 7
#define DESCR_INTERFACE_POWER           8


typedef struct _hcd_t {
  void (*sleep)(u16 t);
  int (*checkhub)(unsigned int k);
  int (*transmit)(urb_t *urb, u8 pid, u32 buffer, u16 len);
  void (*done)(void);
} hcd_t;


typedef struct _descr_device_t {
  u8  bLength;
  u8  bDescriptorType;
  u16 bcdUSB;
  u8  bDeviceClass;
  u8  bDeviceSubClass;
  u8  bDeviceProtocol;
  u8  bMaxPacketSize0;
  u16 idVendor;
  u16 idProduct;
  u16 bcdDevice;
  u8  iManufacturer;
  u8  iProduct;
  u8  iSerialNumber;
  u8  bNumConfigurations;
} descr_device_t;


typedef struct _descr_conf_t {
  u8  bLength;
  u8  bDescriptorType;
  u16 wTotalLength;
  u8  bNumInterfaces;
  u8  bConfigurationValue;
  u8  iConfiguration;
  u8  bmAttributes;
  u8  bMaxPower;
} descr_conf_t;


typedef struct _descr_if_t {
  u8 bLength;
  u8 bDescriptorType;
  u8 bInterfaceNumber;
  u8 bAlternateSetting;
  u8 bNumEndpoints;
  u8 bInterfaceClass;
  u8 bInterfaceSubClass;
  u8 bInterfaceProtocol;
  u8 iInterface;
} descr_if_t;


typedef struct _descr_ep_t {
  u8  bLength;
  u8  bDescriptorType;
  u8  bEndpointAddress;
  u8  bmAttributes;
  u16 wMaxPacketSize;
  u8  bInterval;
} descr_ep_t;


/* Function suspends executions for the given number of ms */
extern void usb_sleep(u16 t);


/* Functions sends/receives data over USB */
extern int usb_transmit(urb_t *urb, u8 pid, u32 buffer, u16 len);


extern int usb_bulk_reset(urb_t *urb, urb_t *rurb);


extern int usb_get_status(urb_t *urb, urb_t *rurb, u16 *st);


/* Function checks given hub port */
extern int usb_check_hub(int port);


/* Function configures newly inserted device */
extern int usb_insertion(int port);


/* Function initializes USB stack */
extern int usb_init(void);


/* Function shutdowns USB stack */
extern void usb_done(void);


#endif
