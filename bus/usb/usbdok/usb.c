/*
 * USB Disk On Key driver for MS-DOS
 * (c) IMMOS, 2003, 2005
 *
 * USB stack implementation
 */

#include "io.h"
#include "uhci.h"
#include "ohci.h"
#include "usb.h"
#include "umass.h"


struct {
  urb_t urb[2][3];
  descr_device_t descr_device;
  char descr[sizeof(descr_conf_t) + sizeof(descr_if_t) + sizeof(descr_ep_t) * 15];
  ctlreq_t req;
  hcd_t hcd;
} usb;


void print_descr_device(descr_device_t *descr)
{
  io_printf("bLength=%d\n", descr->bLength);
  io_printf("bDescriptorType=%d\n", descr->bDescriptorType);
  io_printf("bcdUSB=0x%x\n", descr->bcdUSB);
  io_printf("bDeviceClass=%d\n", descr->bDeviceClass);
  io_printf("bDeviceSubClass=%d\n", descr->bDeviceSubClass);
  io_printf("bDeviceProtocol=%d\n", descr->bDeviceProtocol);
  io_printf("bMaxPacketSize0=%d\n", descr->bMaxPacketSize0);
  io_printf("idVendor=%d\n", descr->idVendor);
  io_printf("idProduct=%d\n", descr->idProduct);
  io_printf("bcdDevice=0x%x\n", descr->bcdDevice);
  io_printf("iManufacturer=%d\n", descr->iManufacturer);
  io_printf("iProduct=%d\n", descr->iProduct);
  io_printf("iSerialNumber=%d\n", descr->iSerialNumber);
  io_printf("bNumConfigurations=%d\n", descr->bNumConfigurations);
}


void print_descr_conf(descr_conf_t *descr)
{
  io_printf(" bLength=%d\n", descr->bLength);
  io_printf(" bDescriptorType=%d\n", descr->bDescriptorType);
  io_printf(" wTotalLength=%d\n", descr->wTotalLength);
  io_printf(" bNumInterfaces=%d\n", descr->bNumInterfaces);
  io_printf(" bConfigurationValue=%d\n", descr->bConfigurationValue);
  io_printf(" iConfiguration=%d\n", descr->iConfiguration);
  io_printf(" bmAttributes=%d\n", descr->bmAttributes);
  io_printf(" bMaxPower=%d\n", descr->bMaxPower);
}


void print_descr_if(descr_if_t *descr)
{
  io_printf("  bLength=%d\n", descr->bLength);
  io_printf("  bDescriptorType=%d\n", descr->bDescriptorType);
  io_printf("  bInterfaceNumber=%d\n", descr->bInterfaceNumber);
  io_printf("  bAlternateSetting=%d\n", descr->bAlternateSetting);
  io_printf("  bNumEndpoints=%d\n", descr->bNumEndpoints);
  io_printf("  bInterfaceClass=%d\n", descr->bInterfaceClass);
  io_printf("  bInterfaceSubClass=%d\n", descr->bInterfaceSubClass);
  io_printf("  bInterfaceProtocol=%d\n", descr->bInterfaceProtocol);
  io_printf("  iInterface=%d\n", descr->iInterface);
}


void print_descr_ep(descr_ep_t *descr)
{
  io_printf("   bLength=%d\n", descr->bLength);
  io_printf("   bDescriptorType=%d\n", descr->bDescriptorType);
  io_printf("   bEndpointAddress=%d\n", descr->bEndpointAddress);
  io_printf("   bmAttributes=%d\n", descr->bmAttributes);
  io_printf("   wMaxPacketSize=%d\n", descr->wMaxPacketSize);
  io_printf("   bInterval=%d\n", descr->bInterval);
}


/* Function suspends executions for the given number of ms */
void usb_sleep(u16 t)
{
  usb.hcd.sleep(t);
}


/* Functions sends/receives data over USB */
int usb_transmit(urb_t *urb, u8 pid, u32 buffer, u16 len)
{
  return usb.hcd.transmit(urb, pid, buffer, len);
}


int set_address(urb_t *urb, u8 addr)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0;
  usb.req.bRequest = SET_ADDRESS;
  usb.req.wValue = addr;
  usb.req.wIndex = 0;
  usb.req.wLength = 0;

  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;

  if ((res = usb_transmit(urb, pidIN, NULL, 0)) < 0)
    return res;

  urb->dev = addr;
  return 0;
}


int set_configuration(urb_t *urb, u8 c)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0x00;
  usb.req.bRequest = SET_CONFIGURATION;
  usb.req.wValue = c;
  usb.req.wIndex = 0;
  usb.req.wLength = 0;
  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, NULL, 0)) < 0)
    return res;
  return 0;
}


int get_configuration(urb_t *urb, u8 *c)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0x80;
  usb.req.bRequest = GET_CONFIGURATION;
  usb.req.wValue = 0;
  usb.req.wIndex = 0;
  usb.req.wLength = 1;
  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, get_addr((u8 *)c), 1)) < 0)
    return res;
  urb->toggle = 1;
  if ((res = usb_transmit(urb, pidOUT, NULL, 0)) < 0)
    return res;
  return 0;
}


int set_interface(urb_t *urb, u8 i)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0x01;
  usb.req.bRequest = SET_INTERFACE;
  usb.req.wValue = 0;
  usb.req.wIndex = i;
  usb.req.wLength = 0;
  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, NULL, 0)) < 0)
    return res;
  return 0;
}


int get_interface(urb_t *urb, u8 *i)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0x81;
  usb.req.bRequest = GET_INTERFACE;
  usb.req.wValue = 0;
  usb.req.wIndex = 0;
  usb.req.wLength = 1;
  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, get_addr((u8 *)i), 1)) < 0)
    return res;
  urb->toggle = 1;
  if ((res = usb_transmit(urb, pidOUT, NULL, 0)) < 0)
    return res;
  return 0;
}


int get_descriptor(urb_t *urb, u8 type, void *dp, u16 size)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0x80;
  usb.req.bRequest = GET_DESCRIPTOR;
  usb.req.wValue = (type << 8);
  usb.req.wIndex = 0;
  usb.req.wLength = size;

  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, get_addr((u8 *)dp), size)) < 0)
    return res;

  urb->toggle = 1;
  if ((res = usb_transmit(urb, pidOUT, NULL, 0)) < 0)
    return res;

  return 0;
}


int usb_bulk_reset(urb_t *urb, urb_t *rurb)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0x02;
  usb.req.bRequest = CLEAR_FEATURE;
  usb.req.wValue = 0;
  usb.req.wIndex = rurb->ep;
  usb.req.wLength = 0;

  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, NULL, 0)) < 0)
    return res;
  return 0;
}


int usb_get_status(urb_t *urb, urb_t *rurb, u16 *st)
{
  int res;

  urb->toggle = 0;
  usb.req.bmRequestType = 0x82;
  usb.req.bRequest = GET_STATUS;
  usb.req.wValue = 0;
  usb.req.wIndex = rurb->ep;
  usb.req.wLength = 2;

  if ((res = usb_transmit(urb, pidSETUP, get_addr((u8 *)&usb.req), sizeof(ctlreq_t))) < 0)
    return res;
  if ((res = usb_transmit(urb, pidIN, get_addr(st), 2)) < 0)
    return res;
  urb->toggle = 1;
  if ((res = usb_transmit(urb, pidOUT, NULL, 0)) < 0)
    return res;
  
  return 0;
}


/* Function checks given hub port */
int usb_check_hub(int port)
{
  return usb.hcd.checkhub(port);
}


/* Function configures newly inserted device */
int usb_insertion(int port)
{
  descr_ep_t *descr_ep;
  int res;
  int k;
  char b;

  if (port & ~1)
    return ERR_USB_PORT;

  /* Get attached device parameters */
  usb.urb[port][0].dev = 0;
  usb.urb[port][0].ep = 0;
  usb.urb[port][0].mtu = 8;

  /* Configure device */
  if ((res = set_address(&usb.urb[port][0], port ? 12 : 19)) < 0)
    return res;

  /* Get smallest descriptor */
  if ((res = get_descriptor(&usb.urb[port][0], DESCR_DEVICE, (void *)&usb.descr_device, 8)) < 0)
    return res;

  usb.urb[port][0].mtu = usb.descr_device.bMaxPacketSize0;

  if ((res = get_descriptor(&usb.urb[port][0], DESCR_DEVICE, (void *)&usb.descr_device, sizeof(descr_device_t))) < 0)
    return res;

  if ((res = get_descriptor(&usb.urb[port][0], DESCR_CONFIGURATION, (void *)usb.descr, sizeof(usb.descr))) < 0)
    return res;

#ifdef _DEBUG
  print_descr_device(&usb.descr_device);
  print_descr_conf((descr_conf_t *)usb.descr);
  print_descr_if((descr_if_t *)(usb.descr + sizeof(descr_conf_t)));
#endif

  /* Check device class, subclass and protocol */
  if (((usb.descr_device.bDeviceClass != 0) &&
      ((usb.descr_device.bDeviceClass != 8) ||
       (usb.descr_device.bDeviceSubClass != 6) ||
       (usb.descr_device.bDeviceProtocol != 80))) ||
      (((descr_if_t *)(usb.descr + sizeof(descr_conf_t)))->bInterfaceClass != 8) ||
      (((descr_if_t *)(usb.descr + sizeof(descr_conf_t)))->bInterfaceSubClass != 6) ||
      (((descr_if_t *)(usb.descr + sizeof(descr_conf_t)))->bInterfaceProtocol != 80))
    return ERR_USB_UNKDEV;
  
  if ((res = set_configuration(&usb.urb[port][0], ((descr_conf_t *)usb.descr)->bConfigurationValue)) < 0) {
    usb.urb[port][0].dev = 0;
    return res;
  }

  /* Get device endpoints */
  usb.urb[port][1].dev = usb.urb[port][0].dev;
  usb.urb[port][2].dev = usb.urb[port][0].dev;
  usb.urb[port][1].ep = 0;
  usb.urb[port][2].ep = 0;
  usb.urb[port][1].toggle = 0;
  usb.urb[port][2].toggle = 0;

  for (k = 0; k < ((descr_if_t *)(usb.descr + sizeof(descr_conf_t)))->bNumEndpoints; k++) {
    descr_ep = (descr_ep_t *)(usb.descr + sizeof(descr_conf_t) + sizeof(descr_if_t) + k * sizeof(descr_ep_t));
    if(descr_ep->bmAttributes == 2)
      if(descr_ep->bEndpointAddress & 0x80) {
        usb.urb[port][1].ep = descr_ep->bEndpointAddress & 0x0f;
        usb.urb[port][1].mtu = descr_ep->wMaxPacketSize;
      }
      else {
        usb.urb[port][2].ep = descr_ep->bEndpointAddress & 0x0f;
        usb.urb[port][2].mtu = descr_ep->wMaxPacketSize;
      }
  }
  if((usb.urb[port][1].ep == 0) || (usb.urb[port][2].ep == 0)) {
    usb.urb[port][0].dev = 0;
    return ERR_USB_UNKDEV;
  }

  return umass_insertion(port, &usb.urb[port][0]);
}


/* Function initializes USB stack (MOD) */
int usb_init(void)
{
  if (!uhci_init(&usb.hcd))
    return 0;
  if (!ohci_init(&usb.hcd))
    return 0;

  return ERR_USB_HOST;
}


/* Function shutdowns USB stack */
void usb_done(void)
{
  usb.hcd.done();
}
