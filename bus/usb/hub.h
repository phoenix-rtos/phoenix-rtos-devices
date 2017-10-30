#ifndef _USB_HUB_H_
#define _USB_HUB_H_

#include "usb.h"

#define USB_MAX_PORTS 31
#define USB_HUB_DATA_LEN 4

typedef struct usb_bus_ usb_bus_t;

typedef struct usb_hub_ {
	usb_dev_t dev;
	usb_bus_t* bus;
	usb_hubDesc_t desc;
	usb_urb_t intrUrb;
	char intrData[USB_HUB_DATA_LEN];
	usb_dev_t** children;
	u32 portChange;
} usb_hub_t;

typedef struct {
	u16 port;
	u16 change;
} PACKED usb_hubPortStatus_t;


#if USB_MAX_PORTS > (USB_HUB_DATA_LEN * 8) - 1
#error intrData too short.
#endif


int usb_hubInit(usb_driver_t** drv);
int usb_isHub(usb_dev_t* dev);
usb_dev_t* usb_getRootHub(int i);

static inline void _usb_hubUrbSetPortChange(usb_hub_t* hub,u32 portChange)
{
	/* First bit is hub change field */
	*((u32*)hub->intrData) |= portChange; 
	hub->intrUrb.transferred = USB_HUB_DATA_LEN;
	hub->intrUrb.status |= USB_URB_DONE;
}




#endif
