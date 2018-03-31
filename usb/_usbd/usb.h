/**
 * USB stack declarations
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file usb.h
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author: Pawel Tryfon <pawel.tryfon@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _USB_H_
#define _USB_H_

#include <lib/list.h>
#include <lib/stdint.h>
#include "usb_spec.h"


#ifdef USB_DEBUG
#define BREAK __asm__ volatile ("BKPT")
#else
#define BREAK do{}while()
#endif

/* USB errors */
#define USB_ERR_INIT 1
#define USB_ERR_TIMEOUT 2
#define USB_ERR_NYI 3
#define USB_ERR_NODRV 4


/* USB status */
#define USB_DISCONNECTED 0
#define USB_CONNECTED 0

typedef LIST_HEAD(usb_driver_) usb_driverList_t;
typedef LIST_HEAD(usb_urb_) usb_urbList_t;
typedef struct usb_hub_ usb_hub_t;
typedef struct usb_driver_ usb_driver_t;


typedef struct usb_ep_ {
	usb_epDesc_t* desc;
	usb_urbList_t urbs;
	void* hcpriv;
} usb_ep_t;


#define USB_MAX_EP_LEN 8
/**
 * USB device structure
 */
typedef struct usb_dev_ {
	signed char addr; /**< Device address */
	u8 state; 
	signed char speed;
	signed char port;
	usb_hub_t* parent;
	usb_devDesc_t desc;
	usb_cfgDesc_t* cfg;
	usb_ep_t* ep[USB_MAX_EP_LEN];
	LIST_ENTRY(usb_dev_) devices;
	usb_driver_t* drv;
} usb_dev_t;


#define USB_INVALID -1

#define USB_FULL_SPEED 0
#define USB_LOW_SPEED 1
#define USB_HIGH_SPEED 2


/**
 * USB driver structure 
 * It declares functions that each driver must implement
 */
typedef struct usb_driver_ {
	int (*probe)(usb_dev_t*); /**< probe() is called to see if driver would like to manage given device */
	void (*disconnect)(usb_dev_t*); /**< disconnect() is called to notify the driver that device was detached */
	LIST_ENTRY(usb_driver_) drivers;
} usb_driver_t;


/**
 * URB - USB Request Block
 * Structure specifying all information required for USB transfer.
 */
typedef struct usb_urb_ usb_urb_t;
struct usb_urb_ {
	/* Transfer spec fields */
	usb_dev_t* dev;
	u32 pipe; /* Transfer direction, type and endpoint number */
	void* data; /**< Data to transfer */
	void* setup; /**< Setup packet for control transfers only, ignored otherwise */
	u16 dataLen;
	u16 setupLen;
	u32 interval; /**< Polling interval for interrupt transfers */

	/* Completion fields */
	volatile u16 status;
	u16 transferred; /**< Actual number of bytes transferred */
	void* compData; /**< Data pointer that might be used by completion callback */ 
	void (*callback)(usb_urb_t*);

	/* Private fields */
	LIST_ENTRY(usb_urb_) urbs; /**< urb owner specific */
	void* hcpriv;
};


#define USB_URB_DONE 0x01
#define USB_URB_IN_COMPLETION 0x2
#define USB_URB_COMPLETED 0x4
#define USB_URB_ERROR 0x08
#define USB_URB_ERRNO_SHIFT 4


extern void _usb_init(void);

/* Called by drivers-initialization code, after _usb_init() is done */
void usb_addDriver(usb_driver_t* drv);

/* Called when device change detected */
extern int usb_connect(usb_dev_t*);
extern int usb_disconnect(usb_dev_t*);

typedef struct usb_bus_ usb_bus_t;
extern void usb_triggerBus(usb_bus_t* bus);

/* Called by drivers for interrupt or isochronous endpoints that require bandwidth guarantee.
 * Bandwidth parameters (data size/interval) are given by endpoint descriptor */
extern int usb_allocBandwidth(usb_dev_t* dev,usb_ep_t* ep);
extern int usb_freeBandwidth(usb_dev_t* dev,usb_ep_t* ep);


/* Synchronous transfer */
extern int usb_transfer(usb_urb_t* urb,u32 timeout);
/* Asynchronous transfer */
extern int usb_submitUrb(usb_urb_t* urb);
extern void usb_killUrb(usb_urb_t* urb);

/* Helper functions for synchronous transfers */
extern int usb_controlTransfer(usb_dev_t* dev,u8 epAddr,usb_ctrlreq_t* req,u8* data,u16 size,u32 timeout);
extern int usb_bulkTransfer(usb_dev_t* dev,u8 epAddr,u8 dir,u8* data,u16 size,u32 timeout);
extern int usb_intrTransfer(usb_dev_t* dev,u8 epAddr,u8 dir,u8* data,u16 size,u32 timeout);

extern int usb_addEndpoint(usb_dev_t* dev, usb_epDesc_t* ep);
extern int usb_dropEndpoint(usb_dev_t* dev,usb_epDesc_t* ep);

extern int usb_setAddress(usb_dev_t* dev,u8 addr);
extern int usb_getDescriptor(usb_dev_t* dev,u8 type,u8 index,u8* buf,u16 size);
extern int usb_setConfiguration(usb_dev_t* dev,u8 cfg);
extern int usb_getConfiguration(usb_dev_t* dev,u8* cfg);
extern int usb_getStatus(usb_dev_t*,u8 type,u16* status);
extern int usb_resetEndpoint(usb_dev_t*,u8 num);


extern int usb_completeUrb(usb_urb_t* urb);

extern usb_dev_t* usb_getByPath(signed char* usbPath);
#endif
