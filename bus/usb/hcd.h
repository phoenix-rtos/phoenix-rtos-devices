/**
 * USB host controller driver declarations
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


#ifndef _USB_HCD_H_
#define _USB_HCD_H_

#include <proc/mutex.h>

#include "hub.h"


typedef struct usb_dev_ usb_dev_t;
typedef struct usb_ep_ usb_ep_t;
typedef struct usb_urb_ usb_urb_t;
typedef struct usb_hcd_ usb_hcd_t;


typedef struct usb_rootHubControl_ {
	void (*getPortStatus)(usb_bus_t* bus,u8 portNum,void* status);
	void (*resetPort)(usb_bus_t* bus,u8 portNum);
	void (*clearConnectChange)(usb_bus_t* bus,u8 portNum);
	void (*updateCtrlEp)(usb_dev_t* dev);
	void (*dropEndpointPriv)(usb_dev_t* dev,u8 num);
} usb_rootHubControl_t;


typedef struct usb_hcd_ {
	int (*irq)(unsigned int n,cpu_context_t* ctx,void* data);
	usb_urbList_t (*scanTransfers)(usb_bus_t* bus);
	int (*urbEnqueue)(usb_bus_t* bus,usb_urb_t* urb);
	int (*urbDequeue)(usb_bus_t* bus,usb_urb_t* urb);
	int (*allocBandwidth)(usb_dev_t* dev,usb_ep_t* endpoint);
	usb_rootHubControl_t ops;
} usb_hcd_t;


#define USB_LOWEST_INVALID_ADDR 128
typedef struct usb_bus_ {
	usb_hub_t 	rootHub;
	usb_hcd_t* 	hcd;
	void* 		hcdCtx; /**< Private data of hcd instance */
	u32 status; /**< Bits:
				* 		0 - transfer complete
				* 		1 - transfer error
				* 		2 - port status change (connect/disconnect) 
				*/
	u32 portChange; 
	spinlock_t lock;
	mutex_t urbLock;
	LIST_ENTRY(usb_bus_) activated;
	u32 addrPool[USB_LOWEST_INVALID_ADDR / 32]; /* 128 available addresses, 0 root hub */
} usb_bus_t;

#define USB_TRANSFER_COMPLETE 0x1
#define USB_TRANSFER_ERROR 0x2
#define USB_PORT_CHANGE 0x4
#define USB_SYSTEM_ERROR 0x8

typedef struct usb_ehciDesc_ {
	u32 baseAddr;
	u16 capRegOffset;
	u16 opRegOffset;
	u16 modeRegOffset;
	u16 regSectionSize;
	u16  periodicScheduleLength;
	u16  irqNum;
} usb_ehciDesc_t;


typedef struct usb_customHcdInit_ {
	usb_dev_t* (*init)(void* data);
	void* data;
} usb_customHcdInit_t;


typedef struct usb_hcDesc_ {
	u16 type;
	u16 portNum;
	union {
		usb_ehciDesc_t ehciDesc;
		usb_customHcdInit_t customHcd;
	};
} usb_hcDesc_t;

#define USB_HCD_OHCI 0
#define USB_HCD_UHCI 1
#define USB_HCD_EHCI 2
#define USB_HCD_XHCI 3
#define USB_HCD_CUSTOM 4

 
typedef struct usb_hcdInitParams_ {
	size_t hcNum;
	usb_hcDesc_t hc[];
} usb_hcdInitParams_t;
	 

extern int usb_insertRootHubs(void);
extern int usb_initBus(usb_bus_t* bus,u8 bcdUSB,usb_cfgDesc_t* cfg,u8 portNum,usb_hcd_t* hcd,void* hcdCtx);

extern void usb_processBusEvents(usb_bus_t* bus);

extern u8 usb_busAllocAddr(usb_bus_t* bus);
extern void usb_busFreeAddr(usb_bus_t* bus,u8 addr);

extern int usb_hcdEnqueueUrb(usb_urb_t* urb);
extern int usb_hcdDequeueUrb(usb_urb_t* urb);
extern int usb_hcdDropEndpoint(usb_dev_t* dev,u8 num);

extern void usb_hcdUpdateCtrlEp(usb_dev_t* dev);


static inline int usb_isRootHub(usb_dev_t* dev)
{
	return &dev->parent->bus->rootHub.dev == dev;
}


#endif
