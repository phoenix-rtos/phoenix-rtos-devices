/**
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @brief USB host controller driver common functions
 * @copyright 2014 Phoenix Systems
 * @author: Pawel Tryfon <pawel.tryfon@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <string.h>
#include <include/errno.h>
#include <vm/if.h>
#include <main/if.h>

#include "urb.h"
#include "usb.h"
#include "host/ehci.h"
#include "hcd.h"


static usb_hcdInitParams_t usb_noHcdParams = {
	.hcNum = 0,
	.hc = {},
};


/* Configuration specific function */
usb_hcdInitParams_t* __attribute__ ((weak)) bsp_getHcdInitParams(void)
{
	return &usb_noHcdParams;
}


int usb_insertRootHubs(void)
{
	int i;
	usb_hcdInitParams_t* initParams = bsp_getHcdInitParams();
	for(i=0;i<initParams->hcNum;i++){
		usb_dev_t* hdev;
		switch (initParams->hc[i].type) {
			case USB_HCD_EHCI:
				hdev=usb_ehciInitHost(&initParams->hc[i]);
				break;
			case USB_HCD_CUSTOM:
				hdev=initParams->hc[i].customHcd.init(initParams->hc[i].customHcd.data);
				break;
			default:
				return -EINVAL;
		}
		if(hdev == NULL)
			return -EFAULT;
		else {		
			usb_connect(hdev);
		}
	}
	return 0;
}


void usb_busScanTransfers(usb_bus_t* bus)
{
	int i;
	usb_dev_t* dev = &bus->rootHub.dev;
	usb_urbList_t tmp;
	usb_urbList_t busDoneUrbs = LIST_HEAD_INITIALIZER;
	usb_urb_t* firstUrb;

	proc_mutexLock(&bus->urbLock);
	for(i = 0;i < USB_MAX_EP_LEN;i++){
		if(dev->ep[i] == NULL || LIST_IS_EMPTY(&dev->ep[i]->urbs))
			continue;
		firstUrb = dev->ep[i]->urbs.first;
		if (firstUrb->status & USB_URB_DONE) {
			if(firstUrb->urbs.prev->status & USB_URB_DONE){ /* All urbs for this endpoint done */
				do {
					firstUrb->status |= USB_URB_IN_COMPLETION;
					firstUrb = firstUrb->urbs.next;
				} while (firstUrb != dev->ep[i]->urbs.first);
				dev->ep[i]->urbs.first = NULL;
			} else { /* Find first not done */
				usb_urb_t* firstUndone = firstUrb->urbs.next;
				usb_urb_t* lastDone;
				while(firstUndone->status & USB_URB_DONE) {
					firstUndone->status |= USB_URB_IN_COMPLETION;
					firstUndone = firstUndone->urbs.next;
				}
				lastDone = firstUndone->urbs.prev;
				firstUndone->urbs.prev = firstUrb->urbs.prev;
				firstUrb->urbs.prev = lastDone;
				dev->ep[i]->urbs.first = firstUndone;
			}
			tmp.first = firstUrb;
			LIST_MERGE(&busDoneUrbs,&tmp,urbs);
		}
	}
	tmp = bus->hcd->scanTransfers(bus);
	LIST_MERGE(&busDoneUrbs,&tmp,urbs);
	/* After finding and unlinking all urbs, call completion functions */
	proc_mutexUnlock(&bus->urbLock);
	if(!LIST_IS_EMPTY(&busDoneUrbs)){
		usb_urb_t* urb = busDoneUrbs.first;
		usb_urb_t* next;
		do {
			next = urb->urbs.next; /* Needed because callback gives back the urb to its submitter */
			urb->callback(urb);
			urb->status |= USB_URB_COMPLETED;
			urb = next;
		} while (urb != busDoneUrbs.first);
	}
}


void usb_processBusEvents(usb_bus_t* bus)
{
	u8 ifSystemError = 0;
	proc_spinlockSet(&bus->lock);	
	while(bus->status != 0) {
		if(bus->status & USB_SYSTEM_ERROR) {
			ifSystemError = 1;
		}
		if(bus->status & USB_PORT_CHANGE){
			bus->status &= ~USB_PORT_CHANGE;
			if(bus->portChange){
				_usb_hubUrbSetPortChange(&bus->rootHub,bus->portChange);
				bus->portChange = 0;
				bus->status |= USB_TRANSFER_COMPLETE;
			}
		}
		if(bus->status & USB_TRANSFER_COMPLETE) {
			bus->status &= ~USB_TRANSFER_COMPLETE;
			proc_spinlockClear(&bus->lock,0);	
			usb_busScanTransfers(bus);
			proc_spinlockSet(&bus->lock);	
		}
	}
	proc_spinlockClear(&bus->lock,0);	
	if(ifSystemError)
		main_printf(ATTR_ERROR,"usb: system error\n");
}


int usb_initBus(usb_bus_t* bus,u8 bcdUSB,usb_cfgDesc_t* cfg,u8 portNum,usb_hcd_t* hcd,void* hcdCtx)
{
	int i;
	bus->rootHub.dev.addr = USB_INVALID;
	bus->rootHub.dev.speed = USB_INVALID;
	bus->rootHub.dev.port = USB_INVALID;
	bus->rootHub.dev.parent = &bus->rootHub;
	bus->rootHub.dev.desc.bLength = sizeof(usb_devDesc_t);
	bus->rootHub.dev.desc.bDescriptorType = USB_DESC_DEV;
	bus->rootHub.dev.desc.bcdUSB = bcdUSB;
	bus->rootHub.dev.desc.bDeviceClass = USB_CLASS_HUB;

	bus->rootHub.dev.desc.bMaxPacketSize0 = 64;

	bus->rootHub.dev.desc.bNumConfigurations = 1;
	bus->rootHub.dev.cfg = cfg;
	for(i=0;i < USB_MAX_EP_LEN;i++)
		bus->rootHub.dev.ep[i] = NULL;
	usb_addEndpoint(&bus->rootHub.dev,NULL);

	bus->rootHub.bus = bus;

	bus->rootHub.children = vm_kmalloc(portNum * sizeof(bus->rootHub.children[0]));
	if(bus->rootHub.children == NULL)
		return -ENOMEM;
	for(i = 0;i < USB_MAX_PORTS;i++)
		bus->rootHub.children[i] = NULL;
	memset(bus->rootHub.intrData,0,USB_HUB_DATA_LEN);
	bus->rootHub.portChange = 0;
	bus->rootHub.desc.bDescLength = sizeof(bus->rootHub.desc);
	bus->rootHub.desc.bDescriptorType = USB_DESC_HUB;
	bus->rootHub.desc.bNbrPorts = portNum;
	bus->rootHub.desc.DeviceRemovable = 0;

	bus->status = 0;
	bus->portChange = 0;
	bus->hcd = hcd;
	bus->hcdCtx = hcdCtx;
	bus->activated.next = bus->activated.prev = NULL;
	memset(bus->addrPool,0xFF,sizeof(bus->addrPool));
	proc_spinlockCreate(&bus->lock,"usb bus lock");
	proc_mutexCreate(&bus->urbLock);
	return EOK;
}



int usb_rootHubEnqueueIntr(usb_urb_t* urb)
{
	usb_hub_t* rootHub = urb->dev->parent;
	if(rootHub->bus->hcd->irq == NULL) { /* No interrupt, install polling timer */
		return -ENYI;
	} else {
		usb_linkUrb(urb);
		return 0;
	}
}

static inline void usb_rootHubGetDescriptor(usb_hub_t* rootHub, usb_urb_t* urb)
{
	usb_ctrlreq_t* req = (usb_ctrlreq_t*) urb->setup;
	void* buf;
	size_t cpySize;
	switch (req->wValue >> 8) {
		case USB_DESC_DEV:
			buf = &rootHub->dev.desc;
			cpySize = sizeof(rootHub->dev.desc);
			break;
		case USB_DESC_CFG:
			buf = rootHub->dev.cfg;
			cpySize = rootHub->dev.cfg->wTotalLength;
			break;
		case USB_DESC_HUB:
			buf = &rootHub->desc;
			cpySize = sizeof(rootHub->desc);
			break;
		default:
			usb_urbError(urb,EINVAL);
			return;
	}
	urb->transferred = cpySize > urb->dataLen ? urb->dataLen : cpySize;
	memcpy(urb->data,buf,urb->transferred);
}


void usb_busUpdateStatus(usb_bus_t* bus,u32 status)
{
	proc_spinlockSet(&bus->lock);
	bus->status |= status;
	proc_spinlockClear(&bus->lock,0);
}


int usb_rootHubEnqueueCtrl(usb_urb_t* urb)
{
	usb_hub_t* rootHub = urb->dev->parent;
	usb_ctrlreq_t* req = (usb_ctrlreq_t*) urb->setup;
	urb->transferred = 0;
	switch (req->bmRequestType) {
		case USB_READ_STD_DEV:
			switch (req->bRequest) {
				case USB_GET_DESC:
					usb_rootHubGetDescriptor(rootHub,urb);
					break;
				case USB_GET_CFG:
					if(urb->dataLen > 0) {
						*(u8*)urb->data = urb->dev->cfg->bConfigurationValue;
						urb->transferred = 1;
					} else
						usb_urbError(urb,ENOMEM);
					break;
				case USB_GET_STAT:
					if(urb->dataLen < 2)
						usb_urbError(urb,ENOMEM);
					else {
						*(u16*)urb->data = 0x0100;
						urb->transferred = 2;
					}
					break;
				default:
					usb_urbError(urb,EINVAL);
			}
			break;
		case USB_WRITE_STD_DEV:
			switch (req->bRequest) {
				case USB_SET_ADDR:
				case USB_SET_CFG:
					break;
				default:
					usb_urbError(urb,EINVAL);
			}
			break;
		case USB_READ_CLASS_DEV:
			switch(req->bRequest) {
				case USB_GET_DESC:
					usb_rootHubGetDescriptor(rootHub,urb);
					break;
				default:
					usb_urbError(urb,EINVAL);
			}
			break;
		case USB_READ_CLASS_OTH:
			switch (req->bRequest) {
				case USB_GET_STAT:
					if(urb->dataLen >= sizeof(usb_hubPortStatus_t)) {
						rootHub->bus->hcd->ops.getPortStatus(rootHub->bus,req->wIndex,urb->data);
						urb->transferred = sizeof(usb_hubPortStatus_t);
					} else
						usb_urbError(urb,ENOMEM);
					break;
				default:
					usb_urbError(urb,EINVAL);
			}
			break;
		case USB_WRITE_CLASS_OTH:
			switch (req->bRequest) {
				case USB_SET_FEAT:
					if (req->wValue == USB_PORT_RESET)
						rootHub->bus->hcd->ops.resetPort(rootHub->bus,req->wIndex);
					else if(req->wValue == USB_PORT_POWER)
						do{}while(0);
					else
						return usb_urbError(urb,EINVAL);
					break;
				case USB_CLEAR_FEAT:
					if (req->wValue == USB_C_PORT_CONN)
						rootHub->bus->hcd->ops.clearConnectChange(rootHub->bus,req->wIndex);
					else if(req->wValue == USB_C_PORT_RESET)
						do{}while(0);
					else
						return usb_urbError(urb,EINVAL);
					break;
				default:
					usb_urbError(urb,EINVAL);
			}
			break;
		default:
			usb_urbError(urb,EINVAL);
	} /* switch (req->bRequestType) */
	urb->status |= USB_URB_DONE;
	usb_linkUrb(urb);
	usb_busUpdateStatus(rootHub->bus,USB_TRANSFER_COMPLETE);
	usb_triggerBus(rootHub->bus);
	return EOK;
}


static inline int usb_rootHubExecute(usb_urb_t* urb)
{
	switch (usb_urbGetPipeType(urb)) {
	   	case USB_EP_INTR: 
			return usb_rootHubEnqueueIntr(urb);
		case USB_EP_CTRL: 
			return usb_rootHubEnqueueCtrl(urb);
		default:
			return -EINVAL;
	}
}

/*
static inline void usb_rootHubDequeueIntr(usb_urb_t* urb)
{
	usb_hub_t* rootHub = urb->dev->parent;
	if(rootHub->bus->hcd->irq == NULL) / * No interrupt, install polling timer * /
		main_printf(ATTR_ERROR,"usb: [rootHubDequeueIntr] NYI\n");
	else
		usb_rootHubDequeue(urb);
}


static inline void usb_rootHubCancel(usb_urb_t* urb)
{
	switch (usb_urbGetPipeType(urb)) {
	   	case USB_EP_INTR: 
			usb_rootHubDequeueIntr(urb);
			return;
		case USB_EP_CTRL: 
			usb_rootHubDequeue(urb);
			return;
	}
}
*/


u8 usb_busAllocAddr(usb_bus_t* bus)
{
	int i,retval;
	for(i = 0;i < USB_LOWEST_INVALID_ADDR / 32;i++)
		if(bus->addrPool[i] != 0){
			retval = hal_cpuGetFirstBit(bus->addrPool[i]);
			bus->addrPool[i] ^= (1 << retval);
			return retval + i * 32;
		}
	return USB_LOWEST_INVALID_ADDR; /* i * 32 , highest valid address is 127*/
}


void usb_busFreeAddr(usb_bus_t* bus,u8 addr)
{
	bus->addrPool[addr / 32] ^= 1 << (addr % 32);
}


int usb_hcdEnqueueUrb(usb_urb_t* urb)
{
	/* Check if root hub if so, execute root hub routines
	 * if not call hcd specific enquque function */
	if(usb_isRootHub(urb->dev))
		return usb_rootHubExecute(urb);
	else
		return urb->dev->parent->bus->hcd->urbEnqueue(urb->dev->parent->bus,urb);
}


int usb_hcdDequeueUrb(usb_urb_t* urb)
{
	if(usb_isRootHub(urb->dev))
		return usb_dequeueUrb(urb);
	else
		return urb->dev->parent->bus->hcd->urbDequeue(urb->dev->parent->bus,urb);
}


int usb_hcdDropEndpoint(usb_dev_t* dev,u8 num)
{
	dev->parent->bus->hcd->ops.dropEndpointPriv(dev,num);
	return 0;
}


void usb_hcdUpdateCtrlEp(usb_dev_t* dev)
{
	if(!usb_isRootHub(dev))
		dev->parent->bus->hcd->ops.updateCtrlEp(dev);
}

