 /**
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @brief USB stack core functions
 * @copyright 2014 Phoenix Systems
 * @author: Pawel Tryfon <pawel.tryfon@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <proc/if.h>
#include <proc/spinlock.h>
#include <lib/ptr.h>
#include <main/std.h>
#include <vm/dmapool.h>
#include <main/string.h>

#include "hub.h"
#include "usb.h"
#include "hcd.h"
#include "urb.h"


static struct {
	usb_driverList_t drivers;
	LIST_HEAD(usb_bus_) activated; /**< list of buses that have port change or complete transfers */
	spinlock_t lock;
	thq_t queue; 
} usb_g;


static inline void _usb_processActivatedBuses(void)
{
	while(!LIST_IS_EMPTY(&usb_g.activated)){
		usb_bus_t* bus = usb_g.activated.first;
		LIST_REMOVE(&usb_g.activated,bus,activated);
		bus->activated.next = bus->activated.prev = NULL;
		proc_spinlockClear(&usb_g.lock,0);
		usb_processBusEvents(bus);
		proc_spinlockSet(&usb_g.lock);
	}
}


static int usb_mainThread(void* data)
{
	proc_spinlockSet(&usb_g.lock);
	do {
		_usb_processActivatedBuses();
		if(proc_threadCondWait(&usb_g.queue,&usb_g.lock,0) < 0) {
			main_printf(ATTR_ERROR,"usb: [usb_main] mainThread conditional wait error\n");
			proc_spinlockSet(&usb_g.lock);
		}
	} while (1);
	proc_spinlockClear(&usb_g.lock,0);

	return 0;
}

void usb_triggerBus(usb_bus_t* bus)
{
	proc_spinlockSet(&usb_g.lock);
	if(bus->activated.next == NULL){
		LIST_ADD(&usb_g.activated,bus,activated);
		proc_threadCondSignal(&usb_g.queue);	
	}
	proc_spinlockClear(&usb_g.lock,0);
}


void usb_addDriver(usb_driver_t* drv)
{
	proc_spinlockSet(&usb_g.lock);
	LIST_ADD(&usb_g.drivers,drv,drivers);
	proc_spinlockClear(&usb_g.lock,0);
}


void _usb_init(void)
{
	int rc;
	usb_driver_t* drv;

	proc_spinlockCreate(&usb_g.lock,"usb global lock");
	LIST_HEAD_INIT(&usb_g.drivers);
	LIST_HEAD_INIT(&usb_g.activated);
	proc_thqCreate(&usb_g.queue);

	if(usb_hubInit(&drv) != EOK){
		main_printf(ATTR_ERROR,"usb: [_usb_init] hub driver init failed. USB down.\n");
		return;
	} else
		usb_addDriver(drv);

	if(EOK != (rc = proc_thread(NULL,usb_mainThread,NULL,0,NULL,ttRegular))){
		main_printf(ATTR_ERROR,"usb: [_usb_init] Failed to start usb main thread (%d)\n",rc);
		return;
	}

	// deffer: Does drivers-initialization need to be inserted right here???

	if(usb_insertRootHubs() != EOK)
		main_printf(ATTR_ERROR,"usb: [_usb_init] root hubs initialization failed\n");

	main_printf(ATTR_INFO,"usb: drivers and root hubs initialized 2\n");
}


int usb_connect(usb_dev_t* dev)
{
	int rc;
	usb_driver_t* drv;
	usb_cfgDesc_t cfgDesc;
	usb_cfgDesc_t* newCfg = NULL;
	u8 addr = usb_busAllocAddr(dev->parent->bus);
	if(addr == USB_LOWEST_INVALID_ADDR)
		return -ENOENT;
	if((rc = usb_setAddress(dev,addr)) < 0) {
		usb_busFreeAddr(dev->parent->bus,addr);
		return rc;
	} else {
		dev->addr = addr;
		usb_hcdUpdateCtrlEp(dev);
	}
	if(rc >= 0)
		rc = usb_getDescriptor(dev,USB_DESC_DEV,0,(u8*)&dev->desc,USB_MIN_CTRL_PACKET_SIZE);
	if(rc >= EOK){
		dev->ep[0]->desc->wMaxPacketSize = dev->desc.bMaxPacketSize0;
		usb_hcdUpdateCtrlEp(dev);
	}

	if(rc >= EOK)
		rc = usb_getDescriptor(dev,USB_DESC_DEV,0,(u8*)&dev->desc,sizeof(dev->desc));
	if(rc >= EOK)
		rc = usb_getDescriptor(dev,USB_DESC_CFG,0,(u8*)&cfgDesc,sizeof(cfgDesc));
	if(rc >= EOK){
		newCfg = vm_kmalloc(cfgDesc.wTotalLength);
		rc = newCfg == NULL ? -ENOMEM : EOK;
	}
	if(rc >= EOK)
		rc = usb_getDescriptor(dev,USB_DESC_CFG,0,(u8*)newCfg,cfgDesc.wTotalLength);

	/* Set configuration to the first one, driver can change it if needed */
	if(rc >= EOK){
		dev->cfg = newCfg;
		rc = usb_setConfiguration(dev,dev->cfg->bConfigurationValue);
	}

	if(rc >= EOK){
		u8 cfgValue;
		rc = usb_getConfiguration(dev,&cfgValue);
		if(rc >= EOK && cfgValue != dev->cfg->bConfigurationValue)
			rc = -EINVAL;
	}
	
	if(rc < 0){
		if(newCfg != NULL) vm_kfree(newCfg);
		usb_busFreeAddr(dev->parent->bus,addr);
		main_printf(ATTR_ERROR,"usb: Error connecting device\n");
		return rc;
	}

	main_printf(ATTR_INFO,"usb: Device connected. Looking for driver ...\n");
	drv = usb_g.drivers.first;
	if(drv == NULL)
		return 0;
	do {
		dev->drv = drv;
		if(drv->probe(dev) == 1)
			return 1;
		drv = drv->drivers.next;
	} while (drv != usb_g.drivers.first);
	dev->drv = NULL;
	main_printf(ATTR_INFO,"usb: No driver found for device\n");
	return 0;
}


int usb_disconnect(usb_dev_t* dev)
{
	int i;
	if(dev->drv != NULL)
		dev->drv->disconnect(dev);
	for(i = 0;i < USB_MAX_EP_LEN;i++)
		if(dev->ep[i] != NULL) usb_dropEndpoint(dev,dev->ep[i]->desc);
	if(dev->cfg != NULL) vm_kfree(dev->cfg);
	if(dev->addr != 0) usb_busFreeAddr(dev->parent->bus,dev->addr);
	return 0;
}


int usb_addEndpoint(usb_dev_t* dev, usb_epDesc_t* ep)
{
	u8 epNum;
	if(ep != NULL) {
		if((epNum = ep->bEndpointAddress & USB_EP_NUM_MASK) > USB_MAX_EP_LEN){
			main_printf(ATTR_ERROR,"usb: [usb_addEndpoint] endpoint array too short\n");
			return -ENOENT;
		}
		if(dev->ep[epNum] != NULL) {
			main_printf(ATTR_ERROR,"usb: [addEndpoint] endpoint already added (device not compliant with USB specification)\n");	
			return -EBUSY;
		}
		dev->ep[epNum] = vm_kmalloc(sizeof(*dev->ep[0]));
	} else { /* 0 control endpoint */
		/* Alloc ep + epdesc, set epDesc */
		epNum = 0;
		dev->ep[epNum] = vm_kmalloc(sizeof(*dev->ep[0])+sizeof(*ep));
		if(dev->ep[epNum] != NULL) {
			ep = (usb_epDesc_t*)PTR_ADD(dev->ep[0],sizeof(*dev->ep[0]));
			ep->bLength = sizeof(*dev->ep[0]);
			ep->bDescriptorType = USB_DESC_EP;
			ep->bEndpointAddress = 0;
			ep->bmAttributes = 0;
			ep->wMaxPacketSize = dev->desc.bMaxPacketSize0;
			ep->bInterval=0;
		}
	} 

	if(dev->ep[epNum] == NULL)
		return -ENOMEM;
	dev->ep[epNum]->desc = ep;
	LIST_HEAD_INIT(&dev->ep[epNum]->urbs);
	dev->ep[epNum]->hcpriv = NULL;
	return 0;
}


int usb_dropEndpoint(usb_dev_t* dev,usb_epDesc_t* ep)
{
	u8 epNum;
	if(ep == NULL)
		epNum = 0;
	else if(!(epNum = ep->bEndpointAddress & USB_EP_NUM_MASK)){
		main_printf(ATTR_ERROR,"usb: [usb_dropEndpoint] endpoint array too short\n");
		return -ENOENT;
	}

	if(dev->ep[epNum] != NULL){
		if(!LIST_IS_EMPTY(&dev->ep[epNum]->urbs)){
			main_printf(ATTR_ERROR,"usb: [usb_dropEndpoint] active urbs exist\n");
			return -EBUSY;
		}
		usb_hcdDropEndpoint(dev,epNum);
		vm_kfree(dev->ep[epNum]);
		dev->ep[epNum] = NULL;
	}
	return 0;
}


int usb_submitUrb(usb_urb_t* urb)
{
	usb_ep_t* ep;
	u8 epNum,epType;

	if(urb == NULL || urb->callback == NULL)
		return -EINVAL;
	if(urb->dev == NULL)
		return -ENODEV;
	epNum = usb_urbGetEpNum(urb);
	epType = usb_urbGetPipeType(urb);
	if(epNum != 0){ /* Consistency checking for ep > 0 */
		if(epNum >= USB_MAX_EP_LEN)
			return -ENOENT;
		ep = urb->dev->ep[epNum];
		if(ep == NULL)
			return -ENOENT;
		if(epType != (ep->desc->bmAttributes & USB_EP_TYPE_MASK))
			return -EINVAL;
		if(usb_urbGetPipeDir(urb) != (ep->desc->bEndpointAddress & USB_EP_DIR_MASK))
			return -EINVAL;
	} else {
		if(epType != USB_EP_CTRL)
			return -EINVAL;
	}
	/* If transfer is write, write buffer needs to be flushed, if read it needs another buffer allocated
	 * that will be later copied to primary buffer */
	/* Both write and read need dma buffer */

	/* Link urb to endpoint */
	return usb_hcdEnqueueUrb(urb);
}


void usb_killUrb(usb_urb_t* urb)
{
	int i = 0;
	if(usb_hcdDequeueUrb(urb) == -EBUSY)
		while(!(urb->status & USB_URB_COMPLETED)) {
			i++;
			if(i == 1000000) {
				main_printf(ATTR_ERROR,"usb [killUrb] waiting for urb completion too long, aborting\n");
				return;
			}
		}
}

typedef struct usb_syncTransferData {
	semaphore_t mutex;
	thq_t tq;
} usb_syncTransferData_t;

void usb_syncTransferCallback(usb_urb_t* urb)
{
	usb_syncTransferData_t* syncData = (usb_syncTransferData_t*) urb->compData;
	proc_semaphoreDown(&syncData->mutex);
	proc_threadCondSignal(&syncData->tq);
	proc_semaphoreUp(&syncData->mutex);
}

int usb_transfer(usb_urb_t* urb,u32 timeout)
{
	int rc;
	struct usb_syncTransferData syncData;
	//time_t start,end;

	proc_semaphoreCreate(&syncData.mutex,1);
	proc_thqCreate(&syncData.tq);
	urb->compData = &syncData;
	urb->callback = usb_syncTransferCallback; 
	proc_semaphoreDown(&syncData.mutex);
	if((rc = usb_submitUrb(urb))) {
		proc_semaphoreUp(&syncData.mutex);
	} else {
		//start = timesys_getTime();
		rc = proc_condWait(&syncData.tq,&syncData.mutex,timeout);
		//end = timesys_getTime();
		//if(rc == -ETIME)
		//	main_printf(ATTR_INFO,"usb: [transfer] timeout (expected/actual): %d/%d\n",USB_TIMEOUT,end-start);
		if(rc < 0)
			usb_killUrb(urb);
		proc_semaphoreUp(&syncData.mutex);
	}
	proc_semaphoreTerminate(&syncData.mutex);

	if(rc)
		return rc;
	else
		return usb_urbReturnStatus(urb);
}


int usb_controlTransfer(usb_dev_t* dev,u8 epAddr, usb_ctrlreq_t* req,u8* data,u16 size,u32 timeout)
{
	int rc;
	usb_urb_t urb;
	usb_fillUrb(&urb,dev,(req->bmRequestType & USB_DIR_IN) | USB_PIPE_TYPE(USB_EP_CTRL) | (epAddr & USB_EP_ADDR_MASK),data,size,NULL);
	urb.setup = req;
	urb.setupLen = sizeof(*req);
	rc = usb_transfer(&urb,timeout);
	if(rc != EOK)
		main_printf(ATTR_DEBUG,"usb: control transfer failure (%d) {bmRequestType = %x, bRequest, = %x, wValue = %x, wIndex = %x, wLength = %x}\n",rc,
				(u32)req->bmRequestType,(u32)req->bRequest,(u32)req->wValue,(u32)req->wIndex,(u32)req->wLength);
	else
		rc = urb.transferred;
	return rc;
}


int usb_bulkTransfer(usb_dev_t* dev,u8 epAddr,u8 dir,u8* data,u16 size,u32 timeout)
{
	int rc;
	usb_urb_t urb;
	usb_fillUrb(&urb,dev,(dir & USB_DIR_IN) | USB_PIPE_TYPE(USB_EP_BULK) | (epAddr & USB_EP_ADDR_MASK),data,size,NULL);
	rc = usb_transfer(&urb,timeout);
	if(rc != EOK)
		main_printf(ATTR_DEBUG,"usb: bulk transfer failure (%d)\n",rc);
	else
		rc = urb.transferred;
	return rc;
}

int usb_intrTransfer(usb_dev_t* dev,u8 epAddr,u8 dir,u8* data,u16 size,u32 timeout)
{
	int rc;
	usb_urb_t urb;
	usb_fillUrb(&urb,dev,(dir & USB_DIR_IN) | USB_PIPE_TYPE(USB_EP_INTR) | (epAddr & USB_EP_ADDR_MASK),data,size,NULL);
	rc = usb_transfer(&urb,timeout);
	if(rc != EOK)
		main_printf(ATTR_DEBUG,"usb: intr transfer failure (%d)\n",rc);
	else
		rc = urb.transferred;
	return rc;
}


int usb_setAddress(usb_dev_t* dev,u8 addr)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_STD_DEV,
		.bRequest = USB_SET_ADDR,
		.wValue = addr,
		.wIndex = 0,
		.wLength = 0,
	};
	if(addr>127)
		return -EINVAL;
	return usb_controlTransfer(dev,0,&req,NULL,0,USB_TIMEOUT);
}

int usb_getDescriptor(usb_dev_t* dev,u8 type,u8 index,u8* buf,uint16_t size)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_READ_STD_DEV,
		.bRequest = USB_GET_DESC,
		.wValue = (type << 8) | index,
		.wIndex = 0,
		.wLength = size,
	};
	return usb_controlTransfer(dev,0,&req,buf,size,USB_TIMEOUT);
}

int usb_setConfiguration(usb_dev_t* dev,u8 cfgValue)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_STD_DEV,
		.bRequest = USB_SET_CFG,
		.wValue = cfgValue,
		.wIndex = 0,
		.wLength = 0,
	};
	return usb_controlTransfer(dev,0,&req,NULL,0,USB_TIMEOUT);
}       

int usb_getConfiguration(usb_dev_t* dev,u8* cfgValue)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_READ_STD_DEV,
		.bRequest = USB_GET_CFG,
		.wValue = 0,
		.wIndex = 0,
		.wLength = 1,
	};
	return usb_controlTransfer(dev,0,&req,cfgValue,1,USB_TIMEOUT);
}


int usb_getStatus(usb_dev_t* dev,u8 recip,u16* status)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_READ_STD_DEV | (recip & 0x03),
		.bRequest = USB_GET_STAT,
		.wValue = 0,
		.wIndex = 0,
		.wLength = 2,
	};
	return usb_controlTransfer(dev,0,&req,(u8*)status,2,USB_TIMEOUT);
}


int usb_resetEndpoint(usb_dev_t* dev,u8 num)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_STD_EP,
		.bRequest = USB_CLEAR_FEAT,
		.wValue = USB_FEAT_EP_HALT,
		.wIndex = num,
		.wLength = 0,
	};
	return usb_controlTransfer(dev,0,&req,NULL,0,USB_TIMEOUT);
}


/* usbPath is 8 long array */
usb_dev_t* usb_getByPath(signed char* usbPath)
{
	int i;
	usb_dev_t* dev;
	usb_hub_t* hub;
	if(usbPath[0] == -1)
		return NULL;
	dev = usb_getRootHub(usbPath[0]);
	if(dev == NULL)
		return NULL;
	for(i = 1;usbPath[i] != -1;i++) {
		if(!usb_isHub(dev))
			return NULL;
		hub = PTR_CONTAINER(dev,usb_hub_t,dev);
		dev = hub->children[usbPath[i]];
	}
	return dev;
}
