#include <string.h>
#include <include/errno.h>
#include <lib/list.h>
#include <lib/ptr.h>
#include <proc/spinlock.h>
#include <proc/threads.h>
#include <main/std.h>
#include <vm/if.h>

#include "urb.h"
#include "hub.h"


static void _usb_hubProcessPortChanges(void);
static void usb_hubProcessPortChange(usb_hub_t* hub,int num);
static void _usb_hubCheckDevicesAlive(void);
static void usb_hubIntrCallback(usb_urb_t* urb);
static void usb_hubRemoveDevice(usb_hub_t* hub,u16 portNum);
inline static int usb_hubGetDescriptor(usb_dev_t* dev,usb_hubDesc_t* desc);
inline static int usb_hubSetPortFeature(usb_dev_t* dev,u16 portNum,u8 feature);


static struct {
	LIST_HEAD(usb_dev_) devices;
	spinlock_t lock;
	thq_t queue; 
} hub_g;


static usb_driver_t hub_driver;

int usb_hubProbe(usb_dev_t* dev)
{
	int rc,i;
	usb_hub_t* hub;
	if(dev->desc.bDeviceClass != USB_CLASS_HUB)	
		return 0;
	usb_epDesc_t* intrEp = usb_cfgEpDesc(dev->cfg,0,0);

	if((intrEp->bmAttributes & USB_EP_TYPE_MASK) != USB_EP_INTR)
		return 0;

	if((rc = usb_addEndpoint(dev,intrEp)))
		return rc;

	if(&dev->parent->bus->rootHub.dev != dev){ /* new hub */
		hub = vm_kmalloc(sizeof(*hub));
		if(hub == NULL){
			usb_dropEndpoint(dev,intrEp);
			return -ENOMEM;
		}
		memcpy(&hub->dev,dev,sizeof(*dev));
		dev->parent->children[dev->port] = &hub->dev;
		vm_kfree(dev);
		dev = &hub->dev;
	} else
		hub = PTR_CONTAINER(dev,usb_hub_t,dev);
	
	hub->bus = dev->parent->bus;
	// Get hub descriptor
	rc = usb_hubGetDescriptor(&hub->dev,&hub->desc);
	if(rc < 0) {
		main_printf(ATTR_ERROR,"usb: [hubProbe] Error getting hub descriptor (%d), "
				"assuming only 1 port\n",rc);
		hub->desc.bNbrPorts = 1;
	}
	if(hub->desc.bNbrPorts > 31){
		main_printf(ATTR_ERROR,"usb: [hubProbe] Current hub driver doesn't support hubs that have more than 31 ports\n");
		hub->desc.bNbrPorts = 31;
	}
	hub->children = vm_kmalloc(hub->desc.bNbrPorts * sizeof(hub->children[0]));
	if(hub->children == NULL)
		return -ENOMEM;
	for(i = 0;i < hub->desc.bNbrPorts;i++)
		hub->children[i] = NULL;
	memset(hub->intrData,0,USB_HUB_DATA_LEN);
	hub->portChange = 0;
	
	proc_spinlockSet(&hub_g.lock);
	LIST_ADD(&hub_g.devices,&hub->dev,devices);
	proc_spinlockClear(&hub_g.lock,0);

	usb_fillUrb(&hub->intrUrb,&hub->dev,(intrEp->bEndpointAddress & USB_EP_ADDR_MASK) 
			| USB_PIPE_TYPE(USB_EP_INTR) | USB_DIR_IN, 
			hub->intrData,(hub->desc.bNbrPorts + 7) / 8,usb_hubIntrCallback);
	hub->intrUrb.interval = intrEp->bInterval;
	usb_submitUrb(&hub->intrUrb);

	for(i = 0;i < hub->desc.bNbrPorts;i++) {
		proc_threadSleep(1000 * 200);
		if(usb_hubSetPortFeature(&hub->dev,i,USB_PORT_POWER) >= 0) {
			proc_threadSleep(1000 * 200);
			usb_hubProcessPortChange(hub,i);
		} else
			main_printf(ATTR_ERROR,"usb: [hubProbe] powering port %d failed\n",i+1);
	}

	return 1;
}


void usb_hubDisconnect(usb_dev_t* dev)
{
	int i;
	usb_hub_t* hub = PTR_CONTAINER(dev,usb_hub_t,dev);
	usb_killUrb(&hub->intrUrb);
	for(i = 0;i < hub->desc.bNbrPorts;i++)
		usb_hubRemoveDevice(hub,i);
	vm_kfree(hub->children);
	proc_spinlockSet(&hub_g.lock);
	LIST_REMOVE(&hub_g.devices,dev,devices);
	proc_spinlockClear(&hub_g.lock,0);
}

static usb_driver_t hub_driver = {
	.probe = usb_hubProbe,
	.disconnect = usb_hubDisconnect,
};


#define USB_HUB_TIMEOUT 500000
static int usb_hubThread(void* data)
{
	int rc;
	//time_t start,end;
	proc_spinlockSet(&hub_g.lock);
	do {
		//start = timesys_getTime();
		if((rc = proc_threadCondWait(&hub_g.queue,&hub_g.lock,USB_HUB_TIMEOUT)) < 0) {
			if(rc != -ETIME) {
				main_printf(ATTR_ERROR,"usb_hub: hubThread conditional wait error (%d)\n",rc);
				proc_spinlockSet(&hub_g.lock);
			} else { 
				/*
				end = timesys_getTime();
				proc_spinlockClear(&hub_g.lock,0);
				main_printf(ATTR_INFO,"usb_hub: timeout (expected/actual): %d/%d\n",USB_HUB_TIMEOUT,end-start);
				proc_spinlockSet(&hub_g.lock);
				*/
				_usb_hubCheckDevicesAlive();
			}
		} else {
			_usb_hubProcessPortChanges();
		}
	} while (1);
	proc_spinlockClear(&hub_g.lock,0);
	return 0;
}


int usb_hubInit(usb_driver_t** drvPtr)
{
	int rc;
	LIST_HEAD_INIT(&hub_g.devices);
	proc_spinlockCreate(&hub_g.lock,"usb hub driver lock");
	proc_thqCreate(&hub_g.queue);
	if(EOK != (rc = proc_thread(NULL,usb_hubThread,NULL,0,NULL,ttRegular))){
		proc_spinlockTerminate(&hub_g.lock);
		main_printf(ATTR_ERROR,"usb_hubInit: Failed to start usb hub thread (%d)\n",rc);
		return rc;
	}

	*drvPtr = &hub_driver;
	return 0;
}


inline static int usb_hubGetDescriptor(usb_dev_t* dev,usb_hubDesc_t* desc)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_READ_CLASS_DEV,
		.bRequest = USB_GET_DESC,
		.wValue = USB_DESC_HUB << 8,
		.wIndex = 0,
		.wLength = sizeof(usb_hubDesc_t),
	};
	return usb_controlTransfer(dev,0,&req,(u8*)desc,req.wLength,USB_TIMEOUT);
}


inline static int usb_hubGetPortStatus(usb_dev_t* dev,u16 portNum,void* status)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_READ_CLASS_OTH,
		.bRequest = USB_GET_STAT,
		.wValue = 0,
		.wIndex = portNum+1,
		.wLength = 4,
	};
	return usb_controlTransfer(dev,0,&req,status,req.wLength,USB_TIMEOUT);
}


inline static int usb_hubSetPortFeature(usb_dev_t* dev,u16 portNum,u8 feature)
{	
	usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_CLASS_OTH,
		.bRequest = USB_SET_FEAT,
		.wValue = feature,
		.wIndex = portNum+1,
		.wLength = 0,
	};
	return usb_controlTransfer(dev,0,&req,NULL,0,USB_TIMEOUT);
}


inline static int usb_hubClearPortFeature(usb_dev_t* dev,u16 portNum,u8 feature)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_CLASS_OTH,
		.bRequest = USB_CLEAR_FEAT,
		.wValue = feature,
		.wIndex = portNum+1,
		.wLength = 0,
	};
	return usb_controlTransfer(dev,0,&req,NULL,0,USB_TIMEOUT);
}


static int usb_hubNewDevice(usb_hub_t* hub,u16 portNum,u16 speed)
{
	int i,rc;
	usb_dev_t* dev = vm_kmalloc(sizeof(usb_dev_t));
	if(dev == NULL)
		return -ENOMEM;
	dev->addr = 0;
	dev->state = USB_CONNECTED;
	dev->speed = speed;
	dev->port = portNum;
	dev->parent = hub;
	dev->cfg = NULL;
	for(i = 0;i < USB_MAX_EP_LEN;i++)
		dev->ep[i] = NULL;
	LIST_ENTRY_INIT(&dev->devices);
	dev->drv = NULL;
	hub->children[portNum] = dev;
	dev->desc.bMaxPacketSize0 = USB_MIN_CTRL_PACKET_SIZE;
	rc = usb_addEndpoint(dev,NULL);
	if(rc != EOK || usb_connect(hub->children[portNum]) < 0){
		usb_dropEndpoint(hub->children[portNum],NULL);
		vm_kfree(hub->children[portNum]);
		hub->children[portNum] = NULL;
	}
	return 0;
}

static void usb_hubRemoveDevice(usb_hub_t* hub,u16 portNum)
{
	if(hub->children[portNum] != NULL) {
		usb_dropEndpoint(hub->children[portNum],NULL);
		usb_disconnect(hub->children[portNum]);
		vm_kfree(hub->children[portNum]);
		hub->children[portNum] = NULL;
	}
}


static void usb_hubProcessPortChange(usb_hub_t* hub,int num)
{
	usb_hubPortStatus_t status;

	proc_threadSleep(1000 * 100);
	usb_hubClearPortFeature(&hub->dev,num,USB_C_PORT_CONN);
	proc_threadSleep(1000 * 100);
	usb_hubGetPortStatus(&hub->dev,num,&status);
	if(status.port & USB_PORT_STATUS_CONN_MASK){
		u8 speed;
		main_printf(ATTR_DEBUG,"usb_hubProcessPortChange: device attached\n");
		if(hub->children[num] != NULL) {
			main_printf(ATTR_DEBUG,"usb_hubProcessPortChange: device already attached\n");
			return;
		}

		usb_hubSetPortFeature(&hub->dev,num,USB_PORT_RESET);
		//do {
			proc_threadSleep(1000 * 50);
			usb_hubGetPortStatus(&hub->dev,num,&status);
		//} while (status.port & USB_PORT_STATUS_RESET_MASK);
		proc_threadSleep(1000 * 50);
		usb_hubClearPortFeature(&hub->dev,num,USB_C_PORT_RESET);
		speed = (status.port & USB_PORT_STATUS_SPEED_MASK) >> USB_PORT_STATUS_SPEED_SHIFT;
		usb_hubNewDevice(hub,num,speed);
	} else {
		main_printf(ATTR_DEBUG,"usb: [hubProcessPortChange] device detached\n");
		usb_hubRemoveDevice(hub,num);
	}
}


static void _usb_hubProcessPortChanges(void)
{
	usb_hub_t* hub;
	if(LIST_IS_EMPTY(&hub_g.devices))
		return;
	hub = PTR_CONTAINER(hub_g.devices.first,usb_hub_t,dev);
	do {
		while(hub->portChange != 0) {
			int i = hal_cpuGetFirstBit(hub->portChange);
			hub->portChange ^= 1 << i;	
			proc_spinlockClear(&hub_g.lock,0);
			usb_hubProcessPortChange(hub,i-1);
			proc_spinlockSet(&hub_g.lock);
		}
		hub = PTR_CONTAINER(hub->dev.devices.next,usb_hub_t,dev);
	} while (&hub->dev != hub_g.devices.first);
}


static void _usb_hubCheckDevicesAlive(void)
{
	usb_hub_t* hub;
	if(LIST_IS_EMPTY(&hub_g.devices))
		return;
	hub = PTR_CONTAINER(hub_g.devices.first,usb_hub_t,dev);
	do {
		int i,rc;
		u16 status;
		proc_spinlockClear(&hub_g.lock,0);
		for(i = 0;i < hub->desc.bNbrPorts;i++) 
			if(hub->children[i] != NULL) {
				if((rc = usb_getStatus(hub->children[i],USB_RECIP_DEV,&status)) < 0) {
					main_printf(ATTR_ERROR,"usb: device %d stopped to respond (%d), "
							"forced device removal\n",hub->children[i]->addr,rc);
					usb_hubRemoveDevice(hub,i);
					if((rc = usb_hubSetPortFeature(&hub->dev,i,USB_PORT_RESET)) < 0)
						main_printf(ATTR_ERROR,"usb: Couldn't reset port after forced disconnect (%d), "
								"port might be unavailable\n",rc);
				}					
			} else {
				usb_hubPortStatus_t status = {0};
				if((rc = usb_hubSetPortFeature(&hub->dev,i,USB_PORT_RESET)) < 0)
						main_printf(ATTR_ERROR,"usb: Couldn't reset port (%d)\n",rc);
				usb_hubGetPortStatus(&hub->dev,i,&status);
				if(status.port & USB_PORT_STATUS_CONN_MASK){
					u8 speed;
					speed = (status.port & USB_PORT_STATUS_SPEED_MASK) >> USB_PORT_STATUS_SPEED_SHIFT;
					usb_hubNewDevice(hub,i,speed);
				}
			}
		proc_spinlockSet(&hub_g.lock);
		hub = PTR_CONTAINER(hub->dev.devices.next,usb_hub_t,dev);
	} while (&hub->dev != hub_g.devices.first);
}


static void usb_hubIntrCallback(usb_urb_t* urb)
{
	usb_hub_t* hub = PTR_CONTAINER(urb->dev,usb_hub_t,dev);
	u32 portChange = *(u32*)urb->data;
	*(u32*)urb->data = 0;
	urb->status = 0;
	portChange &= ~(u32)1; /* Remove hub change field for now */
	main_printf(ATTR_DEBUG,"usb_hubIntrCallback: %x\n",portChange);
	proc_spinlockSet(&hub_g.lock);
	hub->portChange |= portChange;
	proc_threadCondSignal(&hub_g.queue);	
	proc_spinlockClear(&hub_g.lock,0);
	usb_submitUrb(urb);
}


int usb_isHub(usb_dev_t* dev)
{
	usb_dev_t* iter;
	proc_spinlockSet(&hub_g.lock);
	LIST_FIND(&hub_g.devices,iter,devices,iter == dev);
	proc_spinlockClear(&hub_g.lock,0);
	return iter != NULL;
}


usb_dev_t* usb_getRootHub(int i)
{
	int c = 0;
	usb_dev_t* retval = NULL;
	usb_dev_t* iter;
	proc_spinlockSet(&hub_g.lock);
	iter = hub_g.devices.first;
	if(iter != NULL)
		do {
			if(usb_isRootHub(iter) && i == c++)
				retval = iter;
			iter = iter->devices.next;
		} while (retval == NULL && iter != hub_g.devices.first);
	proc_spinlockClear(&hub_g.lock,0);
	return retval;
}

/*
static void usb_hubIntrDebugCallback(usb_urb_t* urb)
{
	main_printf(ATTR_DEBUG,"usb: [hubIntrDebugCallback] %x\n",(u32)(((u8*)urb->data)[0]));
	usb_submitUrb(urb);
}
*/
