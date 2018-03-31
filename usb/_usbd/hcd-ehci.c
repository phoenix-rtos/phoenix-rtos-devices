#include <hal/if.h>
#include<usb/hcd.h>
#include <lib/ptr.h>
#include<vm/dmapool.h>
#include<vm/if.h>
#include <main/if.h>
#include<string.h>

#include "../urb.h"
#include "ehci_def.h"

typedef LIST_HEAD(usb_ehciQtd_) usb_ehciQtdList_t;
typedef LIST_HEAD(usb_ehciQh_) usb_ehciQhList_t;

static int usb_ehciCreateTransfer(usb_urb_t* urb,usb_ehciQtdList_t* qtdList);
static int usb_ehciEnqueueQh(usb_bus_t* bus,usb_urb_t* urb, usb_ehciQtdList_t* qtdList);
static int _usb_ehciUrbDequeue(usb_bus_t* bus,usb_urb_t* urb);
static void usb_ehciUnlinkAsync(usb_bus_t* bus,usb_ehciQh_t* qh);
static void usb_ehciUnlinkIntr(usb_bus_t* bus,usb_ehciQh_t* qh);


typedef struct usb_ehciCtx_ {
  page_t *page;
	void* ioVaddr;
	usb_ehciCapReg_t* creg;
	usb_ehciOpReg_t* oreg;
	volatile u32* mode; 
	volatile u32* periodicQueue;
	usb_ehciItd_t* itd;
	addr_t itdAddr;
	usb_ehciQhList_t periodic;
	usb_ehciQhList_t async;
} usb_ehciCtx_t;


/** Pool of qh and qtd descriptors */
static vm_dmapool_t ehci_pool64 = {.size = 0};


static int usb_ehciIrq(unsigned int n,cpu_context_t* ctx,void* data)
{
	int i;
	usb_bus_t* bus = (usb_bus_t*) data;
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	u32 sts;
	u32 portsc;
	u32 status = 0;
	u32 portChange = 0;

	sts = ehci->oreg->USBSTS;
    if(sts & USB_USBSTS_SEI_MASK){
		status |= USB_SYSTEM_ERROR;
        return 0;
    }

	if(sts & USB_USBSTS_PCI_MASK) {
		ehci->oreg->USBSTS = USB_USBSTS_PCI_MASK;
		status |= USB_PORT_CHANGE;
		for(i = 0;i < bus->rootHub.desc.bNbrPorts;i++){
			portsc = ehci->oreg->PORTSC[i];
			if(portsc & USB_PORTSC_CSC_MASK){
				//ehci->oreg->PORTSC[i] = portsc | USB_PORTSC_CSC_MASK;
				portChange |= 1 << (i+1);
			}
		}
	}
	if(sts & (USB_USBSTS_UI_MASK | USB_USBSTS_UEI_MASK)) {
		ehci->oreg->USBSTS = (USB_USBSTS_UI_MASK | USB_USBSTS_UEI_MASK);
		status |= USB_TRANSFER_COMPLETE;
	}
	
	proc_spinlockSet(&bus->lock);

	bus->status |= status;
	bus->portChange |= portChange;

	proc_spinlockClear(&bus->lock,0);

	usb_triggerBus(bus);

	return 0;
}


/* Return aligned data buffer, urb->hcpriv contains actual beginning of the buffer */
static u8 *usb_ehciSetupTransferBuffer(usb_urb_t *urb)
{
	u8 *retval;
	u8 *current;
	addr_t addr;
	size_t totalLen = urb->dataLen;

	if (usb_urbGetPipeType(urb) == USB_EP_CTRL)
		totalLen += urb->setupLen;

	if (totalLen <= 64)
		retval = urb->hcpriv = vm_dmapoolAlloc(&ehci_pool64, &addr);

	else {

		/* Isolating buffer from both sides to prevent any cache-related errors */
		urb->hcpriv = vm_kmalloc(totalLen + 2 * SIZE_CACHE_LINE - 1);
		retval = PTR_ALIGN(urb->hcpriv, SIZE_CACHE_LINE);
	}
	current = retval;

	if (usb_urbGetPipeType(urb) == USB_EP_CTRL) {
		memcpy(current, urb->setup, urb->setupLen);
		current += urb->setupLen;
	}

	if (usb_urbGetPipeDir(urb) == USB_DIR_OUT) {
		memcpy(current, urb->data, urb->dataLen);
		current += urb->dataLen;
	}
	hal_cpuFlushCache(retval,current-retval);
	return retval;
}


static void usb_ehciFreeTransferBuffer(usb_urb_t* urb)
{
	size_t totalLen = urb->dataLen;
	if(usb_urbGetPipeType(urb) == USB_EP_CTRL)
		totalLen += urb->setupLen;

	/* Copy transfer results if finished and inbound */
	if((urb->status & USB_URB_DONE) && (usb_urbGetPipeDir(urb) == USB_DIR_IN)) { 
		u8* current;
		if(totalLen <= 64)
			current = urb->hcpriv;
		else
			current = PTR_ALIGN(urb->hcpriv,SIZE_CACHE_LINE);
		if(usb_urbGetPipeType(urb) == USB_EP_CTRL)
			current += urb->setupLen;
		hal_cpuInvalCache(current,urb->transferred);
		memcpy(urb->data,current,urb->transferred);
	}
	if(totalLen <= 64)
		vm_dmapoolFree(&ehci_pool64,urb->hcpriv);
	else
		vm_kfree(urb->hcpriv);
}


static usb_urbList_t usb_ehciFetchCompleted(usb_bus_t* bus, usb_ehciQhList_t* qhList)
{
	usb_urbList_t retval = LIST_HEAD_INITIALIZER;
	usb_ehciQh_t* qh = qhList->first;
	if(LIST_IS_EMPTY(qhList))
		return retval;
	do {
		if(!LIST_IS_EMPTY(&qh->qtdList)){
			usb_ehciQtd_t* qtd = qh->qtdList.first;
			usb_ehciQtd_t* urbStart = qtd;
			/* Extract slices of list coming from one urb 
			 * if all qtd from one urb finished add urb to completed*/
			// INVALIDATE qh
			hal_cpuInvalCache(qh,sizeof(*qh));
			do {
				// INVALIDATE qtd
				hal_cpuInvalCache(qtd,sizeof(*qtd));
				if(qtd->token & EHCI_QTD_ACTIVE_MASK) {
					if((qtd->dmaAddr == qh->currQtd) && (qh->status & EHCI_QTD_ERROR_MASK)) {
						_usb_ehciUrbDequeue(bus,qtd->urb);
					}
					break;
				}
				if(qtd->token & EHCI_QTD_IOC_MASK) { /* last qtd from urb */
					usb_urb_t* urb = urbStart->urb;
					size_t untransferred = 0;
					usb_ehciQtd_t* nextQtd = qtd->qtdList.next;

					if(nextQtd == urbStart)
						qh->qtdList.first = NULL;
					else {
						urbStart->qtdList.prev->qtdList.next=nextQtd;
						nextQtd->qtdList.prev = urbStart->qtdList.prev;
					}
					qtd->qtdList.next = urbStart;
					urbStart->qtdList.prev = qtd;

					urb->status = USB_URB_DONE | USB_URB_IN_COMPLETION;

					qtd = urbStart;
					do {
						if(qtd->token & EHCI_QTD_ERROR_MASK){
							urb->status |= USB_URB_ERROR;
							if(qtd->token & EHCI_QTD_STALL_MASK)
								urb->status |= EPIPE << USB_URB_ERRNO_SHIFT;
							else if(qtd->token & EHCI_QTD_BUFFER_ERROR_MASK)
								urb->status |= ENOBUFS << USB_URB_ERRNO_SHIFT;
							else if(qtd->token & EHCI_QTD_BUBBLE_MASK)
								urb->status |= EOVERFLOW << USB_URB_ERRNO_SHIFT;
							else if(qtd->token & EHCI_QTD_TRANSACTION_ERROR_MASK)
								urb->status |= EIO << USB_URB_ERRNO_SHIFT;
							else
								urb->status |= EPERM << USB_URB_ERRNO_SHIFT;
							break;
						} else
							untransferred += (qtd->token & EHCI_QTD_BYTES_TO_TRANSFER_MASK) 
								>> EHCI_QTD_BYTES_TO_TRANSFER_SHIFT;

						qtd = qtd->qtdList.next;
					} while (qtd != urbStart);

					if(!(urb->status & USB_URB_ERROR))
						urb->transferred = urb->dataLen - untransferred;
					else
						urb->transferred = 0;
					
					nextQtd = urbStart->qtdList.prev->qtdList.next;
					qtd = urbStart;
					do {
						usb_ehciQtd_t* tmp = qtd->qtdList.next;
						vm_dmapoolFree(&ehci_pool64,qtd);
						qtd = tmp;
					} while (qtd != urbStart);
					// TODO: Memcpy and free urb->hcpriv
					usb_ehciFreeTransferBuffer(urb);

					_usb_unlinkUrb(urb);
					LIST_ADD(&retval,urb,urbs);
					qtd = urbStart = nextQtd;
				} else 
					qtd = qtd->qtdList.next;
			} while (qh->qtdList.first != NULL);
		}
		qh = qh->qhList.next;
	} while (qh != qhList->first);
	return retval;
}	


static usb_urbList_t usb_ehciScanTransfers(usb_bus_t* bus)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	usb_urbList_t completedPeriodic = LIST_HEAD_INITIALIZER;
	usb_urbList_t completedAsync = LIST_HEAD_INITIALIZER;

	/* Scan iso and interrupt transfers first */
	/* Scan async transfers */
	completedAsync = usb_ehciFetchCompleted(bus,&ehci->async);
	completedPeriodic = usb_ehciFetchCompleted(bus,&ehci->periodic);
	LIST_MERGE(&completedPeriodic,&completedAsync,urbs);
	return completedPeriodic;
}


static int usb_ehciUrbEnqueue(usb_bus_t* bus,usb_urb_t* urb)
{
	usb_ehciQtdList_t qtdList = LIST_HEAD_INITIALIZER;
	u8 type = usb_urbGetPipeType(urb);

	if(type != USB_EP_ISOC){
		if(usb_ehciCreateTransfer(urb,&qtdList) != EOK)
			return -ENOMEM;
		return usb_ehciEnqueueQh(bus,urb,&qtdList);
			/*
		if(type != USB_EP_INTR)
			return usb_ehciEnqueueAsync(bus,urb,&qtdList);
		else
			return usb_ehciEnqueueIntr(bus,urb,&qtdList);
			*/
	} else
		return -ENYI;
}


static inline void usb_ehciStopSchedule(usb_bus_t* bus,u8 type)
{
	int i = 0;
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	switch(type) {
		case USB_EP_CTRL:
		case USB_EP_BULK:
			ehci->oreg->USBCMD &= ~(u32)USB_USBCMD_ASE_MASK;
			while(ehci->oreg->USBSTS & USB_USBSTS_AS_MASK)
				if(i++ > 100000) break;
			break;
		case USB_EP_INTR:
			ehci->oreg->USBCMD &= ~(u32)USB_USBCMD_PSE_MASK;
			while(ehci->oreg->USBSTS & USB_USBSTS_PS_MASK)
				if(i++ > 100000) break;
			break;
	}
}


static inline void usb_ehciStartSchedule(usb_bus_t* bus,u8 type)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	switch(type) {
		case USB_EP_CTRL:
		case USB_EP_BULK:
			ehci->oreg->USBCMD |= USB_USBCMD_ASE_MASK;
			break;
		case USB_EP_INTR:
			ehci->oreg->USBCMD |= USB_USBCMD_PSE_MASK;
			break;
	}
}



static int _usb_ehciUrbDequeue(usb_bus_t* bus,usb_urb_t* urb)
{
	int retval = EOK;
	u8 type = usb_urbGetPipeType(urb);
	if(type != USB_EP_ISOC){
		usb_ehciQtd_t* qtd;
		usb_ehciQtd_t* nextQtd;
		usb_ehciQh_t* qh;
		u8 error = 0;
		u8 done = 1;

		usb_ehciStopSchedule(bus,type);
		qh = (usb_ehciQh_t*) usb_urbGetEp(urb)->hcpriv;
		LIST_FIND(&qh->qtdList,qtd,qtdList,qtd->urb == urb);
		if(qtd == NULL)
			return EOK;
		nextQtd = qtd;
		while(nextQtd->urb == urb) {
			// INVALIDATE nextQtd
			hal_cpuInvalCache(nextQtd,sizeof(*nextQtd));
			if(nextQtd->token & EHCI_QTD_ERROR_MASK)
				error = 1;
			if(nextQtd->token & EHCI_QTD_ACTIVE_MASK)
				done = 0;
			nextQtd = nextQtd->qtdList.next;
			if(nextQtd == qh->qtdList.first) {
				nextQtd = NULL;
				break;
			}
		}
		if(!error && done)
			retval = -EBUSY;
		else {
			if (qtd == qh->qtdList.first) {
				qh->currQtd = EHCI_LINK_INVALID;
				qh->nextQtd = nextQtd == NULL ? EHCI_LINK_INVALID : nextQtd->dmaAddr;
				qh->status = 0;
				qh->qtdList.first = nextQtd;
				hal_cpuFlushCache(qh,sizeof(*qh));
				if(nextQtd != NULL) {
					nextQtd->qtdList.prev->qtdList.next = NULL;
					nextQtd->qtdList.prev = qtd->qtdList.prev;
					nextQtd->qtdList.prev->qtdList.next = nextQtd;
				} else
					qtd->qtdList.prev->qtdList.next = NULL;
			} else {
				qtd->qtdList.prev->nextQtd = nextQtd->dmaAddr;
				qtd->qtdList.prev->qtdList.next = nextQtd;
				nextQtd->qtdList.prev->qtdList.next = NULL;
				nextQtd->qtdList.prev = qtd->qtdList.prev;
				// FLUSH qtd->qtdList.prev
				hal_cpuFlushCache(qtd->qtdList.prev,sizeof(*qtd));
			}
			do {
				usb_ehciQtd_t* tmp = qtd->qtdList.next;
				vm_dmapoolFree(&ehci_pool64,qtd);
				qtd = tmp;
			} while (qtd != NULL);
			_usb_unlinkUrb(urb);
		}
		usb_ehciStartSchedule(bus,type);
		return retval;
	} else
		return -ENYI;
}


static int usb_ehciUrbDequeue(usb_bus_t* bus,usb_urb_t* urb)
{
	int retval = 0;
	proc_mutexLock(&bus->urbLock);
	if(urb->urbs.next != NULL)
		retval = _usb_ehciUrbDequeue(bus,urb);
	proc_mutexUnlock(&bus->urbLock);
	return retval;
}


static int usb_ehciAllocBandwidth(usb_dev_t* dev,usb_ep_t* ep)
{
	return 0;
}


static void usb_ehciGetPortStatus(usb_bus_t* bus,u8 portNum,void* data)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	usb_hubPortStatus_t* status = (usb_hubPortStatus_t*) data;
	status->port = 0;
	status->change = 0;
	u32 portsc = ehci->oreg->PORTSC[portNum - 1];
	status->port |= (portsc & USB_PORTSC_CCS_MASK) |
		(portsc & USB_PORTSC_PSPD_MASK) >> USB_PORTSC_PSPD_SHIFT << USB_PORT_STATUS_SPEED_SHIFT;
}


static void usb_ehciResetPort(usb_bus_t* bus,u8 portNum)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	ehci->oreg->PORTSC[portNum - 1] |= USB_PORTSC_PR_MASK;
	while(ehci->oreg->PORTSC[portNum - 1] & USB_PORTSC_PR_MASK);
}


static void usb_ehciClearConnectChange(usb_bus_t* bus,u8 portNum)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	ehci->oreg->PORTSC[portNum - 1] |= USB_PORTSC_CSC_MASK;
}


static void usb_ehciUpdateCtrlEp(usb_dev_t* dev)
{
	usb_ehciQh_t* qh = (usb_ehciQh_t*) dev->ep[0]->hcpriv;
	usb_epDesc_t* ep = (usb_epDesc_t*) dev->ep[0]->desc;

	qh->epCap =  (qh->epCap & EHCI_QH_H_MASK)
		| EHCI_QH_MAX_PACKET(ep->wMaxPacketSize) 
		| EHCI_QH_EP_SPEED(dev->speed) 
		| EHCI_QH_EP_NUM(0)
		| EHCI_QH_DEV_ADDR(dev->addr)
		| EHCI_QH_DATA_TOGGLE_CTRL_MASK;
	if(dev->speed != USB_HIGH_SPEED)
		qh->epCap |= EHCI_QH_CTRL_EP_FLAG_MASK;
	qh->epCap2 = EHCI_QH_MULT((ep->wMaxPacketSize >> USB_EP_MULT_SHIFT) + 1);
	hal_cpuFlushCache(qh,sizeof(*qh));
}


static void usb_ehciDropEndpointPriv(usb_dev_t* dev,u8 num)
{
	if(dev->ep[num]->hcpriv == NULL)
		return;
	switch(dev->ep[num]->desc->bmAttributes & USB_EP_TYPE_MASK) {
		case USB_EP_CTRL:
		case USB_EP_BULK:
			usb_ehciUnlinkAsync(dev->parent->bus,dev->ep[num]->hcpriv);
			break;
		case USB_EP_INTR:
			usb_ehciUnlinkIntr(dev->parent->bus,dev->ep[num]->hcpriv);
			break;
		default:
			break;
	}
}




static usb_hcd_t usb_ehciHcd = {
	.irq = usb_ehciIrq,
	.scanTransfers = usb_ehciScanTransfers,
	.urbEnqueue = usb_ehciUrbEnqueue,
	.urbDequeue = usb_ehciUrbDequeue,
	.allocBandwidth = usb_ehciAllocBandwidth,
	.ops = {
		.getPortStatus = usb_ehciGetPortStatus,
		.resetPort = usb_ehciResetPort,
		.clearConnectChange = usb_ehciClearConnectChange,
		.updateCtrlEp = usb_ehciUpdateCtrlEp,
		.dropEndpointPriv = usb_ehciDropEndpointPriv,
	},
};


static struct {
	usb_cfgDesc_t cfgDesc;
	usb_ifDesc_t ifDesc;
	usb_epDesc_t epDesc;
} PACKED usb_ehciHubCfg	= {
	{
		.bLength = 9,
		.bDescriptorType = USB_DESC_CFG,
		.wTotalLength = 40,
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0,
		.bMaxPower = 0,
	},
	{
		.bLength = 9,
		.bDescriptorType = USB_DESC_IF,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_HUB,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	{
		.bLength = 7,
		.bDescriptorType = USB_DESC_EP,
		.bEndpointAddress = USB_DIR_IN | 1,
		.bmAttributes = USB_EP_INTR,
		.wMaxPacketSize = USB_HUB_DATA_LEN,
		.bInterval = 0xFF,
	},
};


usb_dev_t *usb_ehciInitHost(usb_hcDesc_t *hd)
{
	/* Create usb device structure with descriptors and pass it
	 * to hub driver thread - probe() */
	int i,rc;
	usb_ehciDesc_t* ed = &hd->ehciDesc;
	usb_bus_t* retval = NULL;
	usb_ehciCtx_t* ctx = NULL;

	if (EHCI_QH_ALIGN != 64 || EHCI_QTD_ALIGN != 64 || 
			sizeof(usb_ehciQh_t) > 64 || sizeof(usb_ehciQtd_t) > 64 ||
			sizeof(usb_ehciItd_t) > 64){
		main_printf(ATTR_ERROR,"usb: [ehciInitHost] invalid initialization of ehci structures pool\n");
		return NULL;
	}

	if (ehci_pool64.size == 0)
		if((rc = vm_dmapoolCreate(&ehci_pool64, 64, 64, 1)) < 0)
			return NULL;

	retval = vm_kmalloc(sizeof(*retval));
	if (retval != NULL)
		ctx = vm_kmalloc(sizeof(*ctx));

	rc = -1;
	if (ctx != NULL) {

		ctx->page = NULL;
		ctx->periodicQueue = NULL;

		rc = vm_iomap(ed->baseAddr, ed->regSectionSize, PGHD_DEV_RW, &ctx->ioVaddr);
	}

	if (rc == EOK)
		ctx->page = vm_pageAlloc(1,vm_pageAlloc);

	if(ctx->page == NULL)
		rc = -ENOMEM;
	else
	   	rc = vm_kmap(ctx->page,PGHD_WRITE | PGHD_PRESENT,(void**) &ctx->periodicQueue);
	
	if (rc == EOK)
		ctx->itd = vm_dmapoolAlloc(&ehci_pool64,&ctx->itdAddr);

	if(ctx->itd == NULL)
		rc = -ENOMEM;
	else
		rc = usb_initBus(retval,0x20,(usb_cfgDesc_t*) &usb_ehciHubCfg,hd->portNum,&usb_ehciHcd,ctx);

	if(rc != EOK){
		if(ctx != NULL){
			if(ctx->itd != NULL) vm_dmapoolFree(&ehci_pool64,ctx->itd);
			if(ctx->periodicQueue != NULL) vm_kunmap((void*)ctx->periodicQueue);
			if(ctx->page != NULL) vm_pageFree(ctx->page);
			if(ctx->ioVaddr != NULL) vm_iounmap(ctx->ioVaddr,ed->regSectionSize);
			vm_kfree(ctx);
		}
		if(retval != NULL)
			vm_kfree(retval);
		return NULL;
	}

	ctx->creg = ctx->ioVaddr + ed->capRegOffset;
	ctx->oreg = ctx->ioVaddr + ed->opRegOffset;
	ctx->mode = ed->modeRegOffset != 0 ? ctx->ioVaddr + ed->modeRegOffset : NULL;
	LIST_HEAD_INIT(&ctx->async);
	LIST_HEAD_INIT(&ctx->periodic);

	/* Change while for function waiting with timeout */
	ctx->oreg->USBCMD &= ~USB_USBCMD_RS_MASK;
	
	while (ctx->oreg->USBCMD & USB_USBCMD_RS_MASK);
	  ctx->oreg->USBCMD |= USB_USBCMD_RST_MASK;
  
  while (ctx->oreg->USBCMD & USB_USBCMD_RST_MASK);
	if (ctx->mode != NULL)
		do {
			*ctx->mode = (*ctx->mode & ~USB_USBMODE_CM_MASK) | (USB_USBMODE_CM_MASK & 0x03);
		} while((*ctx->mode & USB_USBMODE_CM_MASK) != 0x03);

    ctx->oreg->USBCMD = (ctx->oreg->USBCMD & ~(USB_USBCMD_ITC_MASK | USB_USBCMD_FS1_MASK)) | USB_USBCMD_ITC(0);
	{ /* Set periodic schedule length */
		u8 tmp = 0;
		switch (ed->periodicScheduleLength) {
			case 1024:
				tmp = 0;
				break;
			case 512:
				tmp = 1;
				break;
			case 256:
				tmp = 2;
				break;
			case 128:
				tmp = 3;
				break;
			default:
				panic("usb: invalid periodic schedule length\n");
		}
		ctx->oreg->USBCMD |= USB_USBCMD_FS1(tmp);
	}
	memset(ctx->itd,0,sizeof(*ctx->itd));
	ctx->itd->nextLink = EHCI_LINK_INVALID;
	// FLUSH ctx->itd
	hal_cpuFlushCache(ctx->itd,sizeof(*ctx->itd));
	for(i=0;i<ed->periodicScheduleLength;i++){
		ctx->periodicQueue[i]=ctx->itdAddr;
	}
    if(ctx->creg->HCSPARAMS & USB_HCSPARAMS_PPC_MASK)
		for(i = 0;i < hd->portNum;i++)
			ctx->oreg->PORTSC[i] |= USB_PORTSC_PP_MASK;


	ctx->oreg->ASYNCLISTADDR = EHCI_LINK_INVALID;
	ctx->oreg->PERIODICLISTBASE = ctx->page->addr;

    hal_interruptsSetHandler(ed->irqNum,usb_ehciHcd.irq,retval);
    ctx->oreg->USBINTR |= USB_USBINTR_PCE_MASK | USB_USBINTR_SEE_MASK | USB_USBINTR_UE_MASK | USB_USBINTR_UEE_MASK;

    ctx->oreg->USBCMD |= USB_USBCMD_RS_MASK;

	return &retval->rootHub.dev;
}


static usb_ehciQtd_t* usb_ehciQtdAlloc(void)
{
	addr_t dmaAddr;
	usb_ehciQtd_t* retval = vm_dmapoolAlloc(&ehci_pool64,&dmaAddr);
	if(retval != NULL){
		retval->dmaAddr = dmaAddr;
		retval->token = EHCI_QTD_ACTIVE_MASK;
	}
	return retval;
}


static int usb_ehciQtdFillBuffers(usb_ehciQtd_t* qtd,u8* data,u16 len)
{
    int i=0;
    u8* end=data+len;
	memset((void*)&qtd->buff[0],0,sizeof(qtd->buff));
	if(data == NULL)
		return 0;
    do {
        if(i>4)
            return -1;
        vm_kmapResolve(data,(u32*)&qtd->buff[i]);
        i++;
        data=(u8*) PTR_ALIGN_DOWN(PTR_ADD(data,SIZE_PAGE),SIZE_PAGE);
    } while (data<end);
	qtd->token |= len << EHCI_QTD_BYTES_TO_TRANSFER_SHIFT;
	/* Set bytes to transfer in token */
    return 0;
}



static void usb_ehciQtdListAdd(usb_ehciQtdList_t* qtdList,usb_ehciQtd_t* qtd)
{
	LIST_ADD(qtdList,qtd,qtdList);
	qtd->qtdList.prev->nextQtd = qtd->dmaAddr;
	// FLUSH qtd->qtdList.prev
	hal_cpuFlushCache(qtd->qtdList.prev,sizeof(*qtd));
	qtd->nextQtd = EHCI_LINK_INVALID;
	qtd->altQtd = EHCI_LINK_INVALID;
}


int usb_ehciCreateTransfer(usb_urb_t* urb,usb_ehciQtdList_t* qtdList)
{
	usb_ehciQtd_t* qtd = NULL;
	u16 maxLen, chunkLen;
	u32 tokenTmp;
	size_t processed = 0;
	u8* dataBuffer = usb_ehciSetupTransferBuffer(urb);
	if(dataBuffer == NULL)
		return -ENOMEM;
	if(usb_urbGetPipeType(urb) == USB_EP_CTRL) { /* setup phase */
		qtd = usb_ehciQtdAlloc();
		if(qtd == NULL){
			usb_ehciFreeTransferBuffer(urb);
			while(!LIST_IS_EMPTY(qtdList))
				LIST_REMOVE(qtdList,qtdList->first->qtdList.prev,qtdList);
			/* Free whole qtdList */
			return -ENOMEM;
		}
		usb_ehciQtdListAdd(qtdList,qtd);
		usb_ehciQtdFillBuffers(qtd,dataBuffer,urb->setupLen);
		dataBuffer += urb->setupLen;
		qtd->token |= EHCI_QTD_PID(EHCI_PID_SETUP);
		qtd->urb = urb;
		// FLUSH qtd
		hal_cpuFlushCache(qtd,sizeof(*qtd));
	}
	maxLen = usb_urbGetEp(urb)->desc->wMaxPacketSize;
	/* Data toggle bit only relevant for ctrl transfers, because for bulk and interrupt
	 * qh is tracking toggle bit */
	tokenTmp = EHCI_QTD_PID(usb_urbGetPipeDir(urb) == USB_DIR_IN ? EHCI_PID_IN : EHCI_PID_OUT) |
		EHCI_QTD_DATA_TOGGLE_MASK; 
	while(processed < urb->dataLen) {
		qtd = usb_ehciQtdAlloc();
		if(qtd == NULL){
			usb_ehciFreeTransferBuffer(urb);
			/* Free whole qtdList */
			while(!LIST_IS_EMPTY(qtdList))
				LIST_REMOVE(qtdList,qtdList->first->qtdList.prev,qtdList);
			return -ENOMEM;
		}
		usb_ehciQtdListAdd(qtdList,qtd);
		chunkLen = maxLen < urb->dataLen - processed ? maxLen : urb->dataLen - processed;
		usb_ehciQtdFillBuffers(qtd,dataBuffer+processed,chunkLen);
		qtd->token |= tokenTmp;
		qtd->urb = urb;
		// FLUSH qtd
		hal_cpuFlushCache(qtd,sizeof(*qtd));
		tokenTmp ^= EHCI_QTD_DATA_TOGGLE_MASK;
		processed += chunkLen;
	}	
	if(usb_urbGetPipeType(urb) == USB_EP_CTRL) { /* status phase */
		qtd = usb_ehciQtdAlloc();
		if(qtd == NULL) {
			usb_ehciFreeTransferBuffer(urb);
			/* Free whole qtdList */
			while(!LIST_IS_EMPTY(qtdList))
				LIST_REMOVE(qtdList,qtdList->first->qtdList.prev,qtdList);
			return -ENOMEM;
		}
		usb_ehciQtdListAdd(qtdList,qtd);
		usb_ehciQtdFillBuffers(qtd,NULL,0);
		qtd->urb = urb;
		tokenTmp ^= EHCI_QTD_PID(EHCI_PID_IN); /* reverse direction */ 
		qtd->token |= tokenTmp | EHCI_QTD_DATA_TOGGLE_MASK;
	}
	if(qtd != NULL)
		qtd->token |= EHCI_QTD_IOC_MASK;
	// FLUSH qtd
	hal_cpuFlushCache(qtd,sizeof(*qtd));
	return 0;
}


static usb_ehciQh_t* usb_ehciMakeQh(usb_urb_t* urb)
{
	addr_t dmaAddr;
	usb_ehciQh_t* retval = NULL;
	usb_ep_t* ep = usb_urbGetEp(urb);
	retval = vm_dmapoolAlloc(&ehci_pool64,&dmaAddr);
	retval->dmaAddr = dmaAddr;
	retval->epCap = EHCI_QH_MAX_PACKET(ep->desc->wMaxPacketSize) 
		| EHCI_QH_EP_SPEED(urb->dev->speed) 
		| EHCI_QH_EP_NUM(usb_urbGetEpNum(urb))
		| EHCI_QH_DEV_ADDR(urb->dev->addr);
	if(usb_urbGetPipeType(urb) == USB_EP_CTRL){
		retval->epCap |= EHCI_QH_DATA_TOGGLE_CTRL_MASK;
		if(urb->dev->speed != USB_HIGH_SPEED)
			retval->epCap |= EHCI_QH_CTRL_EP_FLAG_MASK;
	}
	retval->epCap2 = EHCI_QH_MULT((ep->desc->wMaxPacketSize >> USB_EP_MULT_SHIFT) + 1);
	if(usb_urbGetPipeType(urb) == USB_EP_INTR)
		retval->epCap2 |= EHCI_QH_S_MASK | EHCI_QH_C_MASK;
	retval->nextQtd = EHCI_LINK_INVALID;
	retval->altQtd = EHCI_LINK_INVALID;
	retval->currQtd = EHCI_LINK_INVALID;
	retval->status = 0;
	LIST_HEAD_INIT(&retval->qtdList);
		
	return retval;
}

static void usb_ehciLinkAsync(usb_bus_t* bus,usb_ehciQh_t* qh)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	proc_mutexLock(&bus->urbLock);
	LIST_ADD(&ehci->async,qh,qhList);
	qh->hlink=qh->qhList.next->dmaAddr;
	// FLUSH qh
	hal_cpuFlushCache(qh,sizeof(*qh));
	qh->qhList.prev->hlink=qh->dmaAddr;
	// FLUSH qh->qhList.prev
	hal_cpuFlushCache(qh->qhList.prev,sizeof(*qh));
	if(ehci->async.first == qh) { /* First qh in async schedule */
		qh->epCap |= EHCI_QH_H_MASK;
		// FLUSH qh
		hal_cpuFlushCache(qh,sizeof(*qh));
		ehci->oreg->ASYNCLISTADDR = qh->dmaAddr;
		ehci->oreg->USBCMD |= USB_USBCMD_ASE_MASK;
	}
	proc_mutexUnlock(&bus->urbLock);
}


static void usb_ehciUnlinkAsync(usb_bus_t* bus,usb_ehciQh_t* qh)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	proc_mutexLock(&bus->urbLock);
	if(qh->epCap & EHCI_QH_H_MASK) {
		qh->qhList.next->epCap |= EHCI_QH_H_MASK;
		// FLUSH qh->qhList.next
		hal_cpuFlushCache(qh->qhList.next,sizeof(*qh));
	}
	qh->qhList.prev->hlink = qh->hlink;
	// FLUSH qh->qhList.prev
	hal_cpuFlushCache(qh->qhList.prev,sizeof(*qh));
	LIST_REMOVE(&ehci->async,qh,qhList);
	if(LIST_IS_EMPTY(&ehci->async))
		ehci->oreg->USBCMD &= ~USB_USBCMD_ASE_MASK;
	proc_mutexUnlock(&bus->urbLock);
}


static void usb_ehciLinkIntr(usb_bus_t* bus,usb_ehciQh_t* qh)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;

	proc_mutexLock(&bus->urbLock);
	LIST_ADD(&ehci->periodic,qh,qhList);
	qh->hlink = EHCI_LINK_INVALID;

	// FLUSH qh
	hal_cpuFlushCache(qh,sizeof(*qh));

	if(ehci->periodic.first == qh) { /* First qh in async schedule */
		ehci->itd->nextLink = qh->dmaAddr | EHCI_QH_TYPE(EHCI_QH_TYPE_QH);
		// FLUSH ehci->itd
		hal_cpuFlushCache(ehci->itd,sizeof(*ehci->itd));
		ehci->oreg->USBCMD |= USB_USBCMD_PSE_MASK;
	} else {
		qh->qhList.prev->hlink=qh->dmaAddr;
		// FLUSH qh->qhList.prev
		hal_cpuFlushCache(qh->qhList.prev,sizeof(*qh));
	}
	proc_mutexUnlock(&bus->urbLock);
}


static void usb_ehciUnlinkIntr(usb_bus_t* bus,usb_ehciQh_t* qh)
{
	usb_ehciCtx_t* ehci = (usb_ehciCtx_t*) bus->hcdCtx;
	proc_mutexLock(&bus->urbLock);
	LIST_REMOVE(&ehci->periodic,qh,qhList);
	if(LIST_IS_EMPTY(&ehci->periodic)){
		ehci->oreg->USBCMD &= ~USB_USBCMD_PSE_MASK;
		ehci->itd->nextLink = EHCI_LINK_INVALID;
		// FLUSH ehci->itd
		hal_cpuFlushCache(ehci->itd,sizeof(*ehci->itd));
	} else {
		qh->qhList.prev->hlink = qh->hlink;
		// FLUSH qh->qhList.prev
		hal_cpuFlushCache(qh->qhList.prev,sizeof(*qh));
	}
	proc_mutexUnlock(&bus->urbLock);
}


static void usb_ehciQhAppendTransfer(usb_bus_t* bus,usb_ehciQh_t* qh,usb_ehciQtdList_t* qtdList)
{
	proc_mutexLock(&bus->urbLock);
	if(!LIST_IS_EMPTY(&qh->qtdList)) {
		qh->qtdList.first->qtdList.prev->nextQtd=qtdList->first->dmaAddr;
		// FLUSH qh->qtdLIst.first->qtdList.prev
		hal_cpuFlushCache(qh->qtdList.first->qtdList.prev,sizeof(*qh->qtdList.first->qtdList.prev));
	} else {
		qh->nextQtd = qtdList->first->dmaAddr;
		//hal_cpuFlushCache(qh,sizeof(*qh));
	}
	LIST_MERGE(&qh->qtdList,qtdList,qtdList);
	// FLUSH qh
	hal_cpuFlushCache(qh,sizeof(*qh));
	proc_mutexUnlock(&bus->urbLock);
}


static int usb_ehciEnqueueQh(usb_bus_t* bus,usb_urb_t* urb, usb_ehciQtdList_t* qtdList)
{
	usb_ep_t* ep = usb_urbGetEp(urb);	
	usb_ehciQh_t* qh = ep->hcpriv;
	if(qh == NULL) {
		if((ep->hcpriv = qh = usb_ehciMakeQh(urb)) == NULL)
				return -ENOMEM;
		else {
			if(usb_urbGetPipeType(urb) == USB_EP_INTR)
				usb_ehciLinkIntr(bus,qh);
			else
				usb_ehciLinkAsync(bus,qh);
		}
	}
	usb_linkUrb(urb);
	usb_ehciQhAppendTransfer(bus,qh,qtdList);
	return 0;
}


    /* After ehci_init_pipe p_addr has qh physical address * /
    qh->op_reg=(EHCI_Op_Reg_t*)(port->base+BSP_EHCI_OP_REG_OFFSET);
    qh->h_link=qh->p_addr; / * One element circular list * /
    qh->p_addr=(u32)qh; / * Previous (so self) virtual address * /
    qh->ep_cap=EHCI_QH_MAX_PACKET(8)|EHCI_QH_EP_SPEED((qh->op_reg->PORTSC&USB_PORTSC_PSPD_MASK)>>USB_PORTSC_PSPD_SHIFT)|EHCI_QH_H_MASK|EHCI_QH_DATA_TOGGLE_CTRL_MASK; / * Ep and dev addr 0 * /
    qh->ep_cap2=EHCI_QH_MULT(1);
    qh->status=0;
    qh->alt_qtd=EHCI_LINK_INVALID;

    / * TODO SYNC: Check if ASYNCLISTADDR is 0 if not than error, otherwise set and release spinlock * /
    if(qh->op_reg->ASYNCLISTADDR){
        hal_spinlockClear(&port->lock);
        main_printf(ATTR_ERROR,"hcd_on_insert error USB%d, ASYNCLISTADDR already initialized\n",port->num);
        return -USB_ERR_ALREADY_INITIALIZED;
    }
    qh->op_reg->USBCMD &= ~USB_USBCMD_ASE_MASK;
    while(qh->op_reg->USBSTS & USB_USBSTS_AS_MASK);
    qh->op_reg->ASYNCLISTADDR=qh->h_link;
    hal_spinlockClear(&port->lock);

    *ctrl_pipe=qh;

    return 0;

}

int hcd_transmit(ehci_qh_t* qh,u32 flags,u8* b,uint16_t size)
{
    ehci_qtd_t* qtd;

    if(!qh->op_reg->PORTSC & USB_PORTSC_CCS_MASK)
        return -USB_ERR_NO_CONNECTION;

    qtd=(ehci_qtd_t*)(qh->next_qtd & ~(u32)EHCI_LINK_INVALID);

    qtd->token=(size<<EHCI_QTD_BYTES_TO_TRANSFER_SHIFT)|((flags&EHCI_TR_FLAGS_PID_MASK)<<EHCI_QTD_PID_SHIFT)|EHCI_QTD_IOC_MASK|EHCI_QTD_ACTIVE_STATUS_MASK|((flags&EHCI_TR_FLAGS_DT_MASK)?EHCI_QTD_DATA_TOGGLE_MASK:0);
    ehci_qtd_set_buffers(qtd,b,size);
    qh->op_reg->USBSTS |= USB_USBSTS_UI_MASK;
    qh->next_qtd = qtd->ph_addr;

    qh->op_reg->USBCMD |= USB_USBCMD_ASE_MASK;
    while(!(qh->op_reg->USBSTS&USB_USBSTS_UI_MASK));

    qh->op_reg->USBCMD &= ~USB_USBCMD_ASE_MASK;
    qh->op_reg->USBSTS |= USB_USBSTS_UI_MASK;
    if(qh->op_reg->USBSTS&USB_USBSTS_UEI_MASK){
        qh->op_reg->USBSTS |= USB_USBSTS_UEI_MASK;
        main_printf(ATTR_ERROR,"ehci_transmit error: %x",qtd->token&EHCI_QTD_STATUS_MASK);
    }
    / * Return actual number of bytes transferred * /
    return size-((qtd->token&EHCI_QTD_BYTES_TO_TRANSFER_MASK)>>EHCI_QTD_BYTES_TO_TRANSFER_SHIFT);
}

int hcd_rt_transmit(ehci_qh_t* qh,u32 flags,u8* b,uint16_t size)
{
	ehci_qtd_t* qtd;

    if(!qh->op_reg->PORTSC & USB_PORTSC_CCS_MASK)
        return -USB_ERR_NO_CONNECTION;

    qtd=(ehci_qtd_t*)(qh->next_qtd & ~(u32)EHCI_LINK_INVALID);

	qtd->token=(size<<EHCI_QTD_BYTES_TO_TRANSFER_SHIFT)|((flags&EHCI_TR_FLAGS_PID_MASK)<<EHCI_QTD_PID_SHIFT)|EHCI_QTD_IOC_MASK|EHCI_QTD_ACTIVE_STATUS_MASK|((flags&EHCI_TR_FLAGS_DT_MASK)?EHCI_QTD_DATA_TOGGLE_MASK:0);
    ehci_qtd_set_buffers(qtd,b,size);
    qh->op_reg->USBSTS |= USB_USBSTS_UI_MASK;
    qh->next_qtd = qtd->ph_addr;

	qh->op_reg->USBCMD |= USB_USBCMD_PSE_MASK;
    while(!(qh->op_reg->USBSTS&USB_USBSTS_UI_MASK));

    qh->op_reg->USBCMD &= ~USB_USBCMD_PSE_MASK;
    qh->op_reg->USBSTS |= USB_USBSTS_UI_MASK;
	if(qh->op_reg->USBSTS&USB_USBSTS_UEI_MASK){
        qh->op_reg->USBSTS |= USB_USBSTS_UEI_MASK;
        main_printf(ATTR_ERROR,"ehci_transmit error: %x",qtd->token&EHCI_QTD_STATUS_MASK);
    }
    return size-((qtd->token&EHCI_QTD_BYTES_TO_TRANSFER_MASK)>>EHCI_QTD_BYTES_TO_TRANSFER_SHIFT);
}

void hcd_update_address(ehci_qh_t* qh,u8 addr)
{
    qh->ep_cap=(qh->ep_cap&~EHCI_QH_DEV_ADDR_MASK)|(addr<<EHCI_QH_DEV_ADDR_SHIFT);
}

void hcd_update_max_packet_size(ehci_qh_t* qh,u8 size)
{
    qh->ep_cap=(qh->ep_cap & ~EHCI_QH_MAX_PACKET_LEN_MASK)|((u32)size)<<EHCI_QH_MAX_PACKET_LEN_SHIFT;
}

size_t hcd_get_max_packet_size(ehci_qh_t* qh)
{
    return (qh->ep_cap & EHCI_QH_MAX_PACKET_LEN_MASK)>>EHCI_QH_MAX_PACKET_LEN_SHIFT;
}

int hcd_replace_pipe(hcd_pipe_t* prev,hcd_pipe_t* qh,size_t size,u8 pipe_type)
{
    u32 as_active=0;
    size_t qtd_n=get_pipe_qtd_n(pipe_type);

    if(ehci_init_pipe(qh,size,qtd_n))
        return -USB_ERR_NOMEM;

    qh->alt_qtd=EHCI_LINK_INVALID;
    qh->status=0;
    qh->op_reg=prev->op_reg;
    qh->ep_cap=prev->ep_cap; / * H bit is inherited as well * /
    qh->ep_cap2=prev->ep_cap2;

    / * Switch off async schedule * /
    if((as_active=qh->op_reg->USBSTS & USB_USBSTS_AS_MASK)){
        qh->op_reg->USBCMD &= ~USB_USBCMD_ASE_MASK;
        while(qh->op_reg->USBSTS & USB_USBSTS_AS_MASK);
    }
    if(qh->op_reg->ASYNCLISTADDR==((hcd_pipe_t*)prev->p_addr)->h_link){
        qh->op_reg->ASYNCLISTADDR=qh->p_addr;
    }

    if(prev!=(hcd_pipe_t*)prev->p_addr){
        ehci_qh_t* next;
        / * Find next by traversing the list
         * ok, because list is very short * /
        for(next=prev;((hcd_pipe_t*)next->p_addr)!=prev;next=(hcd_pipe_t*) prev->p_addr);
        next->p_addr=(u32)qh;
        qh->h_link=prev->h_link;
        ((hcd_pipe_t*)prev->p_addr)->h_link=qh->p_addr;
        qh->p_addr=(u32)prev->p_addr;
    } else {
        qh->h_link=qh->p_addr|EHCI_QH_TYPE(EHCI_QH_TYPE_QH);
        qh->p_addr=(u32)qh;
        / * Set register * /
        qh->op_reg->ASYNCLISTADDR=qh->h_link;
    }

    if(as_active)
        qh->op_reg->USBCMD |= USB_USBCMD_ASE_MASK;

    return 0;
}

int hcd_init_ep_pipe(hcd_pipe_t* qh,size_t size,usb_ep_desc_t* ep,hcd_pipe_t* ctrl_pipe)
{
	int rc;
    if((rc=ehci_init_pipe(qh,size,get_pipe_qtd_n(ep->bmAttributes&USB_EP_DESC_TYPE_MASK))))
        return rc;

    qh->alt_qtd=EHCI_LINK_INVALID;
    qh->status=0;
    qh->op_reg=ctrl_pipe->op_reg;
    qh->ep_cap=(ctrl_pipe->ep_cap&(EHCI_QH_EP_SPEED_MASK|EHCI_QH_DEV_ADDR_MASK))|
        EHCI_QH_EP_NUM(ep->bEndpointAddress&USB_EP_DESC_NUM_MASK)|
        EHCI_QH_MAX_PACKET(ep->wMaxPacketSize&USB_EP_DESC_MAX_PACKET_MASK);
    qh->ep_cap2=(u32)(((ep->wMaxPacketSize&USB_EP_DESC_MULT_MASK)>>USB_EP_DESC_MULT_SHIFT)+1)<<EHCI_QH_MULT_SHIFT;

    if(ctrl_pipe!=(hcd_pipe_t*)ctrl_pipe->p_addr){
        ehci_qh_t* next;
        / * Find next by traversing the list
         * ok, because list is very short * /
        for(next=ctrl_pipe;((hcd_pipe_t*)next->p_addr)!=ctrl_pipe;next=(hcd_pipe_t*) ctrl_pipe->p_addr);
        next->p_addr=(u32)qh;
        qh->h_link=ctrl_pipe->h_link;
        ctrl_pipe->h_link=qh->p_addr;
        qh->p_addr=(u32)ctrl_pipe;
    } else {
        qh->h_link=ctrl_pipe->h_link;
        ctrl_pipe->h_link=qh->p_addr;
        qh->p_addr=(u32)ctrl_pipe;
        ctrl_pipe->p_addr=(u32)qh;
    }
    return 0;
}

int hcd_init_periodic_pipe(hcd_pipe_t* qh,size_t size,usb_ep_desc_t* ep,hcd_pipe_t* ctrl_pipe)
{
    int i,rc;
	ehci_itd_t* itd;

	size_t qtd_n=get_pipe_qtd_n(ep->bmAttributes&USB_EP_DESC_TYPE_MASK);
    if((rc=ehci_init_pipe(qh,size,qtd_n)))
        return rc;
	itd=(ehci_itd_t*)PTR_ADD(qh,sizeof(*qh)+qtd_n*sizeof(ehci_qtd_t));
	if((void*)PTR_ADD(itd,sizeof(*itd))>=(void*)PTR_ADD(qh,size))
		return -USB_ERR_NOMEM;
	memset(itd,0,sizeof(*itd));

	itd->next_link=qh->p_addr|EHCI_QH_TYPE(EHCI_QH_TYPE_QH);

	qh->h_link=EHCI_LINK_INVALID;
    qh->alt_qtd=EHCI_LINK_INVALID;
    qh->status=0;
    qh->op_reg=ctrl_pipe->op_reg;
    qh->ep_cap=(ctrl_pipe->ep_cap&(EHCI_QH_EP_SPEED_MASK|EHCI_QH_DEV_ADDR_MASK))|
        EHCI_QH_EP_NUM(ep->bEndpointAddress&USB_EP_DESC_NUM_MASK)|
        EHCI_QH_MAX_PACKET(ep->wMaxPacketSize&USB_EP_DESC_MAX_PACKET_MASK);
    qh->ep_cap2=(u32)(((ep->wMaxPacketSize&USB_EP_DESC_MULT_MASK)>>USB_EP_DESC_MULT_SHIFT)+1)<<EHCI_QH_MULT_SHIFT|
		EHCI_QH_S_MASK|EHCI_QH_C_MASK;

   	for(i=0;i<PERIODIC_QUEUE_LEN;i++){
		if(periodic_queue[i]&EHCI_LINK_INVALID){
			//periodic_queue[i]=(qh->p_addr+(sizeof(*qh)+qtd_n*sizeof(ehci_qtd_t)));
			periodic_queue[i]=qh->p_addr|EHCI_QH_TYPE(EHCI_QH_TYPE_QH);
			qh->p_addr=(u32)&periodic_queue[i];
			return 0;
		}
	}

	return -USB_ERR_NOMEM;
}
*/
