/**
 * USB urb helper functions
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file urb.h
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author: Pawel Tryfon <pawel.tryfon@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _USB_URB_H
#define _USB_URB_H

#include "usb.h"
#include "hcd.h"
#include "hub.h"

#include <include/errno.h>


#define USB_PIPE_TYPE(x) (((x) & 0x3) << 4)
#define USB_EP_ADDR_MASK 0x0F

static inline void usb_fillUrb(usb_urb_t* urb,usb_dev_t* dev,u8 pipe,void* data,u16 dataLen,void (*callback)(usb_urb_t*))
{
	urb->dev = dev;
	urb->pipe = pipe;
	urb->data = data;
	urb->dataLen = dataLen;
	urb->callback = callback;
	urb->status = 0;
	urb->transferred = 0;
	LIST_ENTRY_INIT(&urb->urbs);
}


static inline u8 usb_urbGetEpNum(usb_urb_t* urb)
{
	return urb->pipe & USB_EP_NUM_MASK;
}


static inline u8 usb_urbGetPipeType(usb_urb_t* urb)
{
	return (urb->pipe >> 4) & USB_EP_TYPE_MASK;
}


static inline u8 usb_urbGetPipeDir(usb_urb_t* urb)
{
	return urb->pipe & USB_EP_DIR_MASK;
}


static inline usb_ep_t* usb_urbGetEp(usb_urb_t* urb)
{
	return urb->dev->ep[usb_urbGetEpNum(urb)];
};

static inline void _usb_linkUrb(usb_urb_t* urb)
{
	LIST_ADD(&urb->dev->ep[usb_urbGetEpNum(urb)]->urbs,urb,urbs);
}


static inline void usb_linkUrb(usb_urb_t* urb)
{
	proc_mutexLock(&urb->dev->parent->bus->urbLock);
	_usb_linkUrb(urb);
	proc_mutexUnlock(&urb->dev->parent->bus->urbLock);
}


static inline void _usb_unlinkUrb(usb_urb_t* urb)
{
	LIST_REMOVE(&urb->dev->ep[usb_urbGetEpNum(urb)]->urbs,urb,urbs);
}

static inline void usb_unlinkUrb(usb_urb_t* urb)
{
	proc_mutexLock(&urb->dev->parent->bus->urbLock);
	_usb_unlinkUrb(urb);
	proc_mutexUnlock(&urb->dev->parent->bus->urbLock);
}


static inline int usb_dequeueUrb(usb_urb_t* urb)
{
	int retval = 0;
	proc_mutexLock(&urb->dev->parent->bus->urbLock);
	if(urb->urbs.next != NULL) {
		if(!(urb->status & USB_URB_IN_COMPLETION))
			_usb_unlinkUrb(urb);
		else
			retval = -EBUSY;
	}
	proc_mutexUnlock(&urb->dev->parent->bus->urbLock);
	return retval;
}


static inline int usb_urbError(usb_urb_t* urb,u32 errno)
{
	urb->status |= USB_URB_ERROR | (errno << USB_URB_ERRNO_SHIFT);
	return -errno;
}


static inline int usb_urbReturnStatus(usb_urb_t* urb)
{
	return -(urb->status >> USB_URB_ERRNO_SHIFT);
}




#endif
