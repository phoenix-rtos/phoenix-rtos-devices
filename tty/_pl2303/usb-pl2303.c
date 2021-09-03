#include <include/errno.h>
#include <lib/list.h>
#include <lib/ptr.h>
#include <fs/if.h>
#include <dev/if.h>
#include <main/if.h>
#include <vm/if.h>

#include <usb/usb.h>
#include <usb/usb_spec.h>
#include <usb/urb.h>

#define SERIAL_SET_LINE_CODING_REQUEST 0x20
#define SERIAL_SET_LINE_CODING_REQUEST_LEN 7
#define SERIAL_DEFAULT_BAUD_RATE 115200
#define SERIAL_SET_CONTROL_LINE_STATE_REQUEST 0x22


typedef struct usb_serialDev_ {
	usb_dev_t* dev;
	usb_epDesc_t* epIn;
	usb_epDesc_t* epOut;
	usb_epDesc_t* epIntr;
	mutex_t mutex;
	char b[64];
	u8 top;
	u8 bLen;
} usb_serialDev_t;


#define SERIAL_MAX_DEV_N 8
static struct {
	usb_serialDev_t* devices[SERIAL_MAX_DEV_N];
} serial_g;


typedef struct usb_serialLineCoding_{
    u32 dWDTERate; /* in bits/sec */
    u8  bCharFormat; /* 0 -> 1 stop bit, 1 -> 1.5 stop bits, 2 -> 2 stop bits */
    u8  bParityType; /* 0 -> None, 1 -> Odd, 2 -> Even, 3 -> Mark, 4 -> Space */
    u8  bDataBits;   /* 5,6,7,8 or 16 */
} usb_serialLineCoding_t;


static int usb_serialSetLineCoding(usb_dev_t* dev)
{
    usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_CLASS_IF,
    	.bRequest = SERIAL_SET_LINE_CODING_REQUEST,
		.wValue = 0,
		.wIndex = 0,
		.wLength = SERIAL_SET_LINE_CODING_REQUEST_LEN,
	};
    usb_serialLineCoding_t coding = {
		.dWDTERate = SERIAL_DEFAULT_BAUD_RATE,
		.bCharFormat = 0,
		.bParityType = 0,
		.bDataBits = 8,
	};

	return usb_controlTransfer(dev,0,&req,(u8*)&coding,SERIAL_SET_LINE_CODING_REQUEST_LEN,USB_TIMEOUT);
}

static int usb_serialSetControlLineState(usb_dev_t* dev)
{
    usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_CLASS_IF,
    	.bRequest = SERIAL_SET_CONTROL_LINE_STATE_REQUEST,
		.wValue = 3,
		.wIndex = 0,
		.wLength = 0,
	};

	return usb_controlTransfer(dev,0,&req,NULL,0,USB_TIMEOUT);
}


static int usb_pl2303Init(usb_dev_t* dev)
{
	int rc;
    usb_ctrlreq_t vread,vwrite;
    u8 buf[4];

    vread.bmRequestType=USB_READ_VENDOR_DEV;
    vwrite.bmRequestType=USB_WRITE_VENDOR_DEV;
    vread.bRequest=vwrite.bRequest=0x01;
    vread.wValue=0x8484;
    vwrite.wValue=0x0404;
    vread.wIndex=vwrite.wIndex=0;
    vread.wLength=1;
    vwrite.wLength=0;

    if((rc=usb_controlTransfer(dev,0,&vread,buf,1,USB_TIMEOUT))<0)
        return rc;
    if((rc=usb_controlTransfer(dev,0,&vwrite,NULL,0,USB_TIMEOUT))<0)
        return rc;
    if((rc=usb_controlTransfer(dev,0,&vread,buf,1,USB_TIMEOUT))<0)
        return rc;
    vread.wValue=0x8383;
    if((rc=usb_controlTransfer(dev,0,&vread,buf,1,USB_TIMEOUT))<0)
        return rc;
    vread.wValue=0x8484;
    if((rc=usb_controlTransfer(dev,0,&vread,buf,1,USB_TIMEOUT))<0)
        return rc;
    if((rc=usb_controlTransfer(dev,0,&vwrite,NULL,0,USB_TIMEOUT))<0)
        return rc;
    if((rc=usb_controlTransfer(dev,0,&vread,buf,1,USB_TIMEOUT))<0)
        return rc;
    vread.wValue=0x8383;
    if((rc=usb_controlTransfer(dev,0,&vread,buf,1,USB_TIMEOUT))<0)
        return rc;
    vwrite.wValue=0;
    vwrite.wIndex=1;
    if((rc=usb_controlTransfer(dev,0,&vwrite,NULL,0,USB_TIMEOUT))<0)
        return rc;
    vwrite.wValue=1;
    vwrite.wIndex=0;
    if((rc=usb_controlTransfer(dev,0,&vwrite,NULL,0,USB_TIMEOUT))<0)
        return rc;
    vwrite.wValue=2;
    vwrite.wIndex=0x44;
    if((rc=usb_controlTransfer(dev,0,&vwrite,NULL,0,USB_TIMEOUT))<0)
        return rc;
    vwrite.wValue=8;
    vwrite.wIndex=0;
    if((rc=usb_controlTransfer(dev,0,&vwrite,NULL,0,USB_TIMEOUT))<0)
        return rc;
    vwrite.wValue=9;
    if((rc=usb_controlTransfer(dev,0,&vwrite,NULL,0,USB_TIMEOUT))<0)
        return rc;
    return 0;
}

int usb_serialFindSlot(usb_dev_t* dev)
{
	int i;
	if(dev == NULL) { /* Find free slot */
		for(i = 0;i < SERIAL_MAX_DEV_N;i++)
			if(serial_g.devices[i] == NULL)
				return i;
	} else { /* Find slot occupied by a device */
		for(i = 0;i < SERIAL_MAX_DEV_N;i++)
			if(serial_g.devices[i] != NULL && serial_g.devices[i]->dev == dev)
				return i;
	}
	return -1;
}


void usb_serialExample(void);


int usb_serialProbe(usb_dev_t* dev)
{
	int i;
	usb_ifDesc_t* intf = usb_cfgIfDesc(dev->cfg,0);
	usb_epDesc_t* ep;
	usb_epDesc_t* epIn = NULL;
	usb_epDesc_t* epOut = NULL;
	usb_epDesc_t* epIntr = NULL;
	usb_serialDev_t* sdev;
	vnode_t* vnode;
	char devName[] = "serialUSBn";
	int devNum;

	if(!(dev->desc.bDeviceClass == 0x2 || (dev->desc.bDeviceClass == 0x0 
					&& (intf->bInterfaceClass == 0x2 || intf->bInterfaceClass == 0xFF))))
		return 0;
	ep = usb_cfgEpDesc(dev->cfg,0,0);
	for(i = 0;i < intf->bNumEndpoints;i++) {
		if((ep->bmAttributes & USB_EP_TYPE_MASK) == USB_EP_BULK) {
			if(ep->bEndpointAddress & USB_EP_DIR_MASK)
				epIn = ep;
			else
				epOut = ep;
		} else if(((ep->bmAttributes & USB_EP_TYPE_MASK) == USB_EP_INTR) && (ep->bEndpointAddress & USB_EP_DIR_MASK))
			epIntr = ep;
		ep = usb_cfgSkipEndpoints(ep,1);
	}

	if(epIn == NULL || epOut == NULL)
		return 0;

	if((devNum = usb_serialFindSlot(NULL)) < 0)
		return 0;

	if((sdev = vm_kmalloc(sizeof(*sdev))) == NULL) {
		main_printf(ATTR_ERROR,"usb: Couldn't allocate memory for USB serial device\n");
		return 0;
	}

	serial_g.devices[devNum] = sdev;
	sdev->dev = dev;
	sdev->epIn = epIn;
	sdev->epOut = epOut;
	sdev->epIntr = epIntr;
	sdev->bLen = 0;
	sdev->top = 0;
	proc_mutexCreate(&sdev->mutex);
	
	usb_addEndpoint(dev,epIn);
	usb_addEndpoint(dev,epOut);
	if(epIntr != NULL)
		usb_addEndpoint(dev,epIntr);

    if((dev->desc.idVendor == 0x067b) && (dev->desc.idProduct == 0x2303))
        usb_pl2303Init(dev);

	usb_serialSetLineCoding(dev);
	usb_serialSetControlLineState(dev);

	fs_lookup("/dev", &vnode, 1);
    if (vnode) {
        devName[sizeof(devName)-2] = '0' + devNum;
        vnode_mknod(vnode,devName,MAKEDEV(MAJOR_USBSERIAL,devNum));
        vnode_put(vnode);
    }

	//usb_serialExample();
	return 1;
}

void usb_serialDisconnect(usb_dev_t* dev)
{
	char devName[] = "serialUSBn";
	int devNum;
	vnode_t* vnode;
	if((devNum = usb_serialFindSlot(dev)) >= 0) {
		proc_mutexTerminate(&serial_g.devices[devNum]->mutex);
		vm_kfree(serial_g.devices[devNum]);
		serial_g.devices[devNum] = NULL;
		fs_lookup("/dev", &vnode, 1);
		devName[sizeof(devName)-2] = '0' + devNum;
		vnode_unlink(vnode,devName);
	}
}


static usb_drv_t serialDriver = {
	.probe = usb_serialProbe,
	.disconnect = usb_serialDisconnect,
};


int usb_serialRead(file_t* file, offs_t offs, char* buff, unsigned int len)
{
    vnode_t* vnode = file->vnode;
	int rc;
	u8 devNum = MINOR(vnode->dev);
	u16 size;
	usb_serialDev_t* sdev = serial_g.devices[devNum];

	if(sdev == NULL)
		return -ENOENT;

	proc_mutexLock(&sdev->mutex);
	if(sdev->bLen > 0) {
		size = sdev->bLen-sdev->top;
		if(size > len)
			size = len;
		memcpy(buff+sdev->top,sdev->b,size);
		sdev->bLen -= size;
		if(sdev->bLen == 0)
			sdev->top = 0;
		else
			sdev->top += size;
		rc = size;
	} else {
		size = sdev->epIn->wMaxPacketSize;
		if(len < size)
			size = len;
		rc = usb_bulkTransfer(sdev->dev,sdev->epIn->bEndpointAddress & USB_EP_NUM_MASK,USB_DIR_IN,(u8*)buff,size,0);
	}
	proc_mutexUnlock(&sdev->mutex);
	//main_printf(ATTR_DEBUG,"usb: [serialRead] read %d bytes: \n",rc);
	//for(i = 0;i < rc;i++) main_printf(ATTR_DEBUG,"%x ",buff[i]);
	//main_printf(ATTR_DEBUG,"\n");
	return rc;
}


void usb_serialWriteCallback(usb_urb_t* urb)
{
	vm_kfree(urb);
}

int usb_serialWrite(file_t* file, offs_t offs, char* buff, unsigned int len)
{
    vnode_t* vnode = file->vnode;
	int rc;
	char* data;
	usb_urb_t* urb;
	u16 size;
	u8 devNum = MINOR(vnode->dev);
	usb_serialDev_t* sdev = serial_g.devices[devNum];

	if(sdev == NULL)
		return -ENOENT;

	size = sdev->epIn->wMaxPacketSize;
	if(len < size)
		size = len;
	if((urb = vm_kmalloc(sizeof(*urb)+size)) == NULL)
		return -ENOMEM;
	data = (char*)PTR_ADD(urb,sizeof(*urb));
	usb_fillUrb(urb,sdev->dev,USB_DIR_OUT | USB_PIPE_TYPE(USB_EP_BULK) | (sdev->epOut->bEndpointAddress & USB_EP_NUM_MASK),data,size,usb_serialWriteCallback);
	memcpy(data,buff,size);
	if((rc = usb_submitUrb(urb)) == EOK)
		return size;
	else
		return rc;
}


int usb_serialPoll(file_t* file, ktime_t timeout, int op)
{
	/* TODO: Add synchronization betweend reads and polls */
    vnode_t* vnode = file->vnode;
	int rc;
	u8 devNum = MINOR(vnode->dev);
	usb_serialDev_t* sdev = serial_g.devices[devNum];

	if(sdev == NULL)
		return -ENOENT;

	if(op == POLL_WRITE)
		return EOK;
	else if(op == POLL_READ) {
		if(sdev->bLen > 0)
			return EOK;
		proc_mutexLock(&sdev->mutex);
		rc = usb_bulkTransfer(sdev->dev,sdev->epIn->bEndpointAddress & USB_EP_NUM_MASK,USB_DIR_IN,(u8*)sdev->b,64,0);
		if(rc > 0) {
			sdev->bLen = rc;
			rc = EOK;
		} 
		proc_mutexUnlock(&sdev->mutex);
		return rc;
	} else
		return -EINVAL;
}


static const file_ops_t usb_serialOps = {
	.read = usb_serialRead,
	.write = usb_serialWrite,
	.poll = usb_serialPoll,
};


int usb_serialInit(void)
{
	int i,rc;
	for(i = 0;i < SERIAL_MAX_DEV_N;i++)
		serial_g.devices[i] = NULL;
	if((rc = dev_register(MAKEDEV(MAJOR_USBSERIAL,0),&usb_serialOps)) < 0)
		return rc;
	usb_addDriver(&serialDriver);
	return 0;
}

#if 0

#include <fs/phfs/phfs.h>

#define BUF_LEN (200+1)
void usb_serialExampleRead(void)
{
	int rc;
	char b[BUF_LEN];
	vnode_t* vnode;
    file_t* file;
	fs_lookup("/dev/serialUSB0", &vnode, 1);
    if (!vnode)
        return;
    fs_openv(vnode, O_WRONLY, &file);
    vnode_put(vnode);
    if (!file)
        return;
	main_printf(ATTR_DEBUG,"usb: [usb_serialExampleRead] start\n");
	while(1){
		if((rc = vnode_read(file,0,b,BUF_LEN-1)) < 0) {
			main_printf(ATTR_DEBUG,"usb: [usb_serialExampleRead] error in read = %d\n",rc);
            fs_close(file);
			return;
		}
		b[rc] = '\000';
		main_printf(ATTR_DEBUG,"[rc=%d] %s\n",rc,b);
	}
}

void usb_serialExampleWrite(void)
{
	int i,rc,rand,size;
	char ref[] = "abcdefghijklmnopqrstuvwxyz!@#";
	//char b[sizeof(ref)];
	vnode_t* vnode;
    file_t* file;
	fs_lookup("/dev/serialUSB0", &vnode, 1);
    if (!vnode)
        return;
    fs_openv(vnode, O_WRONLY, &file);
    vnode_put(vnode);
    if (!file)
        return;
	main_printf(ATTR_DEBUG,"usb: [usb_serialExampleRead] start\n");
	rand = 14; 
	while(1){
		rand = (rand * 3) % 29;
		size = rand % sizeof(ref);
		//memcpy(b,ref,size);
		if((rc = vnode_write(file,0,ref,size)) < 0) {
			main_printf(ATTR_DEBUG,"usb: [usb_serialExampleWrite] error in write = %d\n",rc);
            fs_close(file);
			return;
		}
		for(i=0;i < 100000;i++);
	}
}


void usb_serialPhfsMount(void)
{
	int rc;
	phfs_opt_t phfsOpt = {.magic = 0xaa55a55a, .transport = PHFS_SERIAL, .device = "/dev/serialUSB0"};

	vnode_t* vnode;
	fs_lookup("/", &vnode, 1);
	vnode_mkdir(vnode, "mnt", 0777);
    vnode_put(vnode);

	if((rc = fs_mount("/mnt",TYPE_PHFS,&phfsOpt)) < 0) {
		main_printf(ATTR_ERROR,"usb: [usb_serialPhfsMount] error mounting phfs");
		return;
	}
#if 0
	file_t* file;
	char b[BUF_LEN];
	if((rc = fs_open("/mnt/example.txt",O_RDONLY,&file)) < 0)
		main_printf(ATTR_ERROR,"usb: [usb_serialPhfsMount] error opening file");
	else {
		if((rc = fs_read(file,b,BUF_LEN)) < 0) {
			main_printf(ATTR_DEBUG,"usb: [usb_umassExampleFatMount] error in read = %d\n",rc);
		} else
			b[rc] = '\000';
			main_printf(ATTR_DEBUG,"usb: [umassExampleRead] %s\n",b);
		fs_close(file);
	}
#else
	flash_burn("/mnt/image.img",NULL /*"/mnt/phoenix.img"*/,"/dev/flash0",0);
#endif
}


void usb_serialExampleIntr(void)
{
	int i;
	u8 b[0xa];
	usb_serialDev_t* sdev = serial_g.devices[0];
	main_printf(ATTR_DEBUG,"usb: [serialExampleIntr] begin\n");
	while (1) {
		usb_intrTransfer(sdev->dev,sdev->epIntr->bEndpointAddress & USB_EP_ADDR_MASK,
				USB_DIR_IN,b,0xa,0);
		main_printf(ATTR_DEBUG,"usb: [serialExampleIntr] b = [ ");
		for(i = 0;i < 0xa;i++)
			main_printf(ATTR_DEBUG,"%x ",(u32)b[i]);
		main_printf(ATTR_DEBUG,"]\n");
	};
}


void usb_serialExample(void)
{
//	usb_serialExampleRead();
//	usb_serialExampleWrite();
	usb_serialPhfsMount();
//	usb_serialExampleIntr();
}

#endif
