#include <include/errno.h>
#include <lib/list.h>
#include <vm/if.h>
#include <main/if.h>
#include <dev/if.h>
#include <fs/if.h>

#include <usb/usb.h>
#include <usb/usb_spec.h>
#include <usb/urb.h>


typedef struct usb_umassDev_ usb_umassDev_t;
struct usb_umassDev_ {
	usb_dev_t* dev;
	usb_epDesc_t* epIn;
	usb_epDesc_t* epOut;
	mutex_t mutex;
	u32 cbwTag;
	u32 blockNum;
	u32 blockSize;
	u16 minor;
	u8 maxLun; 
};


typedef struct usb_cbw_ {
	u32 dCBWSignature;
	u32 dCBWTag;
	u32 dCBWDataTransferLength;
	u8 bmCBWFlags;
	u8 bCBWLUN;
	u8 bCBWCBLength;
	u8 CBWCB[16];
} PACKED usb_cbw_t;
#define USB_CBW_SIG 0x43425355


typedef struct usb_csw_ {
	u32 dCSWSignature;
	u32 dCSWTag;
	u32 dCSWDataResidue;
	u8 bCSWStatus;
} PACKED usb_csw_t;
#define USB_CSW_SIG 0x53425355


typedef struct scsi_cmdShort_ {
	u8 dir;
	u8 clen;
	u8 op;
	u8 lun;
	u16 lba;
	u8 len;
	u8 ctrl;
} PACKED scsi_cmdShort_t;


typedef struct scsi_cmdStd_ {
	u8 dir;
	u8 clen;
	u8 op;
	u8 lun;
	u32 lba;
	u8 _res1;
	u16 len;
	u8 ctrl;
} PACKED scsi_cmdStd_t;


typedef union scsi_cmd_ {
	scsi_cmdShort_t cmdShort;
	scsi_cmdStd_t cmdStd;
} PACKED scsi_cmd_t;

#define SCSI_REQ_SHORT 6
#define SCSI_REQ_STD 10
#define SCSI_WRITE 0x00
#define SCSI_READ 0x80

#define SCSI_TEST_READY 0x00
#define SCSI_REQUEST_SENSE 0x03
#define SCSI_READ_CAPACITY 0x25
#define SCSI_READ_EXT 0x28
#define SCSI_WRITE_EXT 0x2A


typedef struct scsi_capacity_ {
	u32 lba;
	u32 bl;
} scsi_capacity_t;


#define UMASS_MAX_DEV_N 8
static struct {
	spinlock_t lock;
	usb_umassDev_t* devices[UMASS_MAX_DEV_N];
} umass_g;


int usb_umassGetMaxLun(usb_dev_t* dev,u8* maxLun);
int usb_umassReset(usb_dev_t* dev);
int usb_umassTestReady(usb_umassDev_t* udev);
int usb_umassRequestSense(usb_umassDev_t* udev);
int usb_umassReadCapacity(usb_umassDev_t* udev,scsi_capacity_t* cap);
int usb_umassRead(file_t* file,offs_t offs,char* buff,unsigned int len);
int usb_umassWrite(file_t* file,offs_t offs, char* buff,unsigned int len);


static inline u32 byteSwap32(u32 v)
{
	return (v << 24) | ((v & 0xFF00) << 8) | ((v & 0xFF0000) >> 8) | (v >> 24);
}

static inline u16 byteSwap16(u16 v)
{
	return (v << 8) | (v >> 8);
}


/*static inline */int usb_umassFindSlot(usb_dev_t* dev)
{
	int i;
	if(dev == NULL) {/* Find free slot */
		for(i = 0;i < UMASS_MAX_DEV_N;i++)
			if(umass_g.devices[i] == NULL)
				return i;
	}
	else {/* Find slot occupied by a device */
		for(i = 0;i < UMASS_MAX_DEV_N;i++)
			if(umass_g.devices[i] != NULL && umass_g.devices[i]->dev == dev)
				return i;
	}
	return -1;
}


void usb_umassExampleFatMount(void);
int usb_umassProbe(usb_dev_t* dev)
{
	int i,rc;
	usb_ifDesc_t* intf = usb_cfgIfDesc(dev->cfg,0);
	usb_epDesc_t* ep;
	usb_epDesc_t* epIn = NULL;
	usb_epDesc_t* epOut = NULL;
	usb_umassDev_t* udev;
	vnode_t* vnode;
	char devName[] = "usbdokn";
	int devNum;
	scsi_capacity_t cap;

	if(!(USB_CHECK_DEV_TYPE(&dev->desc,8,6,80) ||
		(dev->desc.bDeviceClass == 0 && USB_CHECK_IF_TYPE(intf,8,6,80))))
		return 0;

	ep = usb_cfgEpDesc(dev->cfg,0,0);
	for(i = 0;i < intf->bNumEndpoints;i++) {
		if((ep->bmAttributes & USB_EP_TYPE_MASK) == USB_EP_BULK) {
			if(ep->bEndpointAddress & USB_EP_DIR_MASK)
				epIn = ep;
			else
				epOut = ep;
		}
		ep = usb_cfgSkipEndpoints(ep,1);
	}

	if(epIn == NULL || epOut == NULL)
		return 0;
	//epIn->bEndpointAddress = 1 | USB_EP_DIR_MASK;

	if((devNum = usb_umassFindSlot(NULL)) < 0)
		return 0;

	if((udev = vm_kmalloc(sizeof(usb_umassDev_t))) == NULL) {
		main_printf(ATTR_ERROR,"usb: Couldn't allocate memory for device\n");
		return 0;
	}

	umass_g.devices[devNum] = udev;
	udev->dev = dev;
	udev->epIn = epIn;
	udev->epOut = epOut;
	udev->cbwTag = 0;
	proc_mutexCreate(&udev->mutex);

	usb_addEndpoint(dev,epIn);
	usb_addEndpoint(dev,epOut);

	if((rc = usb_umassGetMaxLun(udev->dev,&udev->maxLun)) < 0) {
		udev->maxLun = 0;
		main_printf(ATTR_ERROR,"usb: [umassProbe] failed to retrieve max LUN (%d)\n",rc);
	} else {
		main_printf(ATTR_DEBUG,"usb: [umassProbe] maxLun = %d\n",udev->maxLun);
	}

	rc = usb_umassTestReady(udev);

	if(rc != 0) {
		main_printf(ATTR_ERROR,"usb: [umassProbe] device not ready (%d)\n",rc);
		if(usb_umassRequestSense(udev) < 0)
			main_printf(ATTR_ERROR,"usb: [umassProbe] request sense failed (%d)\n",rc);
	}

	if((rc = usb_umassReadCapacity(udev,&cap)) < 0) {
		udev->blockNum = 0;
		udev->blockSize = 512; /* Default block size */
		main_printf(ATTR_ERROR,"usb: [umassProbe] failed to retrieve capacity (%d), using default block size 512B\n",rc);
	} else {
		udev->blockNum = byteSwap32(cap.lba);
		udev->blockSize = byteSwap32(cap.bl);
		main_printf(ATTR_DEBUG,"usb: [umassProbe] capacity = %dMB { lba = %d, bl = %d }\n", (int)(((udev->blockNum + 1)*(u64)udev->blockSize)/(1024*1024)), (int)udev->blockNum, (int)udev->blockSize);
	}

	fs_lookup("/dev", &vnode, 1);
	devName[sizeof(devName)-2] = '0' + devNum;
	vnode_mknod(vnode,devName,MAKEDEV(MAJOR_UMASS,devNum));

	//usb_umassExampleFatMount();

	return 1;
}


void usb_umassDisconnect(usb_dev_t* dev)
{
	int devNum;
	char devName[] = "usbdokn";
	vnode_t* vnode;
	if((devNum = usb_umassFindSlot(dev)) >= 0) {
		proc_mutexTerminate(&umass_g.devices[devNum]->mutex);
		vm_kfree(umass_g.devices[devNum]);
		umass_g.devices[devNum] = NULL;
		fs_lookup("/dev", &vnode, 1);
		devName[sizeof(devName)-2] = '0' + devNum;
		vnode_unlink(vnode,devName);
	}
}


static usb_driver_t umassDriver = {
	.probe = usb_umassProbe,
	.disconnect = usb_umassDisconnect,
};


static const file_ops_t usb_umassOps = {
	.read = usb_umassRead,
	.write = usb_umassWrite,
};


int usb_umassInit(void)
{
	int i,rc;
	for(i = 0;i < UMASS_MAX_DEV_N;i++)
		umass_g.devices[i] = NULL;
	proc_spinlockCreate(&umass_g.lock,"umass lock");
	if((rc = dev_register(MAKEDEV(MAJOR_UMASS,0),&usb_umassOps)) < 0)
		return rc;
	usb_addDriver(&umassDriver);
	return EOK;
}


int usb_umassReset(usb_dev_t* dev)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_WRITE_CLASS_IF,
		.bRequest = 0xFF,
		.wValue = 0,
		.wIndex = 0,
		.wLength = 0,
	};
	return usb_controlTransfer(dev,0,&req,NULL,0,USB_TIMEOUT);
}


int usb_umassGetMaxLun(usb_dev_t* dev,u8* maxLun)
{
	usb_ctrlreq_t req = {
		.bmRequestType = USB_READ_CLASS_IF,
		.bRequest = 0xFE,
		.wValue = 0,
		.wIndex = 0,
		.wLength = 1,
	};
	return usb_controlTransfer(dev,0,&req,maxLun,1,USB_TIMEOUT);
}


int usb_umassTransfer(usb_umassDev_t* udev,scsi_cmd_t* cmd,u8* data,u16 len)
{
	int rc;
	usb_cbw_t cbw = {
		.dCBWSignature = USB_CBW_SIG,
		.dCBWDataTransferLength = len,
		.bmCBWFlags = cmd->cmdShort.dir,
		.bCBWLUN = 0,
		.bCBWCBLength = cmd->cmdShort.clen,
	};
	usb_csw_t csw;
	u32 transferred = 0;
	u8 retry = 5;
	u8 finish = 0;

	memcpy(cbw.CBWCB,&cmd->cmdShort.op,cmd->cmdShort.clen);

	proc_mutexLock(&udev->mutex);
	while(!finish && (retry-- > 0)) {
		cbw.dCBWTag = udev->cbwTag++;

		if((rc = usb_bulkTransfer(udev->dev,udev->epOut->bEndpointAddress & USB_EP_NUM_MASK,
						USB_DIR_OUT,(u8*)&cbw,sizeof(usb_cbw_t),USB_TIMEOUT)) != sizeof(usb_cbw_t)) {
			main_printf(ATTR_ERROR,"usb: Error when sending umass transfer command (%d)\n",rc);
			if(rc == -EPIPE)
				if(usb_resetEndpoint(udev->dev,udev->epOut->bEndpointAddress & USB_EP_NUM_MASK) < 0) {
					proc_mutexUnlock(&udev->mutex);
					return rc;
				}
			if(usb_umassReset(udev->dev) < 0) {;
				proc_mutexUnlock(&udev->mutex);
				return rc;
			}
			continue;
		}

		if(len > 0) {
			usb_epDesc_t* ep = cmd->cmdShort.dir == SCSI_READ ? udev->epIn : udev->epOut;
			if((rc = usb_bulkTransfer(udev->dev,ep->bEndpointAddress & USB_EP_NUM_MASK,
							ep->bEndpointAddress & USB_EP_DIR_MASK,data,len,USB_TIMEOUT)) < 0) {
				main_printf(ATTR_ERROR,"usb: Error in umass transfer (%d)\n",rc);
				if(rc == -EPIPE)
					if(usb_resetEndpoint(udev->dev,udev->epOut->bEndpointAddress & USB_EP_NUM_MASK) < 0) {
						proc_mutexUnlock(&udev->mutex);
						return rc;
					}
				if(usb_umassReset(udev->dev) < 0) {;
					proc_mutexUnlock(&udev->mutex);
					return rc;
				}
				continue;
			} else
				transferred = rc;
		}

		/* Read CSW */
		if((rc = usb_bulkTransfer(udev->dev,udev->epIn->bEndpointAddress & USB_EP_NUM_MASK,USB_DIR_IN,(u8*)&csw,sizeof(csw),USB_TIMEOUT)) != sizeof(csw)) {
			main_printf(ATTR_ERROR,"usb: Error getting umass transfer status (%d)\n",rc);
			if(rc == -EPIPE)
				if(usb_resetEndpoint(udev->dev,udev->epOut->bEndpointAddress & USB_EP_NUM_MASK) < 0) {
					proc_mutexUnlock(&udev->mutex);
					return rc;
				}
			if(usb_umassReset(udev->dev) < 0) {;
				proc_mutexUnlock(&udev->mutex);
				return rc;
			}
			continue;
		}

		main_printf(ATTR_DEBUG,"usb: umass_csw {sig = %x,tag = %d,res = %d,st = %x}\n",csw.dCSWSignature,csw.dCSWTag,csw.dCSWDataResidue,csw.bCSWStatus);
		if(csw.dCSWSignature != USB_CSW_SIG) {
			main_printf(ATTR_ERROR,"usb: [umassTransfer] Invalid command status (%x)\n",csw.dCSWSignature);
			rc = -EINVAL;
			continue;
		}

		if(csw.bCSWStatus != 0) {
			main_printf(ATTR_ERROR,"usb: [umassTransfer] Transfer failed (%x)\n",csw.bCSWStatus); 
			if(csw.bCSWStatus == 0x01)
				rc = -EFAULT;
			else 
				rc = -EPROTO;
			continue;
		}

		finish = 1;

	}
	proc_mutexUnlock(&udev->mutex);

	if(rc >= 0)	
		rc = transferred;
	return rc;
}


int usb_umassTestReady(usb_umassDev_t* udev)
{
	scsi_cmdShort_t cmd = {
		.dir = SCSI_READ,
		.clen = SCSI_REQ_SHORT,
		.op = 0x00,
		.lun = 0,
		.lba = 0,
		.len = 0,
		.ctrl = 0,
	};
	return usb_umassTransfer(udev,(scsi_cmd_t*)&cmd,NULL,0);
}


#define UMASS_MAX_SENSE_DATA 252
int usb_umassRequestSense(usb_umassDev_t* udev)
{
	int rc;
	scsi_cmdShort_t cmd = {
		.dir = SCSI_READ,
		.clen = SCSI_REQ_SHORT,
		.op = 0x03,
		.lun = 0,
		.lba = 0,
		.len = UMASS_MAX_SENSE_DATA,
		.ctrl = 0,
	};
	u8 senseData[UMASS_MAX_SENSE_DATA];
	rc = usb_umassTransfer(udev,(scsi_cmd_t*)&cmd,senseData,UMASS_MAX_SENSE_DATA);
	return rc;
}


int usb_umassReadCapacity(usb_umassDev_t* udev,scsi_capacity_t* cap)
{
	scsi_cmdStd_t cmd = {
		.dir = SCSI_READ,
		.clen = SCSI_REQ_STD,
		.op = SCSI_READ_CAPACITY,
		.lun = 0,
		.lba = 0,
		.len = 0,
		.ctrl = 0,
	};
	return usb_umassTransfer(udev,(scsi_cmd_t*)&cmd,(u8*)cap,sizeof(*cap));
}


int usb_umassRead(file_t* file, offs_t offs, char* buff, unsigned int len)
{
    vnode_t* vnode = file->vnode;
	int rc;
	//int err = 0;
	scsi_cmdStd_t cmd = {
		.dir = SCSI_READ,
		.clen = SCSI_REQ_STD,
		.op = SCSI_READ_EXT,
		.lun = 0,
		.ctrl = 0,
	};

	u8 devNum = MINOR(vnode->dev);
	u32 transferred = 0;
	usb_umassDev_t* udev = umass_g.devices[devNum];
	/*
	u8* tmp = NULL;
	u32 blOff;
	u32 size;
	*/

	if(udev == NULL)
		return -ENOENT;

	if((offs % udev->blockSize) || (len % udev->blockSize)) {
		main_printf(ATTR_ERROR,"usb: [umassRead] invalid offset or length\n");
		return -EINVAL;
	}
	/*
	if((offs % udev->blockSize) || (len % udev->blockSize)) {
		if((tmp = vm_kmalloc(udev->blockSize)) == NULL)
			return -ENOMEM;
	}

	if((blOff = offs % udev->blockSize)) {
		cmd.lba = byteSwap32(offs / udev->blockSize);
		cmd.len = byteSwap16(1);
		if((rc = usb_umassTransfer(udev,(scsi_cmd_t*)&cmd,tmp,udev->blockSize)) < 0) {
			main_printf(ATTR_ERROR,"usb: [umassRead] Transfer error (%d)\n",rc);
			vm_kfree(tmp);
			return rc;
		}
		size = rc - blOff;
		if(size > len)
			size = len;
		memcpy(buff,tmp + blOff,size);
		buff += size;
		transferred += size;
		offs += size;
		len -= size;
		if(rc != udev->blockSize) {
			main_printf(ATTR_ERROR,"usb: [umassRead] Couldn't read all data (%d)\n",rc);
			vm_kfree(tmp);
			return transferred;
		}
	}
	*/
		/*
		if(len % udev->blockSize) {
			req.lba = BIGEND(num);
			req.len = BIGEND(len);
			rc = usb_umassTransfer(udev,&req,tmp,udev->blockSize);
			// check that rc == blockSize
			// Decrease len accordingly and memcpy to buff
			// and add to transferred
		}
		*/
	cmd.lba = byteSwap32(offs / udev->blockSize);
	cmd.len = byteSwap16(len / udev->blockSize);
	if((rc = usb_umassTransfer(udev,(scsi_cmd_t*)&cmd,(u8*)buff,len)) < 0) {
		main_printf(ATTR_ERROR,"usb: [umassRead] Transfer error (%d)\n",rc);
		return rc;
	}
	transferred += rc;
	if(rc != len) {
		main_printf(ATTR_ERROR,"usb: [umassRead] Couldn't read all data (%d)\n",rc);
	}

	/*TODO: Read also remaining data if last read doesn't finish at sector end */
	return transferred;
}


int usb_umassWrite(file_t* file,offs_t offs, char* buff,unsigned int len)
{
    vnode_t* vnode = file->vnode;
	int rc;
	scsi_cmdStd_t cmd = {
		.dir = SCSI_WRITE,
		.clen = SCSI_REQ_STD,
		.op = SCSI_WRITE_EXT,
		.lun = 0,
		.ctrl = 0,
	};

	u8 devNum = MINOR(vnode->dev);
	u32 transferred = 0;
	usb_umassDev_t* udev = umass_g.devices[devNum];

	if(udev == NULL)
		return -ENOENT;

	if((offs % udev->blockSize) || (len % udev->blockSize)) {
		main_printf(ATTR_ERROR,"usb: [umassWrite] invalid offset or length\n");
		return -EINVAL;
	}

	cmd.lba = byteSwap32(offs / udev->blockSize);
	cmd.len = byteSwap16(len / udev->blockSize);
	if((rc = usb_umassTransfer(udev,(scsi_cmd_t*)&cmd,(u8*)buff,len)) < 0) {
		main_printf(ATTR_ERROR,"usb: [umassWrite] Transfer error (%d)\n",rc);
		return rc;
	}
	
	transferred += rc;
	if(rc != udev->blockSize) {
		main_printf(ATTR_ERROR,"usb: [umassWrite] Couldn't read all data (%d)\n",rc);
	}

	return transferred;
}


#if 0

#define BUF_LEN 512
void usb_umassExampleRead(void)
{
	int rc;
	char b[BUF_LEN];
	vnode_t* vnode;
    file_t* file;
	fs_lookup("/dev/usbdok0", &vnode, 1);
    if (!vnode)
        return;
    fs_openv(vnode, O_RDONLY, &file);
    vnode_put(vnode);
    if (!file)
        return;
	if((rc = vnode_read(file,0,b,BUF_LEN)) < 0) {
		main_printf(ATTR_DEBUG,"usb: [usb_umassExampleRead] error in read = %d\n",rc);
        fs_close(file);
		return;
	}
	main_printf(ATTR_DEBUG,"usb: [umassExampleRead] %s\n",b);
	/*
	main_printf(ATTR_DEBUG,"usb: [usb_umassExampleRead] start\n");
	while(c != '\000') {
		if((rc = vnode_read(vnode,of++,&c,1)) < 0) {
			main_printf(ATTR_DEBUG,"usb: [usb_umassExampleRead] error in read = %d\n",rc);
			return;
		}
		main_printf(ATTR_DEBUG,"%c",c);
	}
	main_printf(ATTR_DEBUG,"\n",c);
	*/
}

void usb_umassExampleWrite(void)
{
	int rc;
	char b[BUF_LEN] = "Pisze Ci tu, ze nie ma przyszlosci\n\000";
	vnode_t* vnode;
    file_t* file;
	fs_lookup("/dev/usbdok0", &vnode, 1);
    if (!vnode)
        return;
    fs_openv(vnode, O_WRONLY, &file);
    vnode_put(vnode);
    if (!file)
        return;
	if((rc = vnode_write(file,0,b,BUF_LEN)) < 0) {
		main_printf(ATTR_DEBUG,"usb: [usb_umassExampleWrite] error in read = %d\n",rc);
        fs_close(file);
		return;
	}
	main_printf(ATTR_DEBUG,"usb: [umassExampleWrite] %s\n",b);
}



#include <fs/ufat/fat.h>

void usb_umassExampleFatMount()
{
	int rc;
	file_t* file;
	char b[BUF_LEN];
	fat_opt_t fatOpt = { 
		.device = "/dev/usbdok0",
		.sector_offset = 0,
		.sfntolower = 0,
		.precache = 0,
	};

	vnode_t* vnode;
	fs_lookup("/", &vnode, 1);
	vnode_mkdir(vnode,"mnt",0777);
    vnode_put(vnode);

	if((rc = fs_mount("/mnt",TYPE_FATFS,&fatOpt)) < 0) {
		main_printf(ATTR_ERROR,"usb: [usb_umassExampleFatMount] error mounting fat (%d)\n",rc);
		return;
	}
	if((rc = fs_open("/mnt/hello.txt",O_RDONLY,&file)) < 0) {
		main_printf(ATTR_ERROR,"usb: [usb_umassExampleFatMount] error opening file (%d)",rc);
	} else {
		main_printf(ATTR_INFO,"Successful open\n");
		if((rc = fs_read(file,b,BUF_LEN)) < 0) {
			main_printf(ATTR_DEBUG,"usb: [usb_umassExampleFatMount] error in read = %d\n",rc);
		} else
			main_printf(ATTR_DEBUG,"usb: [umassExampleRead] %s\n",b);
		fs_close(file);
	}
	
}

#endif




/*
static int usb_umassThread(void* data)
{
	while (1) {
		usb_umassDev_t* udev;
		udev = umass_g.devices.first;
		if(udev != NULL)
			do {
				// Check LUN
			} while (udev != umass_g.devices.first);
	};
}
*/


