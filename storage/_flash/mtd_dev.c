/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * FLASH memory MTD driver
 *
 * Copyright 2014 Phoenix Systems
 *
 * Author: Katarzyna Baranowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <dev/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <include/flashctl.h>
#include <dev/flash/mtd_if.h>
#include <dev/flash/mtd_local_if.h>


typedef struct {
	mutex_t mutex;
	u32 size;
	u16 pageSize;
	ktime_t byteWriteDuration;
	ktime_t pageWriteDuration;
	ktime_t eraseDuration;
	ktime_t eraseAllDuration;
}  flash_chip_t;


static flash_chip_t *chips;
static int chipsCount = 0;


void mtd_lock(unsigned int flNo)
{
	proc_mutexLock(&chips[flNo].mutex);
}


void mtd_unlock(unsigned int flNo)
{
	proc_mutexUnlock(&chips[flNo].mutex);
}


static int _mtd_waitWhileFlashBusy(unsigned int flNo, ktime_t opDuration)
{
	int status;
	opDuration = opDuration / 4;
	do {
		if (opDuration >= 1000) proc_threadSleep(opDuration/8 + 1);
		else if (opDuration >= 200) hal_cpuReschedule();
		status = _mtd_getStatus(flNo);
	} while (status == -EBUSY);
	return status;
}


int _mtd_programPageWithWait(unsigned int flNo, offs_t offs, char *buff, unsigned int len, unsigned int *programmedLen)
{
	int ret;
	if ((ret = _mtd_programPage(flNo, offs, buff, len, programmedLen)) != EOK)
		return ret;
	return _mtd_waitWhileFlashBusy(flNo, chips[flNo].pageWriteDuration);
}


int _mtd_programWithWait(unsigned int flNo, offs_t offs, char *buff, unsigned int len, unsigned int *programmedLen)
{
	int ret;
	offs_t pageOffs;
	unsigned int chunk;
	unsigned int programmed;

	if (programmedLen != NULL)
		*programmedLen = 0;
	while(len > 0) {
		pageOffs = offs % chips[flNo].pageSize;
		chunk = (len < (chips[flNo].pageSize - pageOffs)) ? len : (chips[flNo].pageSize - pageOffs);
		if ((ret = _mtd_programPageWithWait(flNo, offs, buff, chunk, &programmed)) != EOK) {
			if (programmedLen != NULL)
				*programmedLen += programmed;
			return ret;
		}
		if (programmedLen != NULL)
			*programmedLen += chunk;
		buff += chunk;
		offs += chunk;
		len -= chunk;
	}
	return EOK;
}


int _mtd_eraseWithWait(unsigned int flNo, offs_t offs)
{
	int ret;
	if ((ret = _mtd_erase(flNo, offs)) != EOK) return ret;
	return _mtd_waitWhileFlashBusy(flNo, chips[flNo].eraseDuration);
}


int _mtd_eraseUniformWithWait(unsigned int flNo, offs_t offs)
{
	int ret;
	if ((ret = _mtd_eraseUniform(flNo, offs)) != EOK) return ret;
	return _mtd_waitWhileFlashBusy(flNo, chips[flNo].eraseDuration);
}


int _mtd_eraseAllWithWait(unsigned int flNo)
{
	int ret;
	if ((ret = _mtd_eraseAll(flNo)) != EOK) return ret;
	return _mtd_waitWhileFlashBusy(flNo, chips[flNo].eraseAllDuration);
}


static int mtd_devRead(file_t* file, offs_t offs, char *buff, unsigned int len)
{
    vnode_t *vnode = file->vnode;
	int ret;
	unsigned int flNo;

	if (vnode == NULL) return -EINVAL;
	if (buff == NULL) return -EINVAL;
	flNo = MINOR(vnode->dev);
	if (flNo >= chipsCount) return -ENOENT;
	if ((offs + len) > chips[flNo].size) len = chips[flNo].size - offs;
	if (len == 0) return EOK;

	mtd_lock(flNo);
	ret = _mtd_read(flNo, offs, buff, len, NULL);
	mtd_unlock(flNo);

	if (ret == EOK)
		return len;
	else
		return ret;
}


static int mtd_devIOCtl(file_t* file, unsigned int cmd, unsigned long arg)
{
    vnode_t *vnode = file->vnode;
	int ret, regNo;
	flash_cfi_t cfi;
	flash_info_t *fli = (flash_info_t *) arg;
	
	if (vnode == NULL) return -EINVAL;
	if (MINOR(vnode->dev) >= chipsCount) return -ENOENT;

	switch(cmd) {
		case FLASH_IOC_INFO_GET:
			if ((void *)arg == NULL) return -EINVAL;
			mtd_lock(MINOR(vnode->dev));
			/* no read, program or erase can be in progress when reading cfi */
			ret = _mtd_getCfi(MINOR(vnode->dev), &cfi);
			mtd_unlock(MINOR(vnode->dev));
			if (ret == EOK) {
				fli->pageSize = 1 << cfi.bufferSize;
				fli->regionCount = cfi.regionsCount;
				for (regNo = 0; regNo < cfi.regionsCount; regNo++) {
					fli->blocks[regNo].count = cfi.region[regNo].blockCount + 1;
					if (cfi.region[regNo].blockSize == 0)
						fli->blocks[regNo].size = 128;
					else
						fli->blocks[regNo].size = cfi.region[regNo].blockSize * 256;
				}
			}
			return ret;
		default:
			return -ENOENT;
	}
}


int _mtd_devInit(void)
{
	int ret;
	int chipNo;
	flash_cfi_t cfi;
	
	static const file_ops_t mtd_ops = {
		.read  = mtd_devRead,
		.ioctl = mtd_devIOCtl,
	};

	if ((ret = _mtd_init()) == EOK) {
		chipsCount = mtd_getCount();
		chips = (flash_chip_t *) vm_kmalloc(sizeof(flash_chip_t) * chipsCount);
		if (chips == NULL) return -ENOMEM;

		for (chipNo = 0; chipNo < chipsCount; chipNo++) {
			proc_mutexCreate(&chips[chipNo].mutex);
			ret = _mtd_getCfi(chipNo, &cfi);
			if (ret != EOK) {
				main_printf(ATTR_ERROR, "dev/mtd: Can't communicate with flash number %d!\n", chipNo);
				chips[chipNo].size = 0;
			} else {
				chips[chipNo].size = 1 << cfi.chipSize;
				chips[chipNo].byteWriteDuration = 1 << cfi.timeoutTypical.byteWrite;
				chips[chipNo].pageWriteDuration = 1 << cfi.timeoutTypical.bufferWrite;
				chips[chipNo].eraseDuration = (1 << cfi.timeoutTypical.sectorErase) * 1000;
				chips[chipNo].eraseAllDuration = (1 << cfi.timeoutTypical.chipErase) * 1000;
				chips[chipNo].pageSize = 1 << cfi.bufferSize;
				main_printf(ATTR_INFO, "dev/mtd: FLASH %d: %d MB\n", chipNo, chips[chipNo].size / 1024 / 1024);
			}
		}
		if ((ret = dev_register(MAKEDEV(MAJOR_MTD, 0), &mtd_ops)) == EOK) {
			assert(EOK==dev_mknod(MAKEDEV(MAJOR_MTD, 0), "mtd0"));
			return EOK;
		}
	}
	main_printf(ATTR_ERROR, "dev/mtd: Can't register device!\n");
	return ret;
}
