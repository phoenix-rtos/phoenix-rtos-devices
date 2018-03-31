/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * FLASH memory DRIVE driver
 *
 * Copyright 2012-2013 Phoenix Systems
 *
 * Author: Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <dev/flash/mtd_if.h>

#include <dev/if.h>
#include <proc/if.h>
#include <main/if.h>
#include <include/flashctl.h>
#include <fs/vnode.h>


typedef struct {
	unsigned int flNo;
	u32  chipSize;
	u16 pageSize;
	u8 regionsCount;
	struct {
		u32 sectorCount;
		u32 sectorSize;
	} region[4];
} mtd_t;


static mtd_t *mtds = NULL;
static unsigned int mtdsCount = 0;


static int _flash_sectorProgram(mtd_t *mtd, u32 sectorBegin, u32 sectorOffs, char *buff, unsigned int len)
{
	u32 pageBegin = sectorBegin + (sectorOffs / mtd->pageSize) * mtd->pageSize;
	u32 pageOffs  = sectorOffs % mtd->pageSize;
	unsigned int chunk;
	char *firstDiff, *lastDiff;

	int ret;
	while (len > 0) {
		if (len + pageOffs > mtd->pageSize)
			chunk = mtd->pageSize - pageOffs;
		else
			chunk = len;
		while ((chunk > 0) && (*buff == 0xFF)) {
			chunk--;
			buff++;
			pageOffs++;
			len--;
		}
		if (chunk > 0) {
			if ((ret = _mtd_compare(mtd->flNo, pageBegin + pageOffs, buff, chunk, &firstDiff, &lastDiff)) < 0)
				return ret;
			if (ret == MTD_TO_ERASE)
				return -EIO;
			if (ret == MTD_TO_WRITE)
				if ((ret = _mtd_programPageWithWait(mtd->flNo, pageBegin + pageOffs + (firstDiff - buff), firstDiff, lastDiff - firstDiff + 1, NULL)) != EOK)
					return ret;
		}
		buff += chunk;
		pageBegin += mtd->pageSize;
		pageOffs = 0;
		len -= chunk;
	}
	return EOK;
}


static int flash_devWrite(file_t* file, offs_t offs, char *buff, unsigned int len)
{
    vnode_t *vnode = file->vnode;
	int ret = EOK;
	mtd_t *mtd;
	char *buffPtr = buff;
	char *data = NULL;
	int reg;
	page_t *pages = NULL;
	size_t chunk;
	u32 sectorBegin, sectorOffs, sectorNo;
	unsigned int sectorSize, dataSize = 0;

	if (vnode == NULL)
		return -EINVAL;
	if (MINOR(vnode->dev) >= mtdsCount)
		return -ENODEV;
	mtd = &mtds[MINOR(vnode->dev)];
	if (offs >= mtd->chipSize)
		return EOK;
	if (offs + len > mtd->chipSize) len = (mtd->chipSize - offs);
	
	reg = 0;
	sectorBegin = 0;
	
	for (reg = 0; offs > sectorBegin + mtd->region[reg].sectorSize * mtd->region[reg].sectorCount; reg++) {
		sectorBegin += mtd->region[reg].sectorSize * mtd->region[reg].sectorCount;
	}

	sectorSize = mtd->region[reg].sectorSize;
	sectorNo = ((u32)offs - sectorBegin) / sectorSize;
	sectorBegin += sectorNo * sectorSize;
	sectorOffs = (u32)offs - sectorBegin;

	/* Split write into subsequent sectors */
	while (len > 0) {
		/* Calculate amount of data to write in the current sector */
		if (len > sectorSize - sectorOffs)
			chunk = sectorSize - sectorOffs;
		else
			chunk = len;

		mtd_lock(mtd->flNo);

		/* Check if we can write without erase */
		if ((ret = _mtd_compare(mtd->flNo, offs, buffPtr, chunk, 0, 0)) < 0) goto error;

		if (ret != MTD_THE_SAME) {
			if (ret == MTD_TO_WRITE) {
				if ((ret = _flash_sectorProgram(mtd, sectorBegin, sectorOffs, buffPtr, chunk)) != EOK) goto error;
			} else {
				if (chunk != sectorSize) {
					/* We have to program just some part of sector. Therefore original content
					* of the complete sector is saved first, then it is updated with new data.
					*/
					if ((data != NULL) && ((dataSize - 1)/SIZE_PAGE < (sectorSize - 1)/SIZE_PAGE)) {
						vm_kunmap(data);
						vm_pageFree(pages);
					}
					if (data == NULL) {
						if ((pages = vm_pageAlloc((sectorSize - 1)/SIZE_PAGE + 1, vm_pageAlloc)) == NULL) {
							ret = -ENOMEM;
							goto error;
						} else if ((ret = vm_kmap(pages, PGHD_KERNEL_RW, (void **) &data)) != EOK) {
							vm_pageFree(pages);
							goto error;
						}
						dataSize = sectorSize;
					}
					if ((sectorOffs > 0) && ((ret = _mtd_read(mtd->flNo, sectorBegin, data, sectorOffs, NULL)) != EOK)) goto error;
					main_memcpy(data + sectorOffs, buffPtr, chunk);
					if ((sectorSize - sectorOffs - chunk > 0)
						&& ((ret = _mtd_read(mtd->flNo, sectorBegin + sectorOffs + chunk, data + sectorOffs + chunk, sectorSize - sectorOffs - chunk, NULL)) != EOK)) goto error;
				}
				if ((ret = _mtd_eraseWithWait(mtd->flNo, sectorBegin)) != EOK) goto error;
				if (chunk != sectorSize) {
					if ((ret = _flash_sectorProgram(mtd, sectorBegin, 0, data, sectorSize)) != EOK) goto error;
				} else {
					if ((ret = _flash_sectorProgram(mtd, sectorBegin, sectorOffs, buffPtr, chunk)) != EOK) goto error;
				}
			}
		}
		mtd_unlock(mtd->flNo);
 
		buffPtr += chunk;
		offs += chunk;
		len -= chunk;

		sectorOffs = 0;
		sectorBegin += sectorSize;
		sectorNo++;
		if (sectorNo >= mtd->region[reg].sectorCount) {
			reg++;
			sectorSize = mtd->region[reg].sectorSize;
			sectorNo = 0;
		}
	}

	if (data != NULL) {
		vm_kunmap(data);
		vm_pageFree(pages);
	}
	return (char *)buffPtr - buff;

error:
	mtd_unlock(mtd->flNo);
	if (data != NULL) {
		vm_kunmap(data);
		vm_pageFree(pages);
	}
	return ret;
}


static int flash_devRead(file_t* file, offs_t offs, char *buff, unsigned int len)
{
    vnode_t *vnode = file->vnode;
	int ret;
	unsigned int mtd;

	if (vnode == NULL)
		return -EINVAL;
	if (buff == NULL)
		return -EINVAL;
	mtd = MINOR(vnode->dev);
	if (mtd >= mtdsCount)
		return -ENOENT;
	if ((offs + len) > mtds[mtd].chipSize) len = mtds[mtd].chipSize - offs;
	if (len == 0)
		return EOK;

	mtd_lock(mtds[mtd].flNo);
	ret = _mtd_read(mtds[mtd].flNo, offs, buff, len, NULL);
	mtd_unlock(mtds[mtd].flNo);

	if (ret == EOK)
		return len;
	else
		return ret;
}


static int flash_devIOCtl(file_t* file, unsigned int cmd, unsigned long arg)
{
    vnode_t *vnode = file->vnode;
	int ret, regNo;
	unsigned long toErase;
	mtd_t *mtd;
	flash_info_t *fli = (flash_info_t *) arg;

	if (vnode == NULL)
		return -EINVAL;
	if (MINOR(vnode->dev) >= mtdsCount)
		return -ENODEV;
	mtd = &mtds[MINOR(vnode->dev)];

	switch(cmd) {
		case FLASH_IOC_INFO_GET:
			if ((void *)arg == NULL)
				return -EINVAL;
			fli->pageSize = mtd->pageSize;
			fli->regionCount = mtd->regionsCount;
			for (regNo = 0; regNo < mtd->regionsCount; regNo++) {
				fli->blocks[regNo].count = mtd->region[regNo].sectorCount;
				fli->blocks[regNo].size =  mtd->region[regNo].sectorSize;
			}
			return EOK;
		case FLASH_IOC_ERASE:
			regNo = 0;
			toErase = 0;
			while ((regNo < mtd->regionsCount) && (arg > mtd->region[regNo].sectorCount)) {
				toErase += mtd->region[regNo].sectorSize * mtd->region[regNo].sectorCount;
				arg -= mtd->region[regNo].sectorCount;
				regNo++;
			}
			if (regNo > mtd->regionsCount)
				return -EINVAL;
			toErase += arg * mtd->region[regNo].sectorSize;
			mtd_lock(mtd->flNo);
			ret = _mtd_eraseWithWait(mtd->flNo, toErase);
			mtd_unlock(mtd->flNo);
			return ret;
		case FLASH_IOC_ERASE_ALL:
			mtd_lock(mtd->flNo);
			ret = _mtd_eraseAllWithWait(mtd->flNo);
			mtd_unlock(mtd->flNo);
			return ret;
		default:
			return -EINVAL;
	}
}


int flash_attach(dev_t flash, dev_t mtd)
{
	int ret;
	flash_cfi_t cfi;
	mtd_t *newMtd;
	int reg;
	void *new;

	if (MAJOR(mtd) != MAJOR_MTD)
		return -EINVAL;
	if (MINOR(flash) != mtdsCount)
		return -EINVAL;
	if (MINOR(mtd) >= mtd_getCount())
		return -EINVAL;
	
	mtd_lock(MINOR(flash));
	/* no read, program or erase can be in progress when reading cfi */
	ret = _mtd_getCfi(MINOR(flash), &cfi);
	mtd_unlock(MINOR(flash));
	if (ret != EOK)
		return ret;

	if ((new = vm_krealloc(mtds, sizeof(mtd_t) * (mtdsCount + 1))) == NULL)
		return -ENOMEM;

	mtdsCount++;
	mtds = (mtd_t *) new;
	newMtd = &mtds[MINOR(flash)];


	newMtd->flNo = MINOR(mtd);
	newMtd->chipSize = 1 << cfi.chipSize;
	newMtd->pageSize = 1 << cfi.bufferSize;
	newMtd->regionsCount = cfi.regionsCount;

	for (reg = 0; reg < cfi.regionsCount; reg++) {
		newMtd->region[reg].sectorCount = cfi.region[reg].blockCount + 1;
		if (cfi.region[reg].blockSize == 0)
			newMtd->region[reg].sectorSize = 128;
		else
			newMtd->region[reg].sectorSize = cfi.region[reg].blockSize * 256;
	}
	return EOK;
}


int _flash_init(void)
{
	static const file_ops_t flashOps = {
		.read = flash_devRead,
		.write = flash_devWrite,
		.ioctl = flash_devIOCtl,
	};

	if (dev_register(MAKEDEV(MAJOR_DRIVE, 0), &flashOps) < 0) {
		main_printf(ATTR_ERROR, "dev/flash: Can't register device!\n");
		return -ENOMEM;
	}
	assert(EOK==dev_mknod(MAKEDEV(MAJOR_DRIVE, 0), "flash0"));

	return EOK;
}
