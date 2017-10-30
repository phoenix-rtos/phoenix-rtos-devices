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

#include <dev/flash/vybridF/quadspi.h>
#include <dev/flash/mtd_local_if.h>

#include <include/errno.h>
#include <main/std.h>
#include <vm/kmap.h>


#ifdef FLASH_S25FL128S
	#define FLASH_STATUS_MASK  0x61000000
	#define FLASH_STATUS_READY 0x00000000
	#define FLASH_STATUS_BUSY  0x01000000

	#define SEQID_CFI_READ        SEQID_RDID
	#define SEQID_WRITE_ENABLE    SEQID_WREN
	#define SEQID_READ_STATUS     SEQID_RDSR1
	#define SEQID_FLASH_CONFIGURE SEQID_WRR
	#define SEQID_PROGRAM_PAGE    SEQID_QPP
	#define SEQID_READ            SEQID_DDRQIOR
	#define SEQID_ERASE1          SEQID_P4E
	#define SEQID_ERASE2          SEQID_SE
	#define SEQID_ERASE_UNIFORM   SEQID_SE
	#define SEQID_ERASE_ALL       SEQID_BE
	#define SEQID_CLEAR_ERRORS    SEQID_CLSR

	#define ERASE_CHANGE 0x20000
	#define INIT_FLASH_CONFIG 0x00020000 /* enable flash quad mode */
	#define INIT_QUADSPI_DDR
#endif
   

#ifdef FLASH_S25FL132K
	#define FLASH_STATUS_MASK  0x01000000
	#define FLASH_STATUS_READY 0x00000000
	#define FLASH_STATUS_BUSY  0x01000000

	#define SEQID_CFI_READ        SEQID_RDID
	#define SEQID_WRITE_ENABLE    SEQID_WREN
	#define SEQID_READ_STATUS     SEQID_RDSR1
	#define SEQID_FLASH_CONFIGURE SEQID_WRR
	#define SEQID_PROGRAM_PAGE    SEQID_PP
	#define SEQID_READ            SEQID_QIOR
	#define SEQID_ERASE1          SEQID_P4E
	#define SEQID_ERASE2          SEQID_SE
	#define SEQID_ERASE_UNIFORM   SEQID_SE
	#define SEQID_ERASE_ALL       SEQID_BE
	#define SEQID_CLEAR_ERRORS    SEQID_CLSR

	#define ERASE_CHANGE 0x20000
	#define INIT_FLASH_CONFIG 0x00020000 /* enable flash quad mode */
#endif


#ifdef FLASH_S25FL032P
	#define FLASH_STATUS_MASK  0x61000000
	#define FLASH_STATUS_READY 0x00000000
	#define FLASH_STATUS_BUSY  0x01000000

	#define SEQID_CFI_READ        SEQID_RDID
	#define SEQID_WRITE_ENABLE    SEQID_WREN
	#define SEQID_READ_STATUS     SEQID_RDSR1
	#define SEQID_FLASH_CONFIGURE SEQID_WRR
	#define SEQID_PROGRAM_PAGE    SEQID_QPP
	#define SEQID_READ            SEQID_QIOR
	#define SEQID_ERASE1          SEQID_P4E
	#define SEQID_ERASE2          SEQID_SE
	#define SEQID_ERASE_UNIFORM   SEQID_SE
	#define SEQID_ERASE_ALL       SEQID_BE
	#define SEQID_CLEAR_ERRORS    SEQID_CLSR

	#define ERASE_CHANGE 0x20000
	#define INIT_FLASH_CONFIG 0x00020000 /* enable flash quad mode */
#endif


#ifdef CONFIG_FLASH_QUADSPI0_A1
#define COUNT_QUADSPI0_A1 1
#else
#define COUNT_QUADSPI0_A1 0
#endif
#ifdef CONFIG_FLASH_QUADSPI0_A2
#define COUNT_QUADSPI0_A2 1
#else
#define COUNT_QUADSPI0_A2 0
#endif
#ifdef CONFIG_FLASH_QUADSPI0_B1
#define COUNT_QUADSPI0_B1 1
#else
#define COUNT_QUADSPI0_B1 0
#endif
#ifdef CONFIG_FLASH_QUADSPI0_B2
#define COUNT_QUADSPI0_B2 1
#else
#define COUNT_QUADSPI0_B2 0
#endif
#ifdef CONFIG_FLASH_QUADSPI1_A1
#define COUNT_QUADSPI1_A1 1
#else
#define COUNT_QUADSPI1_A1 0
#endif

#define QUADSPI0_COUNT (COUNT_QUADSPI0_A1 + COUNT_QUADSPI0_A2 + COUNT_QUADSPI0_B1 + COUNT_QUADSPI0_B2)
#define QUADSPI1_COUNT COUNT_QUADSPI1_A1


#define FAKE_FLASH_SIZE 0x100000
#define TRUE 1
#define FALSE 0

#if QUADSPI0_COUNT > 0
static mutex_t QuadSPI0Lock;
#endif
#if QUADSPI1_COUNT > 0
static mutex_t QuadSPI1Lock;
#endif


static struct flash_chip_t {
	mutex_t *mutex;
	QuadSPI_Type *quadspi;
	u32 baseAddr;
} chips[QUADSPI0_COUNT + QUADSPI1_COUNT];


static void _mtd_clearErrors(QuadSPI_Type *quadspi)
{
	while(_quadspi_isBusy(quadspi));
	_quadspi_doIpSeq(quadspi, SEQID_CLEAR_ERRORS, 0);
}

static int _mtd_getFlashStatus(QuadSPI_Type *quadspi, u32 flashAddress)
{
	u32 statusValue;
	int ret;

	while(_quadspi_isBusy(quadspi));
	_quadspi_setFlashAddr(quadspi, flashAddress);
	ret = _quadspi_receive4FlashData(quadspi, SEQID_READ_STATUS, &statusValue);

	if (ret != EOK) return ret;
	if ((statusValue & FLASH_STATUS_MASK) == FLASH_STATUS_READY)
		return EOK;
	else if ((statusValue & FLASH_STATUS_MASK) == FLASH_STATUS_BUSY)
		return -EBUSY;
	else {
		_mtd_clearErrors(quadspi);
		return -EREMOTEIO;
	}
}


static int _mtd_readCfi(QuadSPI_Type *quadspi, u32 flashAddress, flash_cfi_t *cfi)
{
	int ret;
	while(_quadspi_isBusy(quadspi));
	_quadspi_setFlashAddr(quadspi, flashAddress);
	if ((ret = _quadspi_receiveFlashData(quadspi, SEQID_CFI_READ, (char *)cfi, sizeof(*cfi))) != EOK) return ret;
	if(cfi->vendorSpecific[0] == 1 && cfi->vendorSpecific[1] == 0x40 && cfi->vendorSpecific[2] == 0x16){
		main_printf(ATTR_INFO, "dev/mtd: S25LF132K detected\n");
		/* setup only parts of CFI used by our code! */
		cfi->chipSize = cfi->vendorSpecific[2];
		cfi->bufferSize = 0x8; //program up to 256 bytes at once
		cfi->regionsCount = 2;
		cfi->timeoutTypical.byteWrite = 0xb;
		cfi->timeoutTypical.bufferWrite = 0xb;
		cfi->timeoutTypical.sectorErase = 0x9;
		cfi->timeoutTypical.chipErase = 0xf;
		
		cfi->region[0].blockCount = 31 ;
		cfi->region[0].blockSize = 4*1024 / 256;
		cfi->region[1].blockCount = (1<<cfi->chipSize)/(64*1024) - 1 - 2;
		cfi->region[1].blockSize = 64*1024 / 256;
	}else if ((cfi->qry[0] != 'Q') || (cfi->qry[1] != 'R') || (cfi->qry[2] != 'Y')){
		main_printf(ATTR_INFO, "flash ID: %c%c%c\n",cfi->qry[0], cfi->qry[1], cfi->qry[2]);
	       	return -EIO;
	}
	cfi->bufferSize = ((cfi->bufferSize < CFI_TX_BUF_SIZE) ? cfi->bufferSize : CFI_TX_BUF_SIZE);
	return ret;
}


static int _mtd_flashInit(QuadSPI_Type *quadspi, u32 flashAddress)
{
	int ret = EOK;

#ifdef SEQID_FLASH_CONFIGURE
	while(_quadspi_isBusy(quadspi));
	_quadspi_setFlashAddr(quadspi, flashAddress);
	_quadspi_doIpSeq(quadspi, SEQID_WRITE_ENABLE, 0);
	while(_quadspi_isBusy(quadspi));
	if ((ret = _quadspi_send4FlashData(quadspi, SEQID_FLASH_CONFIGURE, INIT_FLASH_CONFIG)) != EOK) return ret;
	while((ret = _mtd_getFlashStatus(quadspi, flashAddress)) == -EBUSY);
#endif

	return ret;
}


static int _mtd_initQuadspiFlashes(QuadSPI_Type *quadspi, mutex_t *mutex, u32 flashBase, u8 flashes, int chipsCount, int firstChip)
{
	flash_cfi_t cfi;
	u32 flashAddr = flashBase;
	int chipSize[4] = {0, 0, 0, 0};
	int chipNo, chipSlot;
	int ret;

	if (flashes & QuadSPI_A1) chipSize[0] = FAKE_FLASH_SIZE;
	if (flashes & QuadSPI_A2) chipSize[1] = FAKE_FLASH_SIZE;
	if (flashes & QuadSPI_B1) chipSize[2] = FAKE_FLASH_SIZE;
	if (flashes & QuadSPI_B2) chipSize[3] = FAKE_FLASH_SIZE;

#ifdef INIT_QUADSPI_DDR
	if ((ret = _quadspi_init(quadspi, flashes, TRUE)) != EOK) {
		main_printf(ATTR_ERROR, "quadspi initialization failed\n");
		return ret;
	}
#else
	if ((ret = _quadspi_init(quadspi, flashes, FALSE)) != EOK) {
		main_printf(ATTR_ERROR, "quadspi initialization failed\n");
		return ret;
	}
#endif

	_quadspi_setFlashSizes(quadspi, flashBase, chipSize[0], chipSize[1], chipSize[2], chipSize[3]);
	for (chipNo = 0, chipSlot = 0; chipNo < chipsCount; chipSlot++) {
		if (chipSize[chipSlot] == 0) continue;

		if ((ret = _mtd_flashInit(quadspi, flashBase + chipNo * FAKE_FLASH_SIZE)) != EOK) {
			main_printf(ATTR_ERROR, "mtd nr %d initialization failed\n", chipNo);
			return ret;
		}
		if ((ret = _mtd_readCfi(quadspi, flashBase + chipNo * FAKE_FLASH_SIZE, &cfi)) != EOK) {
			main_printf(ATTR_ERROR, "mtd nr %d reading cfi failed\n", chipNo);
			return ret;
		}

		chips[chipNo + firstChip].mutex = mutex;
		chips[chipNo + firstChip].quadspi = quadspi;
		chips[chipNo + firstChip].baseAddr = flashAddr;
		chipSize[chipSlot] = 1 << cfi.chipSize;
		flashAddr += 1 << cfi.chipSize;
		
		chipNo++;
	}
	_quadspi_setFlashSizes(quadspi, flashBase, chipSize[0], chipSize[1], chipSize[2], chipSize[3]);

	return EOK;
}


int _mtd_init(void)
{
	QuadSPI_Type *quadspi;
	u8 flashes = 0;
	int ret;

#if QUADSPI0_COUNT > 0
	#ifdef CONFIG_FLASH_QUADSPI0_A1
	flashes |= QuadSPI0_A1;
	#endif
	#ifdef CONFIG_FLASH_QUADSPI0_A2
	flashes |= QuadSPI0_A2;
	#endif
	#ifdef CONFIG_FLASH_QUADSPI0_B1
	flashes |= QuadSPI0_B1;
	#endif
	#ifdef CONFIG_FLASH_QUADSPI0_B2
	flashes |= QuadSPI0_B2;
	#endif

	if (flashes != 0) {
		if ((ret = vm_iomap(QuadSPI0_BASE, sizeof(QuadSPI_Type), PGHD_DEV_RW, (void **) &quadspi)) != EOK) return ret;
		proc_mutexCreate(&QuadSPI0Lock);
		if ((ret = _mtd_initQuadspiFlashes(quadspi, &QuadSPI0Lock, QuadSPI0_FLASH_BASE_ADDR, flashes, QUADSPI0_COUNT, 0)) != EOK)
			return ret;
	}
	flashes = 0;
#endif

#if QUADSPI1_COUNT > 0
	#ifdef CONFIG_FLASH_QUADSPI1_A1
	flashes |= QuadSPI1_A1;
	#endif
	if (flashes != 0) {
		if ((ret = vm_iomap(QuadSPI1_BASE, sizeof(QuadSPI_Type), PGHD_DEV_RW, (void **) &quadspi)) != EOK) return ret;
		proc_mutexCreate(&QuadSPI1Lock);
		return _initQuadspiFlashes(quadspi, &QuadSPI1Lock, QuadSPI1_FLASH_BASE_ADDR, flashes, QUADSPI1_COUNT, QUADSPI0_COUNT);
	}
#endif

	return EOK;
}


int mtd_getCount(void)
{
	return QUADSPI0_COUNT + QUADSPI1_COUNT;
}

int _mtd_checkBusy(unsigned int flNo)
{
	QuadSPI_Type *quadspi = chips[flNo].quadspi;
	if (_quadspi_isBusy(quadspi))
		return 1;
	int ret = _mtd_getFlashStatus(quadspi, chips[flNo].baseAddr);
	if (ret != EOK)
		return 1;
	return 0;
}


int _mtd_getStatus(unsigned int flNo)
{
	int ret;

	proc_mutexLock(chips[flNo].mutex);
	ret = _mtd_getFlashStatus(chips[flNo].quadspi, chips[flNo].baseAddr);
	proc_mutexUnlock(chips[flNo].mutex);

	return ret;
}


int _mtd_getCfi(unsigned int flNo, flash_cfi_t *cfi)
{
	int ret;
	proc_mutexLock(chips[flNo].mutex);
	ret = _mtd_readCfi(chips[flNo].quadspi, chips[flNo].baseAddr, cfi);
	proc_mutexUnlock(chips[flNo].mutex);

	return ret;
}


int _mtd_read(unsigned int flNo, offs_t offs, char *buff, unsigned int len, unsigned int *readedLen)
{
	int ret = EOK;
	u16 toRead;
	QuadSPI_Type *quadspi = chips[flNo].quadspi;

	if (readedLen != NULL)
		*readedLen = 0;
	proc_mutexLock(chips[flNo].mutex);
	while(_quadspi_isBusy(quadspi));
	while ((ret == EOK) && (len > 0)) {
		if (len > RX_BUF_SIZE) {
			toRead = RX_BUF_SIZE;
		} else {
			toRead = len;
		}
		_quadspi_setFlashAddr(quadspi, chips[flNo].baseAddr + offs);
		ret = _quadspi_receiveFlashData(quadspi, SEQID_READ, buff, toRead);
		if ((readedLen != NULL) && (ret == EOK))
			*readedLen += toRead;
		len -= toRead;
		offs += toRead;
		buff += toRead;
	}
	proc_mutexUnlock(chips[flNo].mutex);
	return ret;
}

int _mtd_readQuick(unsigned int flNo, u32* buff, offs_t offs, unsigned int len)
{
	int ret;
	QuadSPI_Type *quadspi = chips[flNo].quadspi;
	_quadspi_setFlashAddr(quadspi, chips[flNo].baseAddr + offs);
	ret = _quadspi_receiveFlashDataAlgn(quadspi, SEQID_READ, buff, len);
	return ret;
}


int _mtd_compare(unsigned int flNo, offs_t offs, char *buff, unsigned int len, char **first_diff, char **last_diff)
{
	u16 toRead;
	int ret = MTD_THE_SAME;
	int ret_tmp;
	QuadSPI_Type *quadspi = chips[flNo].quadspi;
	
	proc_mutexLock(chips[flNo].mutex);
	while(_quadspi_isBusy(quadspi));
	while (((ret == MTD_TO_WRITE) || (ret == MTD_THE_SAME)) && (len > 0)) {
		if (len > RX_BUF_SIZE) {
			toRead = RX_BUF_SIZE;
		} else {
			toRead = len;
		}
		_quadspi_setFlashAddr(quadspi, chips[flNo].baseAddr + offs);
		if ((ret_tmp = _quadspi_compareFlashData(quadspi, SEQID_READ, buff, toRead, first_diff, last_diff)) != MTD_THE_SAME) {
			first_diff = NULL;
			ret = ret_tmp;
		}
		len -= toRead;
		offs += toRead;
		buff += toRead;
	}
	proc_mutexUnlock(chips[flNo].mutex);

	return ret;
}


static void _quadspi_prepareFlashToChange(QuadSPI_Type *quadspi, offs_t flashAddr)
{
	while(_quadspi_isBusy(quadspi));
	_quadspi_setFlashAddr(quadspi, flashAddr);
	_quadspi_doIpSeq(quadspi, SEQID_WRITE_ENABLE, 0);
}


int _mtd_programPage(unsigned int flNo, offs_t offs, const char *buff, unsigned int len, unsigned int *programmedLen)
{
	int ret;
	QuadSPI_Type *quadspi = chips[flNo].quadspi;

	proc_mutexLock(chips[flNo].mutex);
	_quadspi_prepareFlashToChange(quadspi, chips[flNo].baseAddr + offs);
	ret = _quadspi_sendFlashData(quadspi, SEQID_PROGRAM_PAGE, buff, len);
	proc_mutexUnlock(chips[flNo].mutex);

	if (programmedLen != NULL)
		*programmedLen = (ret == EOK) ? len : 0;
	return ret;
}

int _mtd_programPageQuick(unsigned int flNo, const u32* buff, offs_t offs, unsigned int len)
{
	int ret;
	QuadSPI_Type *quadspi = chips[flNo].quadspi;
	_quadspi_setFlashAddr(quadspi, chips[flNo].baseAddr + offs);
	_quadspi_doIpSeq(quadspi, SEQID_WRITE_ENABLE, 0);
	ret = _quadspi_sendFlashDataAlgn(quadspi, SEQID_PROGRAM_PAGE, buff, len);
	return ret;
}


int _mtd_erase(unsigned int flNo, offs_t offs)
{
	QuadSPI_Type *quadspi = chips[flNo].quadspi;

	proc_mutexLock(chips[flNo].mutex);
	_quadspi_prepareFlashToChange(quadspi, chips[flNo].baseAddr + offs);
	while(_quadspi_isBusy(quadspi));
	if (offs < ERASE_CHANGE) _quadspi_doIpSeq(quadspi, SEQID_ERASE1, 0);
	else _quadspi_doIpSeq(quadspi, SEQID_ERASE2, 0);
	proc_mutexUnlock(chips[flNo].mutex);

	return EOK;
}


int _mtd_eraseUniform(unsigned int flNo, offs_t offs)
{
	QuadSPI_Type *quadspi = chips[flNo].quadspi;

	proc_mutexLock(chips[flNo].mutex);
	_quadspi_prepareFlashToChange(quadspi, chips[flNo].baseAddr + offs);
	while(_quadspi_isBusy(quadspi));
	_quadspi_doIpSeq(quadspi, SEQID_ERASE_UNIFORM, 0);
	proc_mutexUnlock(chips[flNo].mutex);

	return EOK;
}


int _mtd_eraseAll(unsigned int flNo)
{
	QuadSPI_Type *quadspi = chips[flNo].quadspi;

	proc_mutexLock(chips[flNo].mutex);
	_quadspi_prepareFlashToChange(quadspi, chips[flNo].baseAddr);
	while(_quadspi_isBusy(quadspi));
	_quadspi_doIpSeq(quadspi, SEQID_ERASE_ALL, 0);
	proc_mutexUnlock(chips[flNo].mutex);

	return EOK;
}
