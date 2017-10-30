/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * QuadSPI driver
 *
 * Copyright 2014 Phoenix Systems
 *
 * Author: Katarzyna Baranowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <dev/flash/mtd_local_if.h>
#include <dev/flash/vybridF/quadspi.h>
#include <dev/flash/vybridF/lut.h>
#include <include/errno.h>
#include <vm/if.h>


inline static void _quadspi_LUTunlock(QuadSPI_Type *quadspi)
{
	quadspi->LUTKEY = LUT_MAGIC_KEY;
	quadspi->LCKCR = QuadSPI_LCKCR_UNLOCK_MASK;
}


inline static void _quadspi_LUTlock(QuadSPI_Type *quadspi)
{
	quadspi->LUTKEY = LUT_MAGIC_KEY;
	quadspi->LCKCR = QuadSPI_LCKCR_LOCK_MASK;
}


static void _quadspi_LUTfill(QuadSPI_Type *quadspi)
{
	_quadspi_LUTunlock(quadspi);


	quadspi->LUT[(SEQID_DDRQIOR << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD)     |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_DDRQIOR)
										   | QuadSPI_LUT_INSTR1(LUT_CMD_ADDR_DDR)|QuadSPI_LUT_PAD1(LUT_PAD4)|QuadSPI_LUT_OPRND1(24);
	quadspi->LUT[(SEQID_DDRQIOR << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_MODE_DDR)|QuadSPI_LUT_PAD0(LUT_PAD4)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_MODE_ONCE)
										   | QuadSPI_LUT_INSTR1(LUT_CMD_DUMMY)   |QuadSPI_LUT_PAD1(LUT_PAD4)|QuadSPI_LUT_OPRND1(0x06);
	quadspi->LUT[(SEQID_DDRQIOR << 2) + 2] = QuadSPI_LUT_INSTR0(LUT_CMD_READ_DDR)|QuadSPI_LUT_PAD0(LUT_PAD4)|QuadSPI_LUT_OPRND0(0x80)
										   | QuadSPI_LUT_INSTR1(LUT_CMD_STOP)    |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x00);


	quadspi->LUT[(SEQID_QIOR << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD)  |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_QIOR)
										| QuadSPI_LUT_INSTR1(LUT_CMD_ADDR) |QuadSPI_LUT_PAD1(LUT_PAD4)|QuadSPI_LUT_OPRND1(24);
	quadspi->LUT[(SEQID_QIOR << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_MODE) |QuadSPI_LUT_PAD0(LUT_PAD4)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_MODE_ONCE)
										| QuadSPI_LUT_INSTR1(LUT_CMD_DUMMY)|QuadSPI_LUT_PAD1(LUT_PAD4)|QuadSPI_LUT_OPRND1(0x04);
	quadspi->LUT[(SEQID_QIOR << 2) + 2] = QuadSPI_LUT_INSTR0(LUT_CMD_READ) |QuadSPI_LUT_PAD0(LUT_PAD4)|QuadSPI_LUT_OPRND0(0x01)
										| QuadSPI_LUT_INSTR1(LUT_CMD_STOP) |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x00);


	quadspi->LUT[(SEQID_WREN << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD)   |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_WREN)
										| QuadSPI_LUT_INSTR1(LUT_CMD_STOP)  |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x00);


	quadspi->LUT[(SEQID_QPP << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD)  |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_QPP)
									   | QuadSPI_LUT_INSTR1(LUT_CMD_ADDR) |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(24);
	quadspi->LUT[(SEQID_QPP << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_WRITE)|QuadSPI_LUT_PAD0(LUT_PAD4)|QuadSPI_LUT_OPRND0(0x01)
									   | QuadSPI_LUT_INSTR1(LUT_CMD_STOP) |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x00);


	quadspi->LUT[(SEQID_PP << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD)  |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_PP)
									  | QuadSPI_LUT_INSTR1(LUT_CMD_ADDR) |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(24);
	quadspi->LUT[(SEQID_PP << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_WRITE)|QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(0x01)
									  | QuadSPI_LUT_INSTR1(LUT_CMD_STOP) |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x00);


	quadspi->LUT[(SEQID_RDSR1 << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_RDSR1)
										 | QuadSPI_LUT_INSTR1(LUT_CMD_READ)|QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x01);
	quadspi->LUT[(SEQID_RDSR1 << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_STOP)|QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(0x00);


	quadspi->LUT[(SEQID_RDCR << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_RDCR)
										| QuadSPI_LUT_INSTR1(LUT_CMD_READ)|QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x01);
	quadspi->LUT[(SEQID_RDCR << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_STOP)|QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(0x00);


	quadspi->LUT[(SEQID_WRR << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD)  |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_WRR)
									   | QuadSPI_LUT_INSTR1(LUT_CMD_WRITE)|QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x02);
	quadspi->LUT[(SEQID_WRR << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_STOP) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(0x00);


	quadspi->LUT[(SEQID_BE << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_BE)
									  | QuadSPI_LUT_INSTR1(LUT_CMD_STOP)|QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x00);


	quadspi->LUT[(SEQID_P4E << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_P4E)
									   | QuadSPI_LUT_INSTR1(LUT_CMD_ADDR)|QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(24);
	quadspi->LUT[(SEQID_P4E << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_STOP)|QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(0x00);


	quadspi->LUT[(SEQID_SE << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD)  |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_SE)
									  | QuadSPI_LUT_INSTR1(LUT_CMD_ADDR) |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(24);
	quadspi->LUT[(SEQID_SE << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_STOP) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(0x00);
  

	quadspi->LUT[(SEQID_RDID << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_RDID)
										| QuadSPI_LUT_INSTR1(LUT_CMD_READ)|QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(sizeof(flash_cfi_t));
	quadspi->LUT[(SEQID_RDID << 2) + 1] = QuadSPI_LUT_INSTR0(LUT_CMD_STOP)|QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(0x00);


	quadspi->LUT[(SEQID_CLSR << 2) + 0] = QuadSPI_LUT_INSTR0(LUT_CMD_CMD) |QuadSPI_LUT_PAD0(LUT_PAD1)|QuadSPI_LUT_OPRND0(FLASH_SPANSION_CMD_CLSR)
										| QuadSPI_LUT_INSTR1(LUT_CMD_STOP) |QuadSPI_LUT_PAD1(LUT_PAD1)|QuadSPI_LUT_OPRND1(0x00);


	_quadspi_LUTlock(quadspi);
}


static void quadspi_getChars(u32 src, char *dest, unsigned int len)
{
	switch (len) {
		case 4:
			*(dest + 3) = src >> (8 * 0);
		case 3:
			*(dest + 2) = src >> (8 * 1);
		case 2:
			*(dest + 1) = src >> (8 * 2);
		case 1:
			*(dest + 0) = src >> (8 * 3);
		default:
			break;
	}
}


static u32 quadspi_setChars(const char *src, unsigned int len)
{
	u32 res = 0;
	switch (len) {
		case 4:
			res |= ((u32) *(src + 3) & 0x000000FF) << (8 * 0);
		case 3:
			res |= ((u32) *(src + 2) & 0x000000FF) << (8 * 1);
		case 2:
			res |= ((u32) *(src + 1) & 0x000000FF) << (8 * 2);
		case 1:
			res |= ((u32) *(src + 0) & 0x000000FF) << (8 * 3);
		default:
			break;
	}
	return res;
}


int _quadspi_receive4FlashData(QuadSPI_Type *quadspi, u32 seqid, u32 *dest)
{
	_quadspi_clrRX(quadspi);
	_quadspi_doIpSeq(quadspi, seqid, DEFAULT_TRANSFER_SIZE);
	while (_quadspi_isBusy(quadspi));
	if (_quadspi_RXfillLevel(quadspi) != 1) return -EIO;
	*dest = quadspi->RBDR[0];
	return EOK;
}


int _quadspi_receiveFlashData(QuadSPI_Type *quadspi, u32 seqid, char *dest, unsigned int len)
{
	u32 field = 0;

	_quadspi_clrRX(quadspi);
	_quadspi_doIpSeq(quadspi, seqid, len);
	while(_quadspi_isBusy(quadspi));

	while (len > 4) {
		*((u32 *)dest) = hal_cpuReverseBytes(quadspi->RBDR[field]);
		dest += 4;
		len -= 4;
		field++;
	}
	quadspi_getChars(quadspi->RBDR[field], dest, len);

	return EOK;
}

int _quadspi_receiveFlashDataAlgn(QuadSPI_Type *quadspi, u32 seqid, u32* dest, unsigned int len)
{
	u32 field = 0;
	const u32 end_field = len / 4;

	// SPI transfers are much faster than SPI->memory copying,
	// so avoid monitoring buffer level while copying, as this would introduce even more delay.
	_quadspi_clrRX(quadspi);
	_quadspi_doIpSeq(quadspi, seqid, len);
	while (_quadspi_isBusy(quadspi));

	while (field < end_field) {
		*dest++ = hal_cpuReverseBytes(quadspi->RBDR[field]);
		field++;
	}
	return EOK;
}


int _quadspi_compareFlashData(QuadSPI_Type *quadspi, u32 seqid, char *dest, unsigned int len, char **firstDiff, char **lastDiff)
{
	u32 readed = 0;
	int ret = MTD_THE_SAME;
	u32 field = 0;
	unsigned int bytesChecked = 0;

	_quadspi_clrRX(quadspi);
	_quadspi_doIpSeq(quadspi, seqid, len);
	while(_quadspi_isBusy(quadspi));

	while (len > bytesChecked) {
		if (bytesChecked % 4 == 0) {
			readed = quadspi->RBDR[field];
			field++;
		}
		if (*dest != (char)(readed >> (8 * (3 - (bytesChecked % 4))))) {
			if ((*dest & (char)(readed >> (8 * (3 - (bytesChecked % 4))))) == *dest) {
				if (ret == MTD_THE_SAME) {
					if (firstDiff != NULL) *firstDiff = dest;
					ret = MTD_TO_WRITE;
				}
				if (lastDiff != NULL) *lastDiff = dest;
			} else {
				if (firstDiff != NULL) *firstDiff = dest;
				return MTD_TO_ERASE;
			}
		}
		dest++;
		bytesChecked++;
	}
	return ret;
}


int _quadspi_send4FlashData(QuadSPI_Type *quadspi, u32 seqid, u32 src)
{
	_quadspi_clrTX(quadspi);
	quadspi->TBDR = src;
	_quadspi_doIpSeq(quadspi, seqid, DEFAULT_TRANSFER_SIZE);
	return EOK;
}


int _quadspi_sendFlashData(QuadSPI_Type *quadspi, u32 seqid, const char *src, unsigned int len)
{
	unsigned int bytesSent = 0;

	// We can modify TX data queue even when previous SPI op is still ongoing...
	_quadspi_clrTX(quadspi);
	while (bytesSent < len) {
		if (len - bytesSent >= 4) {
			quadspi->TBDR = hal_cpuReverseBytes(*(u32 *)src);
			src += 4;
			bytesSent += 4;
		} else {
			quadspi->TBDR = quadspi_setChars(src, len - bytesSent);
			bytesSent = len;
		}
	}

	// ... but now - make sure any previous operation is completed (e.g. SEQID_WRITE_ENABLE)
	while(_quadspi_isBusy(quadspi)) {};
	_quadspi_doIpSeq(quadspi, seqid, bytesSent);
	return EOK;
}

int _quadspi_sendFlashDataAlgn(QuadSPI_Type *quadspi, u32 seqid, const u32* src, unsigned int len)
{
	const u32* src_end = (void*)src + len;

	// We can modify TX data queue even when previous SPI op is still ongoing...
	_quadspi_clrTX(quadspi);
	while (src < src_end) {
		quadspi->TBDR = hal_cpuReverseBytes(*src++);
	}

	// ... but now - make sure any previous operation is completed (e.g. SEQID_WRITE_ENABLE)
	while(_quadspi_isBusy(quadspi)) {};
	_quadspi_doIpSeq(quadspi, seqid, len);
	return EOK;
}


static int _quadspi_connectClocks(u8 connectedFlashes)
{
	CCM_Type *ccm;
	int ret = EOK;

	if ((ret = vm_iomap(CCM_BASE, sizeof(CCM_Type), PGHD_DEV_RW, (void **) &ccm)) != EOK) return ret;
	if (connectedFlashes & QuadSPI_0) {
		ccm->CCGR2  |= CCM_CCGR2_CG4(1); /* Clock is on during all modes, except stop mode. */
		ccm->CSCMR1 |= CCM_CSCMR1_QSPI0_CLK_SEL(0x3);
		ccm->CSCDR3 |= CCM_CSCDR3_QSPI0_X4_DIV(0x0)|CCM_CSCDR3_QSPI0_X2_DIV_MASK|CCM_CSCDR3_QSPI0_DIV_MASK|CCM_CSCDR3_QSPI0_EN_MASK;
	}
	if (connectedFlashes & QuadSPI_1) {
		ccm->CCGR8  |= CCM_CCGR8_CG4(1); /* Clock is on during all modes, except stop mode. */
		ccm->CSCMR1 |= CCM_CSCMR1_QSPI1_CLK_SEL(0x3);
		ccm->CSCDR3 |= CCM_CSCDR3_QSPI1_X4_DIV(0x0)|CCM_CSCDR3_QSPI1_X2_DIV_MASK|CCM_CSCDR3_QSPI1_DIV_MASK|CCM_CSCDR3_QSPI1_EN_MASK;
	}
	vm_iounmap(ccm, sizeof(CCM_Type));
	return ret;
}


static int _quadspi_initPins(u8 connectedFlashes)
{
	IOMUXC_Type *iomuxc;
	int ret = EOK;

	if ((ret = vm_iomap(IOMUXC_BASE, sizeof(IOMUXC_Type), PGHD_DEV_RW, (void **) &iomuxc)) != EOK) return ret;
	if (connectedFlashes & QuadSPI_0) {
		if (connectedFlashes & (QuadSPI_A1 | QuadSPI_A2)) {
			iomuxc->SINGLE.PTD0 = 0x001030C3;
			iomuxc->SINGLE.PTD2 = 0x001030C3;
			iomuxc->SINGLE.PTD3 = 0x001030C3;
			iomuxc->SINGLE.PTD4 = 0x001030C3;
			iomuxc->SINGLE.PTD5 = 0x001030C3;
			if (connectedFlashes & QuadSPI_A1) {
				iomuxc->SINGLE.PTD1 = 0x001030FF;
			}
			if (connectedFlashes & QuadSPI_A2) {
				iomuxc->SINGLE.PTB6 = 0x001030FF;
			}
		}
		if (connectedFlashes & (QuadSPI_B1 | QuadSPI_B2)) {
			iomuxc->SINGLE.PTD7 = 0x001030C3;
			iomuxc->SINGLE.PTD9 = 0x001030C3;
			iomuxc->SINGLE.PTD10 = 0x001030C3;
			iomuxc->SINGLE.PTD11 = 0x001030C3;
			iomuxc->SINGLE.PTD12 = 0x001030C3;   
			if (connectedFlashes & QuadSPI_B1) {
				iomuxc->SINGLE.PTD8 = 0x001030FF;
			}
			if (connectedFlashes & QuadSPI_B2) {
				iomuxc->SINGLE.PTB7 = 0x001030FF;
			}
		}
	}
	if (connectedFlashes & QuadSPI_1) {
		if (connectedFlashes & QuadSPI_A1) {
			iomuxc->SINGLE.PTA19 = 0x001030C3;
			iomuxc->SINGLE.PTB0 = 0x001030FF;
			iomuxc->SINGLE.PTB1 = 0x001030C3;
			iomuxc->SINGLE.PTB2 = 0x001030C3;
			iomuxc->SINGLE.PTB3 = 0x001030C3;
			iomuxc->SINGLE.PTB4 = 0x001030C3;
		}
	}
	vm_iounmap(iomuxc, sizeof(IOMUXC_Type));
	return ret;
}


void _quadspi_setFlashSizes(QuadSPI_Type *quadspi, u32 base, u32 a1Size, u32 a2Size, u32 b1Size, u32 b2Size)
{
	while(_quadspi_isBusy(quadspi));

	quadspi->SFA1AD = base + a1Size;
	quadspi->SFA2AD = base + a1Size + a2Size;
	quadspi->SFB1AD = base + a1Size + a2Size + b1Size;
	quadspi->SFB2AD = base + a1Size + a2Size + b1Size + b2Size;
}


static void _quadspi_configure(QuadSPI_Type *quadspi, int initDdr)
{
	quadspi->MCR |= QuadSPI_MCR_MDIS_MASK;
	if (initDdr) {
		quadspi->MCR |= QuadSPI_MCR_DDR_EN_MASK;
		quadspi->SMPR = QuadSPI_SMPR_DDRSMP(0x3);
	}
	quadspi->RBCT |= QuadSPI_RBCT_RXBRD_MASK;

	quadspi->MCR &= ~QuadSPI_MCR_MDIS_MASK;
	quadspi->MCR |= QuadSPI_MCR_CLR_TXF_MASK | QuadSPI_MCR_CLR_RXF_MASK;
}


int _quadspi_init(QuadSPI_Type *quadspi, u8 connectedFlashes, int initDdr)
{
	int ret = EOK;
	
	if ((ret = _quadspi_connectClocks(connectedFlashes)) != EOK) return ret;
	if ((ret = _quadspi_initPins(connectedFlashes)) != EOK) return ret;
	_quadspi_LUTfill(quadspi);
	_quadspi_configure(quadspi, initDdr);
	
	return ret;
}
