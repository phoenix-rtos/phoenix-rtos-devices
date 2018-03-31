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

#ifndef _QUADSPI_H_
#define _QUADSPI_H_

#include <hal/MVF50GS10MK50.h>
#include <dev/flash/flash_cfi.h>

#define QuadSPI0_FLASH_BASE_ADDR 0x20000000
#define QuadSPI1_FLASH_BASE_ADDR 0x50000000

#define QuadSPI_A1 0x01
#define QuadSPI_A2 0x02
#define QuadSPI_B1 0x04
#define QuadSPI_B2 0x08

#define QuadSPI_0  0x10
#define QuadSPI_1  0x20

#define QuadSPI0_A1 (QuadSPI_A1 | QuadSPI_0)
#define QuadSPI0_A2 (QuadSPI_A2 | QuadSPI_0)
#define QuadSPI0_B1 (QuadSPI_B1 | QuadSPI_0)
#define QuadSPI0_B2 (QuadSPI_B2 | QuadSPI_0)
#define QuadSPI1_A1 (QuadSPI_A1 | QuadSPI_1)

#define SEQID_WREN    0
#define SEQID_DDRQIOR 1
#define SEQID_QIOR    2
#define SEQID_QPP     3
#define SEQID_PP      4
#define SEQID_RDSR1   5
#define SEQID_RDCR    6
#define SEQID_WRR     7
#define SEQID_BE      8
#define SEQID_SE      9
#define SEQID_P4E    10
#define SEQID_RDID   11
#define SEQID_CLSR   12

#define RX_BUF_SIZE 0x80
#define CFI_TX_BUF_SIZE 6
#define TX_BUF_SIZE (1 << CFI_TX_BUF_SIZE)
#define DEFAULT_TRANSFER_SIZE 0


static inline u32 _quadspi_isBusy(QuadSPI_Type *quadspi)
{
	return (quadspi->SR & (QuadSPI_SR_AHBGNT_MASK | QuadSPI_SR_AHB_ACC_MASK | QuadSPI_SR_IP_ACC_MASK));
}


static inline void _quadspi_doIpSeq(QuadSPI_Type *quadspi, u32 seqid, u16 transferSize)
{
	quadspi->IPCR = seqid << 24 | transferSize;
}


static inline void _quadspi_setFlashAddr(QuadSPI_Type *quadspi, u32 addr)
{
	quadspi->SFAR = addr;
}


static inline u16 _quadspi_RXfillLevel(QuadSPI_Type *quadspi)
{
	return (quadspi->RBSR & QuadSPI_RBSR_RDBFL_MASK) >> QuadSPI_RBSR_RDBFL_SHIFT;
}


static inline void _quadspi_clrRX(QuadSPI_Type *quadspi)
{
	quadspi->MCR |= QuadSPI_MCR_CLR_RXF_MASK;
}


static inline void _quadspi_clrTX(QuadSPI_Type *quadspi)
{
	quadspi->MCR |= QuadSPI_MCR_CLR_TXF_MASK;
	quadspi->FR = QuadSPI_FR_TBUF_MASK;
}


extern void _quadspi_setFlashSizes(QuadSPI_Type *quadspi, u32 base, u32 a1Size, u32 a2Size, u32 b1Size, u32 b2Size);
extern int _quadspi_init(QuadSPI_Type *quadspi, u8 connectedFlashes, int initDdr);
extern int _quadspi_compareFlashData(QuadSPI_Type *quadspi, u32 seqid, char *dest, unsigned int len, char **firstDiff, char **lastDiff);

extern int _quadspi_receive4FlashData(QuadSPI_Type *quadspi, u32 seqid, u32 *dest);
extern int _quadspi_receiveFlashData(QuadSPI_Type *quadspi, u32 seqid, char *dest, unsigned int len);
extern int _quadspi_receiveFlashDataAlgn(QuadSPI_Type *quadspi, u32 seqid, u32* dest, unsigned int len);

extern int _quadspi_send4FlashData(QuadSPI_Type *quadspi, u32 seqid, u32 src);
extern int _quadspi_sendFlashData(QuadSPI_Type *quadspi, u32 seqid, const char *src, unsigned int len);
extern int _quadspi_sendFlashDataAlgn(QuadSPI_Type *quadspi, u32 seqid, const u32* src, unsigned int len);

#endif /* __QUADSPI_H__ */
