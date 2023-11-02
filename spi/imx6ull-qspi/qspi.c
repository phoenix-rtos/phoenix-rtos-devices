/*
 * Phoenix-RTOS
 *
 * QSPI Controller driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

/*
 * NOTICE: QSPI driver in user space depends on phoenix-rtos-loader (plo) configuration set during
 * flash probing, all settings (clocks, bus setup, command lookup-table, etc) are done in plo not in
 * user space, this driver when used without `plo/devices/flash-imx6ull` will not work properly.
 */

#include <stdint.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "imx6ull-qspi.h"


#define QSPI_LUT (0x310u / sizeof(uint32_t))

#define QSPI_IPCR (0x08u / sizeof(uint32_t))

#define QSPI_FLSHCR (0x0Cu / sizeof(uint32_t))

#define QSPI_MCR          (0u)
#define QSPI_MCR_CLR_TXF  (0x01u << 11)
#define QSPI_MCR_CLR_RXF  (0x01u << 10)
#define QSPI_MCR_CLR_MDIS (0x01u << 14)
#define QSPI_MCR_SWRSTSD  (0x01u)
#define QSPI_MCR_SWRSTHD  (0x01u << 1)

#define QSPI_BFGENCR (0x20u / sizeof(uint32_t))

#define QSPI_SPTRCLR     (0x16cu / sizeof(uint32_t))
#define QSPI_SPTRCLR_IP  (1u << 8)
#define QSPI_SPTRCLR_AHB (1u)

#define QSPI_BUFIND(n) ((0x30u + (4u * (n))) / sizeof(uint32_t))

#define QSPI_BUFCR(n) ((0x10u + (4u * (n))) / sizeof(uint32_t))

#define QSPI_BUFCR_INVALID_MASTER (0xeu)

#define QSPI_SR        (0x15cu / sizeof(uint32_t))
#define QSPI_SR_BUSY   (0x01u)
#define QSPI_SR_RXWE   (0x01u << 16)
#define QSPI_SR_TXEDA  (0x01u << 24)
#define QSPI_SR_TXFULL (0x01u << 27)
#define QSPI_SR_IP_ACC (0x01u << 1)

#define QSPI_RBDRn(n) ((0x200u + (4u * (n))) / sizeof(uint32_t))

#define QSPI_RBSR (0x10cu / sizeof(uint32_t))

#define QSPI_SFAR (0x100u / sizeof(uint32_t))

#define QSPI_RBCT       (0x110u / sizeof(uint32_t))
#define QSPI_RBCT_RXBRD (0x01u << 8)

#define QSPI_FR       (0x160u / sizeof(uint32_t))
#define QSPI_FR_RBDF  (0x01u << 16)
#define QSPI_FR_IPIEF (0x01u << 6)
#define QSPI_FR_IPAEF (0x01u << 7)

#define QSPI_TBDR (0x154u / sizeof(uint32_t))

#define QSPI_SFA1AD (0x180u / sizeof(uint32_t))
#define QSPI_SFA2AD (0x184u / sizeof(uint32_t))
#define QSPI_SFB1AD (0x188u / sizeof(uint32_t))
#define QSPI_SFB2AD (0x18cu / sizeof(uint32_t))

#define QSPI_MMAP_BASE 0x60000000u
#define QSPI_BASE      0x21e0000u

#define WATERMARK 16u


static addr_t qspi_getAddressByPort(qspi_t *qspi, uint8_t port, addr_t addr)
{
	uint32_t reg;

	/* QSPI use the port (chip select) based on an offset of each memory size */
	switch (port) {
		case 0:
			reg = qspi->ahbAddr;
			break;
		case 1:
			reg = qspi->base[QSPI_SFA1AD];
			break;
		case 2:
			reg = qspi->base[QSPI_SFA2AD];
			break;
		case 3:
			reg = qspi->base[QSPI_SFB1AD];
			break;
		default:
			reg = 0;
			break;
	}
	reg &= ~(0x3ff);

	return addr + reg;
}


static void qspi_setIPCR(qspi_t *qspi, unsigned int seq_num, size_t idatsz)
{
	uint32_t reg = qspi->base[QSPI_IPCR];

	reg = (reg & ~(0x0fu << 24)) | ((seq_num & 0x0fu) << 24);
	reg = (reg & ~(0xffffu)) | (idatsz & 0xffffu);


	qspi->base[QSPI_SPTRCLR] |= QSPI_SPTRCLR_IP;
	qspi->base[QSPI_IPCR] = reg;
}


static ssize_t qspi_opRead(qspi_t *qspi, struct xferOp *xfer)
{
	unsigned int i;
	size_t byte, len = 0;
	uint32_t reg;
	time_t retries = 0;

	do {
		if ((xfer->timeout > 0u) && (retries >= xfer->timeout)) {
			return -ETIME;
		}
		retries++;

		qspi->base[QSPI_RBCT] = ((qspi->base[QSPI_RBCT] & ~(0x1fu)) | WATERMARK);
		qspi->base[QSPI_SFAR] = xfer->addr;
		qspi->base[QSPI_MCR] |= QSPI_MCR_CLR_RXF;

		qspi_setIPCR(qspi, xfer->seqIdx, xfer->data.read.sz);
	} while ((qspi->base[QSPI_FR] & (QSPI_FR_IPAEF | QSPI_FR_IPIEF)) != 0u);

	retries = 0;
	while (len < xfer->data.read.sz) {
		do {
			if ((xfer->timeout > 0u) && (retries >= xfer->timeout)) {
				return -ETIME;
			}
			retries++;

			reg = qspi->base[QSPI_SR];
		} while (((reg & QSPI_SR_RXWE) == 0u) && ((reg & QSPI_SR_BUSY) != 0u));

		for (i = 0; (i <= WATERMARK) && (len < xfer->data.read.sz); i++) {
			reg = qspi->base[QSPI_RBDRn(i)];

			for (byte = 0; (byte < 4u) && ((len + byte) < xfer->data.read.sz); byte++) {
				((uint8_t *)xfer->data.read.ptr)[len + byte] = reg & 0xffu;
				reg >>= 8;
			}
			len += byte;
		}
		qspi->base[QSPI_FR] |= QSPI_FR_RBDF; /* Buffer pop. */
	}

	return len;
}


static size_t qspi_writeTx(qspi_t *qspi, const void *data, size_t size)
{
	size_t byte, sent = 0;
	int align64 = 0;
	uint32_t reg;

	while (((qspi->base[QSPI_SR] & QSPI_SR_TXFULL) == 0u) && (sent < size)) {
		reg = 0;
		for (byte = 0; (byte < 4u) && ((sent + byte) < size); byte++) {
			reg |= ((const uint8_t *)data)[sent + byte] << (8u * byte);
		}
		sent += byte;

		qspi->base[QSPI_TBDR] = reg;
		align64 += 4;
	}
	for (; ((align64 % 64) != 0) && (qspi->base[QSPI_SR] & QSPI_SR_TXFULL) == 0u; align64 += 4) {
		qspi->base[QSPI_TBDR] = 0;
	}

	return sent;
}


static ssize_t qspi_opWrite(qspi_t *qspi, struct xferOp *xfer)
{
	size_t sent = 0;
	time_t retries = 0;

	do {
		if ((xfer->timeout > 0u) && (retries >= xfer->timeout)) {
			return -ETIME;
		}
		retries++;

		qspi->base[QSPI_MCR] |= QSPI_MCR_CLR_TXF;

		qspi->base[QSPI_SFAR] = xfer->addr;

		sent += qspi_writeTx(qspi, xfer->data.write.ptr, xfer->data.write.sz);

		qspi_setIPCR(qspi, xfer->seqIdx, xfer->data.write.sz);
	} while ((qspi->base[QSPI_FR] & (QSPI_FR_IPAEF | QSPI_FR_IPIEF)) != 0u);

	retries = 0;
	while (sent < xfer->data.write.sz) {
		if ((xfer->timeout > 0u) && (retries >= xfer->timeout)) {
			return -ETIME;
		}
		retries++;

		sent += qspi_writeTx(qspi, ((const uint8_t *)xfer->data.write.ptr) + sent, xfer->data.write.sz - sent);
	}

	return sent;
}


ssize_t qspi_xferExec(qspi_t *qspi, struct xferOp *xfer)
{
	ssize_t res;
	time_t retries = 0;

	xfer->addr = qspi_getAddressByPort(qspi, xfer->port, xfer->addr);

	switch (xfer->op) {
		case xfer_opWrite: /* IP write is limited to IPDATSZ mask */
			if (xfer->data.write.sz > 0xffffu) {
				return -EPERM;
			}
			res = qspi_opWrite(qspi, xfer);
			break;
		case xfer_opRead:
			if (xfer->data.read.sz > 0xffffu) {
				/* Clear buffers. */
				qspi->base[QSPI_MCR] |= (QSPI_MCR_CLR_RXF | QSPI_MCR_CLR_TXF);
				(void)memcpy(xfer->data.read.ptr, (const void *)(qspi->ahbBase + (xfer->addr - qspi->ahbAddr)), xfer->data.read.sz);
				return xfer->data.read.sz;
			}
			res = qspi_opRead(qspi, xfer);
			break;
		case xfer_opCommand:
			xfer->data.read.sz = 0;
			xfer->data.read.ptr = NULL;
			res = qspi_opRead(qspi, xfer);
			break;
		default:
			return -EINVAL;
	}

	/* Wait for IP command complete */
	while ((qspi->base[QSPI_SR] & QSPI_SR_BUSY) != 0u) {
		if ((xfer->timeout > 0u) && (retries >= xfer->timeout)) {
			qspi->base[QSPI_MCR] |= (QSPI_MCR_CLR_RXF | QSPI_MCR_CLR_TXF);
			return -ETIME;
		}
		retries++;
	}
	qspi->base[QSPI_MCR] |= (QSPI_MCR_CLR_RXF | QSPI_MCR_CLR_TXF);
	return res;
}


int qspi_init(qspi_t *qspi)
{
	size_t ahbSize;

	qspi->base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, QSPI_BASE);
	if (qspi->base == MAP_FAILED) {
		return -ENOMEM;
	}

	qspi->ahbAddr = QSPI_MMAP_BASE;
	ahbSize = qspi->base[QSPI_SFB2AD] - qspi->ahbAddr;
	qspi->ahbBase = mmap(NULL, ahbSize, PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, qspi->ahbAddr);
	if (qspi->ahbBase == MAP_FAILED) {
		(void)munmap((void *)qspi->base, _PAGE_SIZE);
		return -ENOMEM;
	}

	qspi->slFlashSz[0] = qspi->base[QSPI_SFA1AD] - qspi->ahbAddr;
	qspi->slFlashSz[1] = qspi->base[QSPI_SFA2AD] - qspi->base[QSPI_SFA1AD];
	qspi->slFlashSz[2] = qspi->base[QSPI_SFB1AD] - qspi->base[QSPI_SFA2AD];
	qspi->slFlashSz[3] = qspi->base[QSPI_SFB2AD] - qspi->base[QSPI_SFB1AD];

	return EOK;
}
