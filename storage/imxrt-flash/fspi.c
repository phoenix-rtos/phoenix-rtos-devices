/*
 * Phoenix-RTOS
 *
 * FlexSPI Controller driver
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include "fspi.h"


/*
 * NOTICE: FlexSPI driver in user space depends on phoenix-rtos-loader (plo) configuration set during
 * flash probing, all settings (clocks, bus setup, command lookup-table, etc) are done in plo not in
 * user space (as imxrt-flash/flashsrv), this driver when used without `plo/devices/flash-flexspi`
 * will not work properly.
 */


/* clang-format off */

enum { mcr0 = 0, mcr1, mcr2, ahbcr, inten, intr, lutkey, lutcr, ahbrxbuf0cr0, ahbrxbuf1cr0, ahbrxbuf2cr0, ahbrxbuf3cr0,
    ahbrxbuf4cr0, ahbrxbuf5cr0, ahbrxbuf6cr0, ahbrxbuf7cr0, flsha1cr0 = 24, flsha2cr0, flshb1cr0, flshb2cr0, flsha1cr1,
    flsha2cr1, flshb1cr1, flshb2cr1, flsha1cr2, flsha2cr2, flshb1cr2, flshb2cr2, flshcr4 = 37, ipcr0 = 40, ipcr1,
    ipcmd = 44, iprxfcr = 46, iptxfcr, dllacr, dllbcr, misccr4 = 52, misccr5, misccr6, misccr7, sts0 = 56, sts1, sts2,
    ahbspndsts, iprxfsts, iptxfsts, rfdr32 = 64, tfdr32 = 96, lut64 = 128, hmstr0cr = 256, hmstr1cr, hmstr2cr,
    hmstr3cr, hmstr4cr, hmstr5cr, hmstr6cr, hmstr7cr, haddrstart, haddrend, haddroffset };

/* clang-format on */

void flexspi_swreset(flexspi_t *fspi)
{
	*(fspi->base + mcr0) |= 1;
	while ((*(fspi->base + mcr0) & 1) != 0)
		;
}


static addr_t flexspi_getAddressByPort(flexspi_t *fspi, uint8_t port, addr_t addr)
{
	unsigned int i;

	/* FlexSPI use the port (chip select) based on an offset of each memory size */
	for (i = 0; i < port; ++i) {
		if (fspi->slPortMask & (1 << i)) {
			addr += fspi->slFlashSz[i];
		}
	}

	return addr;
}


static int flexspi_checkFlags(flexspi_t *fspi)
{
	uint32_t flags = *(fspi->base + intr) & ((1 << 1) | (1 << 3) | (1 << 11));

	if (flags != 0) {
		/* Clear flags */
		*(fspi->base + intr) |= flags;

		/* Reset FIFOs */
		*(fspi->base + iptxfcr) |= 1;
		*(fspi->base + iprxfcr) |= 1;

		/* Command grant or sequence execution timeout */
		if (flags & ((1 << 11) | (1 << 1))) {
			return -ETIME;
		}

		return -EIO;
	}

	return EOK;
}


static ssize_t flexspi_opRead(flexspi_t *fspi, time_t start, struct xferOp *xfer)
{
	int res;
	size_t cnt, ofs, len = xfer->data.read.sz, size = xfer->data.read.sz & ~7;
	uint8_t *buf = xfer->data.read.ptr;

	/* note: FlexSPI FIFO watermark level is 64bit aligned */
	for (ofs = 0; ofs < size; ofs += 8, buf += 8, len -= 8) {
		/* Wait for rx FIFO available */
		for (cnt = 0; (*(fspi->base + intr) & (1 << 5)) == 0; ++cnt) {
			res = flexspi_checkFlags(fspi);
			if (res != EOK) {
				return res;
			}
			else if (xfer->timeout > 0 && cnt > 10000000) {
				return -ETIME;
			}
		}

		((uint32_t *)buf)[0] = (fspi->base + rfdr32)[0];
		((uint32_t *)buf)[1] = (fspi->base + rfdr32)[1];

		/* Move FIFO pointer to watermark level */
		*(fspi->base + intr) |= 1 << 5;
	}

	if (ofs < xfer->data.read.sz) {
		size_t cnt = 0;
		/* Wait for rx FIFO available */
		for (cnt = 0; (*(fspi->base + intr) & (1 << 5)) == 0; ++cnt) {
			res = flexspi_checkFlags(fspi);
			if (res != EOK) {
				return res;
			}
			else if (xfer->timeout > 0 && cnt > 10000000) {
				return -ETIME;
			}
		}

		for (ofs = rfdr32; len > 0; ++ofs) {
			uint32_t tmp = *(fspi->base + ofs); /* is volatile! */
			size = len < sizeof(tmp) ? len : sizeof(tmp);
			memcpy(buf, &tmp, size);
			len -= size;
			buf += size;
		}
		/* Move FIFO pointer to watermark level */
		*(fspi->base + intr) |= 1 << 5;
	}

	/* FIXME: delay of ~27us */
	for (uint32_t i = 0x10000; i > 0; --i) {
		asm volatile("nop");
	}

	/* Reset rx FIFO */
	*(fspi->base + iprxfcr) |= 1;

	return buf - (uint8_t *)xfer->data.read.ptr;
}


static ssize_t flexspi_opWrite(flexspi_t *fspi, time_t start, struct xferOp *xfer)
{
	int res;
	size_t cnt, ofs, len = xfer->data.write.sz, size = xfer->data.write.sz & ~7;
	const uint8_t *buf = xfer->data.write.ptr;

	/* note: FlexSPI FIFO watermark level is 64bit aligned */
	for (ofs = 0; ofs < size; ofs += 8, buf += 8, len -= 8) {
		/* Wait for tx FIFO available */
		for (cnt = 0; (*(fspi->base + intr) & (1 << 6)) == 0; ++cnt) {
			res = flexspi_checkFlags(fspi);
			if (res != EOK) {
				return res;
			}
			else if (xfer->timeout > 0 && cnt > 10000000) {
				return -ETIME;
			}
		}

		(fspi->base + tfdr32)[0] = ((uint32_t *)buf)[0];
		(fspi->base + tfdr32)[1] = ((uint32_t *)buf)[1];

		/* Move tx FIFO pointer to watermark level */
		*(fspi->base + intr) |= 1 << 6;
	}

	if (ofs < xfer->data.write.sz) {
		/* Wait for tx FIFO available */
		for (cnt = 0; (*(fspi->base + intr) & (1 << 6)) == 0; ++cnt) {
			res = flexspi_checkFlags(fspi);
			if (res != EOK) {
				return res;
			}
			else if (xfer->timeout > 0 && cnt > 10000000) {
				return -ETIME;
			}
		}

		for (ofs = tfdr32; len > 0; ++ofs) {
			uint32_t tmp;
			size = len < sizeof(tmp) ? len : sizeof(tmp);
			memcpy(&tmp, buf, size);
			*(fspi->base + ofs) = tmp;
			len -= size;
			buf += size;
		}
		/* Move FIFO pointer to watermark level */
		*(fspi->base + intr) |= 1 << 6;
	}

	/* Reset tx FIFO */
	*(fspi->base + iptxfcr) |= 1;

	return buf - (uint8_t *)xfer->data.write.ptr;
}


ssize_t flexspi_xferExec(flexspi_t *fspi, struct xferOp *xfer)
{
	uint32_t dataSize, cnt;
	time_t start = 0;

	if (xfer->op == xfer_opRead) {
		/* For >64k read out the data directly from the AHB buffer (data may be cached) */
		if (xfer->data.read.sz > 0xffff) {
			memcpy(xfer->data.read.ptr, fspi->ahbAddr + xfer->addr, xfer->data.read.sz);
			return EOK;
		}

		dataSize = xfer->data.read.sz & 0xffff;
	}
	else if (xfer->op == xfer_opWrite) {
		/* IP write is limited to IPDATSZ mask */
		if (xfer->data.read.sz > 0xffff) {
			return -EPERM;
		}

		dataSize = xfer->data.write.sz & 0xffff;
	}
	else {
		dataSize = 0;
	}

	/* Wait for AHB & IP bus and sequence controller to be idle */
	for (cnt = 0; ((*(fspi->base + sts0) & 3) ^ 3) != 0; ++cnt) {
		if (xfer->timeout > 0 && cnt > 10000000) {
			return -ETIME;
		}
	}

	/* Clear the instruction pointer */
	*(fspi->base + flsha1cr2 + xfer->port) |= 1u << 31;

	/* Clear any triggered AHB & IP errors and grant timeouts */
	*(fspi->base + intr) |= (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);

	/* Set device's start address of transfer */
	*(fspi->base + ipcr0) = flexspi_getAddressByPort(fspi, xfer->port, xfer->addr);

	/* Reset tx/rx FIFOs, no DMA */
	*(fspi->base + iptxfcr) = (*(fspi->base + iptxfcr) & ~3) | 1;
	*(fspi->base + iprxfcr) = (*(fspi->base + iprxfcr) & ~3) | 1;

	/* Configure sequence index[number] and set xfer data size using "individual" mode */
	*(fspi->base + ipcr1) = dataSize | ((xfer->seqIdx & 0xf) << 16) | ((xfer->seqNum & 0x7) << 24);

	/* Trigger an IP transfer now */
	*(fspi->base + ipcmd) |= 1;

	switch (xfer->op) {
		case xfer_opWrite:
			return flexspi_opWrite(fspi, start, xfer);

		case xfer_opRead:
			return flexspi_opRead(fspi, start, xfer);

		case xfer_opCommand:
			/* fall-through */
		default:
			break;
	}

	/* Wait for IP command complete */
	for (cnt = 0; (*(fspi->base + intr) & 1) == 0; ++cnt) {
		if (xfer->timeout > 0 && cnt > 10000000) {
			return -ETIME;
		}
	}

	/* Acknowledge */
	*(fspi->base + intr) |= 1;

	return flexspi_checkFlags(fspi);
}
