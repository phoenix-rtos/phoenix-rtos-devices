/*
 * Phoenix-RTOS
 *
 * FlexSPI Controller driver
 *
 * Copyright 2021-2023 Phoenix Systems
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
#include <sys/time.h>
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


static time_t flexspi_timerGetMillis(void)
{
	struct timespec ts;
	(void)clock_gettime(CLOCK_MONOTONIC, &ts);
	return (time_t)(((unsigned long long)ts.tv_sec) * 1000uLL + ((unsigned long long)(ts.tv_nsec / 1000000L)));
}


static int flexspi_checkFlags(flexspi_t *fspi)
{
	uint32_t flags = *(fspi->base + intr) & ((1u << 1u) | (1u << 3u) | (1u << 11u));

	if (flags != 0u) {
		/* Clear flags */
		*(fspi->base + intr) |= flags;

		/* Reset FIFOs */
		*(fspi->base + iptxfcr) |= 1u;
		*(fspi->base + iprxfcr) |= 1u;

		/* Command grant or sequence execution timeout */
		if (flags & ((1u << 11u) | (1u << 1u))) {
			return -ETIME;
		}

		return -EIO;
	}

	return EOK;
}


static int flexspi_poll(flexspi_t *fspi, volatile uint32_t *addr, uint32_t mask, uint32_t val, time_t start, time_t timeout, int checkFlags)
{
	int res;
	int retries = 0;

	while ((*addr & mask) != val) {
		if (checkFlags != 0) {
			res = flexspi_checkFlags(fspi);
			if (res != EOK) {
				return res;
			}
		}

		if (retries < FLEXSPI_QUICK_POLL_MAX_RETRIES) {
			/*
			 * Majority of polls finish in less than FLEXSPI_QUICK_POLL_MAX_RETRIES. Doing syscall
			 * or yielding CPU is not optimal for such polls.
			 */
			retries++;
		}
		else {
			if ((timeout > 0uLL) && ((flexspi_timerGetMillis() - start) >= timeout)) {
				return -ETIME;
			}
			flexspi_schedYield(fspi);
		}
	}

	return EOK;
}


static addr_t flexspi_getAddressByPort(flexspi_t *fspi, uint8_t port, addr_t addr)
{
	unsigned int i;

	/* FlexSPI use the port (chip select) based on an offset of each memory size */
	for (i = 0u; i < port; ++i) {
		if (fspi->slPortMask & (1u << i)) {
			addr += fspi->slFlashSz[i];
		}
	}

	return addr;
}


static ssize_t flexspi_opRead(flexspi_t *fspi, time_t start, struct xferOp *xfer)
{
	int res;
	uint8_t *ptr = xfer->data.read.ptr;
	uint8_t *end = ptr + xfer->data.read.sz;

	while (ptr != end) {
		volatile uint8_t *rfdr = (volatile uint8_t *)(fspi->base + rfdr32); /* 2x */

		/* Wait for rx FIFO available */
		res = flexspi_poll(fspi, fspi->base + intr, 1u << 5u, 1u << 5u, start, xfer->timeout, 1);
		if (res != EOK) {
			return res;
		}

		/* FlexSPI FIFO watermark level is 64bit aligned */
		for (size_t n = sizeof(uint64_t); (n != 0u) && (ptr != end); --n) {
			*(ptr++) = *(rfdr++);
		}

		/* Move FIFO pointer to watermark level */
		*(fspi->base + intr) |= 1u << 5u;
	}

	res = flexspi_poll(fspi, fspi->base + sts0, 3u, 3u, start, xfer->timeout, 0);
	if (res != EOK) {
		return res;
	}

	return (ssize_t)xfer->data.read.sz;
}


static ssize_t flexspi_opWrite(flexspi_t *fspi, time_t start, struct xferOp *xfer)
{
	int res;
	const uint8_t *ptr = xfer->data.write.ptr;
	const uint8_t *end = ptr + xfer->data.write.sz;

	while (ptr != end) {
		volatile uint8_t *tfdr = (volatile uint8_t *)(fspi->base + tfdr32); /* 2x */

		/* Wait for tx FIFO available */
		res = flexspi_poll(fspi, fspi->base + intr, 1u << 6u, 1u << 6u, start, xfer->timeout, 1);
		if (res != EOK) {
			return res;
		}

		/* FlexSPI FIFO watermark level is 64bit aligned */
		for (size_t n = sizeof(uint64_t); (n != 0u) && (ptr != end); --n) {
			*(tfdr++) = *(ptr++);
		}

		/* Move tx FIFO pointer to watermark level */
		*(fspi->base + intr) |= 1u << 6u;
	}

	res = flexspi_poll(fspi, fspi->base + sts0, 3u, 3u, start, xfer->timeout, 0);
	if (res != EOK) {
		return res;
	}

	return (ssize_t)xfer->data.write.sz;
}


ssize_t flexspi_xferExec(flexspi_t *fspi, struct xferOp *xfer)
{
	int res;
	uint32_t dataSize;
	time_t start = flexspi_timerGetMillis();

	if (xfer->op == xfer_opRead) {
		/* For >64k read out the data directly from the AHB buffer (data may be cached) */
		if (xfer->data.read.sz > 0xffffu) {
			memcpy(xfer->data.read.ptr, fspi->ahbAddr + xfer->addr, xfer->data.read.sz);
			return (ssize_t)xfer->data.read.sz;
		}

		dataSize = xfer->data.read.sz & 0xffffu;
	}
	else if (xfer->op == xfer_opWrite) {
		/* IP write is limited to IPDATSZ mask */
		if (xfer->data.read.sz > 0xffffu) {
			return -EPERM;
		}

		dataSize = xfer->data.write.sz & 0xffffu;
	}
	else {
		dataSize = 0;
	}

	/* Wait for either AHB & IP bus ready or sequence controller to be idle */
	res = flexspi_poll(fspi, fspi->base + sts0, 3u, 3u, start, xfer->timeout, 0);
	if (res != EOK) {
		return res;
	}

	/* Clear the instruction pointer */
	*(fspi->base + flsha1cr2 + xfer->port) |= 1u << 31u;

	/* Clear any triggered AHB & IP errors and grant timeouts */
	*(fspi->base + intr) |= (1u << 4u) | (1u << 3u) | (1u << 2u) | (1u << 1u);

	/* Set device's start address of transfer */
	*(fspi->base + ipcr0) = flexspi_getAddressByPort(fspi, xfer->port, xfer->addr);

	/* Clear tx/rx FIFOs */
	*(fspi->base + iptxfcr) |= 1u;
	*(fspi->base + iprxfcr) |= 1u;

	/* Configure sequence index[number] and set xfer data size using "individual" mode */
	*(fspi->base + ipcr1) = dataSize | ((xfer->seqIdx & 0xfu) << 16u) | ((xfer->seqNum & 0x7u) << 24u);

	/* Trigger an IP transfer now */
	*(fspi->base + ipcmd) |= 1u;

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
	res = flexspi_poll(fspi, fspi->base + intr, 1u, 1u, start, xfer->timeout, 0);
	if (res != EOK) {
		return res;
	}

	/* Acknowledge */
	*(fspi->base + intr) |= 1u;

	return flexspi_checkFlags(fspi);
}
