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


#ifndef _FLEXSPI_H_
#define _FLEXSPI_H_

#if defined(__CPU_IMXRT106X)

#define FLEXSPI_COUNT     2
#define FLEXSPI1_BASE     ((addr_t)0x402a8000)
#define FLEXSPI2_BASE     ((addr_t)0x402a4000)
#define FLEXSPI1_AHB_ADDR ((addr_t)0x60000000)
#define FLEXSPI2_AHB_ADDR ((addr_t)0x70000000)

#elif defined(__CPU_IMXRT117X)

#define FLEXSPI_COUNT     1
#define FLEXSPI1_BASE     ((addr_t)0x400cc000)
#define FLEXSPI2_BASE     ((addr_t)-1)
#define FLEXSPI1_AHB_ADDR ((addr_t)0x30000000)
#define FLEXSPI2_AHB_ADDR ((addr_t)-1)

#else
#error "FlexSPI is not supported on this target"
#endif


typedef struct _flexspi_t {
	volatile uint32_t *base;
	uint32_t *ahbAddr;
	uint8_t instance;
	uint8_t slPortMask;
	size_t slFlashSz[4];
} flexspi_t;


struct xferOp {
	enum { xfer_opCommand = 0,
		xfer_opRead,
		xfer_opWrite } op;
	time_t timeout;
	uint32_t addr;
	uint8_t port;
	uint8_t seqIdx;
	uint8_t seqNum;
	union {
		struct {
			void *ptr;
			size_t sz;
		} read;
		struct {
			const void *ptr;
			size_t sz;
		} write;
		struct {
			/* empty */
		} command;
	} data;
};


/* Software reset (without configuration registers) */
extern void flexspi_swreset(flexspi_t *fspi);


/* Execute a transfer using lookup table of FlexSPI sequences */
extern ssize_t flexspi_xferExec(flexspi_t *fspi, struct xferOp *xfer);


#endif /* _FLEXSPI_H_ */
