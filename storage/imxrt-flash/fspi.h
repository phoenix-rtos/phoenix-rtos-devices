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

#include <unistd.h>
#include <board_config.h>

#ifndef _FLEXSPI_H_
#define _FLEXSPI_H_

#if defined(__CPU_IMXRT106X)

#ifndef FLEXSPI_COUNT
#define FLEXSPI_COUNT 2
#endif

#define FLEXSPI1_BASE     ((addr_t)0x402a8000)
#define FLEXSPI2_BASE     ((addr_t)0x402a4000)
#define FLEXSPI1_AHB_ADDR ((addr_t)0x60000000)
#define FLEXSPI2_AHB_ADDR ((addr_t)0x70000000)

#ifndef FLEXSPI1_XIP
#define FLEXSPI1_XIP 1
#endif
#ifndef FLEXSPI1_PORT
#define FLEXSPI1_PORT 0
#endif
#ifndef FLEXSPI1_PORT_MASK
#define FLEXSPI1_PORT_MASK 1
#endif
#ifndef FLEXSPI2_XIP
#define FLEXSPI2_XIP 0
#endif
#ifndef FLEXSPI2_PORT
#define FLEXSPI2_PORT 0
#endif
#ifndef FLEXSPI2_PORT_MASK
#define FLEXSPI2_PORT_MASK 1
#endif

#elif defined(__CPU_IMXRT117X)

#ifndef FLEXSPI_COUNT
#define FLEXSPI_COUNT 1
#endif

#define FLEXSPI1_BASE     ((addr_t)0x400cc000)
#define FLEXSPI2_BASE     ((addr_t)0x400d0000)
#define FLEXSPI1_AHB_ADDR ((addr_t)0x30000000)
#define FLEXSPI2_AHB_ADDR ((addr_t)0x60000000)

#ifndef FLEXSPI1_XIP
#define FLEXSPI1_XIP 1
#endif
#ifndef FLEXSPI1_PORT
#define FLEXSPI1_PORT 0
#endif
#ifndef FLEXSPI1_PORT_MASK
#define FLEXSPI1_PORT_MASK 1
#endif
#ifndef FLEXSPI2_XIP
#define FLEXSPI2_XIP 0
#endif
#ifndef FLEXSPI2_PORT
#define FLEXSPI2_PORT 0
#endif
#ifndef FLEXSPI2_PORT_MASK
#define FLEXSPI2_PORT_MASK 1
#endif

#else
#error "FlexSPI is not supported on this target"
#endif

#ifndef FLEXSPI_QUICK_POLL_MAX_RETRIES
#define FLEXSPI_QUICK_POLL_MAX_RETRIES 5
#endif


typedef struct _flexspi_t {
	volatile uint32_t *base;
	uint8_t *ahbAddr;
	uint8_t instance;
	uint8_t xip;
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


static inline void flexspi_schedYield(flexspi_t *fspi)
{
	if (fspi->xip == 1) {
		/*
		 * NOTICE: this schedYield is only valid when used with a scheduler that is able to block FLASH (xip)
		 * threads from execution (threads with a lower priority than the higher priority RAM code).
		 * Otherwise, adjust the implementation of this function as needed, following the same principles
		 * as for the implementation of enter/exit critical sections.
		 */
		(void)usleep(0);
	}
	else {
		(void)usleep(100);
	}
}


/* Execute a transfer using lookup table of FlexSPI sequences */
extern ssize_t flexspi_xferExec(flexspi_t *fspi, struct xferOp *xfer);


#endif /* _FLEXSPI_H_ */
