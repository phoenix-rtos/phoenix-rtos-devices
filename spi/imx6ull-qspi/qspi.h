/*
 * Phoenix-RTOS
 *
 * imx6ull QSPI Controller driver
 *
 * Copyright 2021-2023 Phoenix Systems
 * Author: Gerard Swiderski, Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _QSPI_H_
#define _QSPI_H_

#include <time.h>

#define QSPI_PORTS 4

#define QSPI_RXBUFSIZE 128u


/* Use to enable particular slave buses of QSPI controller (slPortMask) */
enum { qspi_slBusA1 = 0x1,
	qspi_slBusA2 = 0x2,
	qspi_slBusB1 = 0x4,
	qspi_slBusB2 = 0x8 };


typedef struct {
	volatile uint32_t *base;
	uint8_t *ahbBase;
	addr_t ahbAddr;
	uint8_t slPortMask;
	size_t slFlashSz[4];
} qspi_t;


struct xferOp {
	enum { xfer_opCommand = 0,
		xfer_opRead,
		xfer_opWrite } op;
	time_t timeout;
	uint32_t addr;
	uint8_t port;
	uint8_t seqIdx;
	union {
		struct {
			void *ptr;
			size_t sz;
		} read;
		struct {
			const void *ptr;
			size_t sz;
		} write;
		uint8_t command;
	} data;
};


/* Execute a transfer using lookup table of QSPI sequences */
extern ssize_t qspi_xferExec(qspi_t *qspi, struct xferOp *xfer);


extern int qspi_init(qspi_t *qspi_t);


#endif /* _QSPI_H_ */
