/*
 * Phoenix-RTOS
 *
 * SPI message interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PHOENIX_SPI_MSG_H_
#define _PHOENIX_SPI_MSG_H_

#include <stddef.h>

#include <sys/msg.h>
#include <sys/types.h>


enum {
	spi_devctl_xfer = 0, /* input: *ctx, *out, olen, *in, ilen, iskip */
};


typedef struct {
	oid_t oid;          /* SPI slave oid, initialized by spimsg_open() */
	unsigned char mode; /* SPI clock mode (phase and polarity), should be set by the user */
	unsigned int speed; /* SPI clock speed, should be set by the user */
} spimsg_ctx_t;


typedef struct {
	union {
		struct {
			unsigned int type; /* Devctl type */
			spimsg_ctx_t ctx;  /* SPI context */
			struct {
				size_t isize; /* Size of input data */
				size_t osize; /* Size of output data */
				size_t iskip; /* Number of bytes to skip from MISO */
			} xfer;
		} i;

		struct {
			int err;
		} o;
	} u;

	unsigned char payload[0];
} spi_devctl_t;


/* Performs SPI transaction */
extern int spimsg_xfer(const spimsg_ctx_t *ctx, const void *out, size_t olen, void *in, size_t ilen, size_t iskip);


/* Closes SPI message context */
extern int spimsg_close(const spimsg_ctx_t *ctx);


/* Opens SPI message context and initializes its oid */
extern int spimsg_open(unsigned int dev, unsigned int ss, spimsg_ctx_t *ctx);


#endif
