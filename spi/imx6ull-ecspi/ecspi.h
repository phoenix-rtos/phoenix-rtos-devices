/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI lib API
 *
 * Copyright 2018 Phoenix Systems
 * Author: Daniel Sawka, Krystian Wasik
 *
 * %LICENSE%
 */

#ifndef IMX6ULL_ECSPI_API_H
#define IMX6ULL_ECSPI_API_H

#include <stddef.h>
#include <stdint.h>

#include <sys/threads.h>
#include <phoenix/arch/imx6ull.h>

enum { ecspi1 = 1, ecspi2, ecspi3, ecspi4 };


typedef int ecspi_writerProc_t(const uint8_t *rx, size_t len, uint8_t *out);


typedef struct {
	int dev_no;
	handle_t inth;

	volatile size_t rx_count;
	uint8_t *out_periodical;
	uint8_t *in_periodical;
	unsigned int prev_wait_states;

	ecspi_writerProc_t *writer_proc;
} ecspi_ctx_t;


int ecspi_init(int dev_no, uint8_t chan_msk);
int ecspi_registerContext(int dev_no, ecspi_ctx_t *ctx, handle_t cond);

int ecspi_exchange(int dev_no, const uint8_t *out, uint8_t *in, size_t len);
int ecspi_exchangeBusy(int dev_no, const uint8_t *out, uint8_t *in, size_t len);

int ecspi_setChannel(int dev_no, uint8_t chan);
int ecspi_setMode(int dev_no, uint8_t chan, uint8_t mode);
int ecspi_setClockDiv(int dev_no, uint8_t pre, uint8_t post);
int ecspi_setCSDelay(int dev_no, uint8_t delay);
int ecspi_setSSDelay(int dev_no, uint16_t delay);

int ecspi_writeAsync(ecspi_ctx_t *ctx, const uint8_t *out, size_t len);
int ecspi_exchangeAsync(ecspi_ctx_t *ctx, const uint8_t *out, size_t len);
int ecspi_exchangePeriodically(ecspi_ctx_t *ctx, uint8_t *out, uint8_t *in, size_t len, unsigned int wait_states, ecspi_writerProc_t writer_proc);
int ecspi_readFifo(ecspi_ctx_t *ctx, uint8_t *buf, size_t len);

addr_t ecspi_getTxFifoPAddr(int dev_no);
addr_t ecspi_getRxFifoPAddr(int dev_no);

#endif /* IMX6ULL_ECSPI_API_H */
