/*
 * Phoenix-RTOS
 *
 * RTT pipes: communication through debug probe
 *
 * Copyright 2023-2024 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef LIBRTT_H
#define LIBRTT_H

#include <sys/types.h>

/* Size of the common descriptor for RTT channels */
#define LIBRTT_DESC_SIZE 256

typedef int (*librtt_cacheOp_t)(void *addr, unsigned int sz);


/* Initialize rtt descriptor if it is not initialized already */
int librtt_init(void *addr, librtt_cacheOp_t invalFn, librtt_cacheOp_t cleanFn);


/* Initialize a single RTT channel */
int librtt_initChannel(int isTx, unsigned int ch, unsigned char *buf, size_t sz);


/* Release resources, cleanup */
void librtt_done(void);


/* Check if TX channel is present */
int librtt_checkTx(unsigned int ch);


/* Check if RX channel is present */
int librtt_checkRx(unsigned int ch);


/* Non-blocking read from channel */
ssize_t librtt_read(unsigned int ch, void *buf, size_t count);


/* Non-blocking write to channel */
ssize_t librtt_write(unsigned int ch, const void *buf, size_t count, int allowOverwrite);


/* Check for available data in rx */
ssize_t librtt_rxAvail(unsigned int ch);


/* Check for available space in tx */
ssize_t librtt_txAvail(unsigned int ch);


/* Check if a reader has read something from chosen TX channel since the last check */
int librtt_txCheckReaderAttached(unsigned int ch);


/* Reset rx fifo pointers */
void librtt_rxReset(unsigned int ch);


/* Reset tx fifo pointers */
void librtt_txReset(unsigned int ch);


#endif /* end of LIBRTT_H */
