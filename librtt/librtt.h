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

/* Initialize rtt descriptor and rtt internal structures.
   TODO: on armv7m targets MAP_UNCACHED flag seems to not be implemented.
   Caller must supply `bufptr` in non-cached memory.
*/
int librtt_init(void *addr, void *bufptr, size_t bufsz, librtt_cacheOp_t invalFn, librtt_cacheOp_t cleanFn);


/* Release resources, cleanup */
void librtt_done(void);


/* Check if channel is present */
int librtt_check(int chan);


/* Non-blocking read from channel */
ssize_t librtt_read(int chan, void *buf, size_t count);


/* Non-blocking write to channel */
ssize_t librtt_write(int chan, const void *buf, size_t count);


/* Check for available space in tx */
ssize_t librtt_txAvail(int chan);


/* Reset rx fifo pointers */
void librtt_rxReset(int chan);


/* Reset tx fifo pointers */
void librtt_txReset(int chan);


#endif /* end of LIBRTT_H */
