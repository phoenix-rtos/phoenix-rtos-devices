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


/* Initialize rtt descriptor and rtt internal structures */
int rtt_init(void *addr, void *bufptr, size_t bufsz);


/* Release resources, cleanup */
void rtt_done(void);


/* Check if channel is present */
int rtt_check(int chan);


/* Non-blocking read from channel */
ssize_t rtt_read(int chan, void *buf, size_t count);


/* Non-blocking write to channel */
ssize_t rtt_write(int chan, const void *buf, size_t count);


/* Check for available space in tx */
ssize_t rtt_txAvail(int chan);


/* Reset rx fifo pointers */
void rtt_rxReset(int chan);


/* Reset tx fifo pointers */
void rtt_txReset(int chan);


#endif /* end of LIBRTT_H */
