/*
 * Phoenix-RTOS
 *
 * Zynq-7000 Quad-SPI Controller driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _QUAD_SPI_H_
#define _QUAD_SPI_H_

#include <stdint.h>
#include <time.h>


/* NOTE: All synchronization and exclusion must be done externally.  */


/* Transmit and receive number of data defines in 'size'. If TX Buff is empty, the dummy words are sent,
 * otherwise txBuff data is sent, returns EOK on success or <0 on error */
extern ssize_t qspi_transfer(const uint8_t *txBuff, uint8_t *rxBuff, size_t size, time_t timeout);


/* Raise CS down, enable controller */
extern void qspi_start(void);


/* Clean up RX fifo, raise CS high and disable controller */
extern void qspi_stop(void);


/* Switch off clocks and qspi controller */
extern int qspi_deinit(void);


/* Initialize controller:
 * - I/O mode and manual mode,
 * - little endian
 * - src clk = 200 MHz, qspi clk = 100 MHz or 50 MHz if Feedback Clock (QSPI_FCLK < 0) pin is not used.
 * returns EOK on success <0 on error */
extern int qspi_init(void);


#endif
