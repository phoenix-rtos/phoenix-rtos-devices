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

#include <stdint.h>

#include <phoenix/arch/imx6ull.h>

enum { ecspi1 = 1, ecspi2, ecspi3, ecspi4 };


int ecspi_init(int dev_no, uint8_t chan_msk);

int ecspi_exchange(int dev_no, const uint8_t *out, uint8_t *in, uint16_t len);

int ecspi_setChannel(int dev_no, uint8_t chan);
int ecspi_setMode(int dev_no, uint8_t chan, uint8_t mode);
int ecspi_setClockDiv(int dev_no, uint8_t pre, uint8_t post);
int ecspi_setCSDelay(int dev_no, uint8_t delay);

addr_t ecspi_getTxFifoPAddr(int dev_no);
addr_t ecspi_getRxFifoPAddr(int dev_no);

#endif /* IMX6ULL_ECSPI_API_H */
