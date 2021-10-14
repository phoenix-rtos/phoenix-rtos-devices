/*
 * Phoenix-RTOS
 *
 * AD7779 components API
 *
 * Copyright 2021 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef AD7779_LOW_H
#define AD7779_LOW_H

/* For now most of the drivers do not have common api.
   This set of functions represents abstraction layer for targets
   and must be implemented by the user */

/* DMA api */
int dma_init(size_t size, size_t count, addr_t *phys_addr);


void dma_free(void);


void dma_enable(void);


void dma_disable(void);


/* dma_read in most cases is used to read an interrupt counter (and to wait for the interrupt) */
int dma_read(void *data, size_t len);


/* SAI api */
int sai_init(void);


void sai_free(void);


void sai_rx_enable(void);


void sai_rx_disable(void);


addr_t sai_fifo_rx_ptr(void);


uint32_t sai_fifo_watermark(void);


/* SPI api */
int spi_exchange(uint8_t *buff, uint8_t len);


int spi_init(void);


/* GPIO api */
typedef enum {
	start,
	reset,
	hardreset
} ad7779_gpio_t;


int ad7779_gpio(ad7779_gpio_t gpio, int state);


int ad7779_gpio_init(void);

#endif
