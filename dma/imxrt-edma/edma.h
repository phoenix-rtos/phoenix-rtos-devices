/*
 * Phoenix-RTOS
 *
 * i.MX RT eDMA lib
 *
 * Copyright 2019 Phoenix Systems
 * Author: Krystian Wasik
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef IMXRT_EDMA_H
#define IMXRT_EDMA_H

#include <stdint.h>

#define EDMA_NUM_OF_CHANNELS        (32)
#define EDMA_ERROR_IRQ              (16 + 16)
#define EDMA_CHANNEL_IRQ(channel)   (16 + channel%16)

/* eDMA Transfer Control Descriptor */
struct edma_tcd_s {
	uint32_t saddr;
	uint16_t soff;
	uint16_t attr;
	union {
		uint32_t nbytes_mlno;
		uint32_t nbytes_mlnoffno;
		uint32_t nbytes_mlnoffyes;
	};
	uint32_t slast;
	uint32_t daddr;
	uint16_t doff;
	union {
		uint16_t citer_elinkno;
		uint16_t citer_elinkyes;
	};
	uint32_t dlast_sga;
	uint16_t csr;
	union {
		uint16_t biter_elinkno;
		uint16_t biter_elinkyes;
	};
} __attribute__ ((aligned(32)));

void dmamux_set_source(uint8_t channel, uint8_t source);
void dmamux_channel_enable(uint8_t channel);
void dmamux_channel_disable(uint8_t channel);
int dmamux_channel_is_enabled(uint8_t channel);

int edma_init(int (*error_isr)(unsigned int n, void *arg));

void edma_copy_tcd(volatile struct edma_tcd_s *dst,
	const volatile struct edma_tcd_s *src);

int edma_install_tcd(const volatile struct edma_tcd_s* tcd, uint8_t channel);

int edma_read_tcd(volatile struct edma_tcd_s *tcd, uint8_t channel);

int edma_initialize_tcd_ring(const struct edma_tcd_s* prototype,
	volatile struct edma_tcd_s** tcds, unsigned cnt,
	unsigned src_offset, unsigned dst_offset);

/* Returns value of SSIZE/DSIZE bits for transfer size passed as a param */
uint8_t edma_get_tcd_attr_xsize(uint8_t num_of_bytes);

/* Checks if there is a pending hardware request for given channel */
int edma_is_hw_req_pending(unsigned channel);
uint32_t edma_error_status(unsigned channel);
uint32_t edma_error_channel(void);

void edma_channel_enable(unsigned channel);
void edma_channel_disable(unsigned channel);

void edma_clear_interrupt(unsigned channel);
void edma_clear_error(unsigned channel);

void edma_software_request_start(int channel);

#endif /* IMXRT_EDMA_H */
