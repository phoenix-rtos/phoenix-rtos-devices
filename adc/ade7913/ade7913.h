/*
 * Phoenix-RTOS
 *
 * i.MX RT1176 ADE7913 API
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Marcin Baran, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef ADE7913_H
#define ADE7913_H

#include <stdint.h>
#include <sys/types.h>


typedef struct {
	uint32_t iwv : 24;
	uint32_t v1wv : 24;
	uint32_t v2wv : 24;
	uint16_t adc_crc;
	uint8_t status0;
	uint16_t cnt_snapshot;
} __attribute__((packed)) ade7913_burst_reg_t;


int ade7913_init(oid_t *device, int cs, int clkout);
int ade7913_sync(oid_t *device, const char *cs, int devcnt, int snap);

int ade7913_get_sampling_rate(oid_t *device, int cs, int *freq_hz);
int ade7913_set_sampling_rate(oid_t *device, int cs, int freq_hz);

int ade7913_version(oid_t *device, int cs);
int ade7913_ready(oid_t *device, int cs);

int ade7913_sample_lost(oid_t *device, int cs);
int ade7913_sample_regs_read(oid_t *device, int cs, ade7913_burst_reg_t *reg);

int ade7913_lock(oid_t *device, int cs);
int ade7913_unlock(oid_t *device, int cs);

int ade7913_emi(oid_t *device, int cs, uint8_t val);

int ade7913_enable(oid_t *device, int cs);
int ade7913_disable(oid_t *device, int cs);

int ade7913_temperature(oid_t *device, int cs);
int ade7913_voltage(oid_t *device, int cs);

int ade7913_reset_soft(oid_t *device, int cs);
int ade7913_reset_hard(oid_t *device, int cs);

#endif /* ADE7913_H */
