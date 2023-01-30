/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq 7000 PWM driver message API
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef LIBPWMMSG_H
#define LIBPWMMSG_H


/* pwmmsg API context. Stores communication information between client and pwm driver */
typedef struct {
	oid_t oid;
} pwm_ctx_t;

typedef struct {
	uint32_t id;
	uint32_t val;
} pwm_data_t;


extern int pwmmsg_batchWrite(const pwm_ctx_t *ctx, pwm_data_t *vals, int n);

/* Reads value from pwm channel associated with opened `ctx` into `val` */
extern int pwmmsg_read(const pwm_ctx_t *ctx, uint32_t *val);


/* Writes `val` to pwm channel associated with opened `ctx` */
extern int pwmmsg_write(const pwm_ctx_t *ctx, const uint32_t val);


/* Closes pwmmsg API context specified by `ctx` */
extern int pwmmsg_close(pwm_ctx_t *ctx);


/* Opens pwmmsg API communication context. Returns 0 on success */
extern int pwmmsg_open(pwm_ctx_t *ctx, const char *path);

#endif
