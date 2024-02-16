/*
 * Phoenix-RTOS
 *
 * GRLIB DMA Controller driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GRDMAC2_H_
#define _GRDMAC2_H_


#include <stdint.h>
#include <stdbool.h>


/* Descriptor type: 0 - data, 1 - polling, 2 - triggering, 3 - poll on trigger */
#define GRDMA_DESC_TYPE(x) (((x) & 0xf) << 1)

/* Data descriptor ctrl */
#define GRDMA_DATA_EN    (1 << 0)                 /* Enable */
#define GRDMA_DATA_WB    (1 << 5)                 /* Write back status */
#define GRDMA_DATA_IE    (1 << 8)                 /* Interrupt enable on descr. complete */
#define GRDMA_DATA_SF    (1 << 9)                 /* Source address fixed */
#define GRDMA_DATA_DF    (1 << 10)                /* Destination address fixed */
#define GRDMA_DATA_SZ(x) (((x) & 0x1fffff) << 11) /* Transfer size (in bytes) */


typedef struct {
	uint32_t ctrl;         /* Control word */
	uint32_t next;         /* Next descriptor address */
	uint32_t dest;         /* Destination base address */
	uint32_t src;          /* Source base address */
	volatile uint32_t sts; /* Status word */
} __attribute__((packed, aligned(4))) grdma_dataDescr_t;


/* Conditional descriptor ctrl */
#define GRDMA_COND_EN       (1 << 0)             /* Enable */
#define GRDMA_COND_WB       (1 << 5)             /* Write back status */
#define GRDMA_COND_IRQN(x)  (((x) & 0x3f) << 7)  /* Interrupt line number to be monitored */
#define GRDMA_COND_ERRTO    (1 << 13)            /* Treat timeout as error */
#define GRDMA_COND_IE       (1 << 14)            /* Interrupt enable on descr. complete */
#define GRDMA_COND_TRIG(x)  (((x) & 0x1) << 15)  /* Expected trigger: 0 - edge, 1 - level */
#define GRDMA_COND_INTRV(x) (((x) & 0xff) << 16) /* Clock cycles between polls */
#define GRDMA_COND_CNT(x)   (((x) & 0xff) << 24) /* Number of polls */


typedef struct {
	uint32_t ctrl;         /* Control word */
	uint32_t next;         /* Next descriptor address */
	uint32_t nextFail;     /* Next descriptor address on fail */
	uint32_t poll;         /* Poll address */
	volatile uint32_t sts; /* Status word */
	uint32_t expData;      /* Expected data */
	uint32_t mask;         /* Conditional mask */
} __attribute__((packed, aligned(4))) grdma_condDescr_t;


typedef struct {
	volatile uint32_t *base;
	uint8_t irq;
	void *descr;
	size_t descrSize;
} grdma_ctx_t;


/* Setup GRDMA descriptors
 * Writes the `first` descriptor address to GRDMA register.
 * Following descriptors are read based on the `next` field of the descriptor.
 */
void grdma_setup(grdma_ctx_t *ctx, void *first);


/* Map memory for descriptors */
int grdma_descrAlloc(grdma_ctx_t *ctx, size_t size);


/* Start operation */
void grdma_start(grdma_ctx_t *ctx);


/* Restart operation */
void grdma_restart(grdma_ctx_t *ctx);


/* Check if queue is finished */
bool grdma_finished(grdma_ctx_t *ctx);


/* Initialize GRDMA context */
grdma_ctx_t *grdma_init(unsigned int instance);


/* Destroy GRDMA context */
void grdma_destroy(grdma_ctx_t *ctx);


#endif
