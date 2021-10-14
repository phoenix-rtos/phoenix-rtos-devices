/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL SDMA driver API
 *
 * Copyright 2018 Phoenix Systems
 * Author: Krystian Wasik
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef IMX6ULL_SDMA_API_H
#define IMX6ULL_SDMA_API_H

#include <string.h>
#include <stdint.h>

/* Buffer Descriptor flags */
#define SDMA_BD_DONE                            (1 << 0)
#define SDMA_BD_WRAP                            (1 << 1)
#define SDMA_BD_CONT                            (1 << 2) /* Continuous */
#define SDMA_BD_INTR                            (1 << 3) /* Interrupt */
#define SDMA_BD_ERR                             (1 << 4) /* Error */
#define SDMA_BD_LAST                            (1 << 5) /* Last Buffer Descriptor */

/* Buffer Descriptor Commands for scripts supporting various data sizes */
#define SDMA_CMD_MODE_32_BIT                    (0b00)
#define SDMA_CMD_MODE_24_BIT                    (0b11)
#define SDMA_CMD_MODE_16_BIT                    (0b10)
#define SDMA_CMD_MODE_8_BIT                     (0b01)

/* Data memory address to program memory address */
#define SDMA_DA_TO_PA(addr)                     (addr*2)

/* Program memory address to data memory address */
#define SDMA_PA_TO_DA(addr)                     (addr/2)

#define SDMA_CHANNEL_PRIORITY_MAX               (7)
#define SDMA_CHANNEL_PRIORITY_MIN               (1)
#define SDMA_CHANNEL_PRIORITY_DISABLED          (0) /* Channel disabled */

/* ROM script addresses */
typedef enum {
	sdma_script__ap_2_ap         = 642,
	sdma_script__ap_2_mcu        = 683,
	sdma_script__mcu_2_ap        = 747,
	sdma_script__uart_2_mcu      = 817,
	sdma_script__shp_2_mcu       = 891,
	sdma_script__mcu_2_shp       = 960,
	sdma_script__uartsh_2_mcu    = 1032,
	sdma_script__spdif_2_mcu     = 1100,
	sdma_script__mcu_2_spdif     = 1134,
} sdma_script_t;

struct __attribute__((packed)) sdma_buffer_desc_s {
	uint32_t count:16; /* Size of the buffer */
	uint32_t flags:8;
	uint32_t command:8;
	uint32_t buffer_addr; /* Buffer address */
	uint32_t ext_buffer_addr; /* Extended buffer address */
};

typedef struct sdma_buffer_desc_s sdma_buffer_desc_t;

struct __attribute__((packed)) sdma_context_s {
	uint32_t state[2];
	uint32_t gr[8];
	uint32_t mda;
	uint32_t msa;
	uint32_t ms;
	uint32_t md;
	uint32_t pda;
	uint32_t psa;
	uint32_t ps;
	uint32_t pd;
	uint32_t ca;
	uint32_t cs;
	uint32_t dda;
	uint32_t dsa;
	uint32_t ds;
	uint32_t dd;
	uint32_t scratch[8];
};

typedef struct sdma_context_s sdma_context_t;

#define SDMA_CONTEXT_PC_MASK        (0x3fff)

static inline void sdma_context_init(sdma_context_t *ctx)
{
	memset(ctx, 0, sizeof(sdma_context_t));
}

static inline void sdma_context_set_pc(sdma_context_t *ctx, uint16_t pc)
{
	ctx->state[0] &= ~SDMA_CONTEXT_PC_MASK;
	ctx->state[0] |= pc & SDMA_CONTEXT_PC_MASK;
}

typedef enum {
	sdma_trig__event,
	sdma_trig__host,
} sdma_trig_t;

typedef struct {
	addr_t bd_paddr; /* Physical address of buffer descriptor array */
	unsigned bd_cnt;
	sdma_trig_t trig;
	unsigned event;
	unsigned priority;
} sdma_channel_config_t;

typedef enum {
	sdma_dev_ctl__channel_cfg,
	sdma_dev_ctl__data_mem_write,
	sdma_dev_ctl__data_mem_read,
	sdma_dev_ctl__context_dump,
	sdma_dev_ctl__context_set,
	sdma_dev_ctl__enable,
	sdma_dev_ctl__disable,
	sdma_dev_ctl__trigger,
	sdma_dev_ctl__ocram_alloc
} sdma_dev_ctl_type_t;

typedef struct {
	sdma_dev_ctl_type_t type;
	oid_t oid;

	union {
		/* mem read/write */
		struct {
			uint16_t addr;
			uint16_t len;
		} mem;

		sdma_channel_config_t cfg;

		struct {
			size_t size;
			addr_t paddr;
		} alloc;
	};
} sdma_dev_ctl_t;

#endif /* IMX6ULL_SDMA_API_H */
