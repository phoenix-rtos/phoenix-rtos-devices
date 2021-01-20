/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash Configurator
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMXRT_FLASH_CONFIG_H_
#define _IMXRT_FLASH_CONFIG_H_


#include "flashdrv.h"

#define GET_MANUFACTURE_ID(flashID)      (flashID & 0xff)
#define GET_DEVICE_ID(flashID)           (((flashID >> 16) & 0xff) | (flashID & (0xff << 8)))

/* List of supported flash memories */
#define WINDBOND_W25Q32JV_IQ             0x4016
#define ISSI_DEV_IS25WP064A              0x7017
#define MICRON_MT25QL512ABB              0xba20
#define MICORN_MT25QL01GBBB              0xba21

enum { flash_windbond = 0xef, flash_issi = 0x9d, flash_micron = 0x20 };


/* List of sequences impose by ROM API */
#define QUAD_FAST_READ_SEQ_ID      0
#define READ_STATUS_REG_SEQ_ID     1
#define WRITE_ENABLE_SEQ_ID        3
#define SECTOR_ERASE_SEQ_ID        5
#define BLOCK_ERASE_SEQ_ID         8
#define PAGE_PROGRAM_SEQ_ID        9
#define CHIP_ERASE_SEQ_ID          11
#define READ_JEDEC_ID_SEQ_ID       12


int flash_getConfig(flash_context_t *ctx);


#endif
