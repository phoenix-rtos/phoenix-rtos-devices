/*
 * Phoenix-RTOS
 *
 * libphoenix
 *
 * stdio.h
 *
 * Copyright 2017 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _MBR_H_
#define _MBR_H_ /* mbr.h */

#include <stdint.h>

#include "atadrv.h"

#define PENTRY_LINUX 0x83       /* any native linux partition */
#define PENTRY_PROTECTIVE 0xEE  /* protective mbr mode for gpt partition table */

/* partition entry structure */
struct pentry_t {
	uint8_t      status;
	uint8_t      first_sect[3]; /* chs */
	uint8_t      type;
	uint8_t      last_sect[3];  /* chs */
	uint32_t     first_sect_lba;
	uint32_t     sector_count;
} __attribute__((packed));

#define MBR_SIGNATURE 0xAA55

/* master boot record structure */
typedef struct {
	char            	bca[446];   /* bootstrap code area */
	struct pentry_t 	pent[4];    /* partition entries */
	uint16_t			boot_sign;  /* mbr signature */
} mbr_t;

mbr_t *alloc_mbr(ata_dev_t *dev);


#endif /* mbr.h */
