/*
 * Phoenix-RTOS
 *
 * Master Boot Record
 *
 * Copyright 2017, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MBR_H_
#define _MBR_H_

#include <stdint.h>

#include "ata.h"


/* Misc definitions */
#define MBR_MAGIC 0xaa55


/* Partition types */
enum {
	PENTRY_LINUX      = 0x83, /* Any native Linux partition */
	PENTRY_PROTECTIVE = 0xee  /* Protective MBR mode for GPT partition table */
};


typedef struct {
	uint8_t status;   /* Partition status */
	uint8_t first[3]; /* First sector (CHS) */
	uint8_t type;     /* Partition type */
	uint8_t last[3];  /* Last sector (CHS) */
	uint32_t start;   /* Partition start (LBA) */
	uint32_t sectors; /* Number of sectors */
} __attribute__((packed)) pentry_t;


typedef struct {
	char bca[446];    /* Bootstrap Code Area */
	pentry_t pent[4]; /* Partition entries */
	uint16_t magic;   /* MBR magic */
} __attribute__((packed)) mbr_t;


/* Reads MBR */
int mbr_read(ata_dev_t *dev, mbr_t *mbr);


#endif
