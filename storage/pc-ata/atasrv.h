/*
 * Phoenix-RTOS
 *
 * PC ATA server.
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PC_ATASRV_H_
#define _PC_ATASRV_H_

#include "atadrv.h"

typedef struct _ata_msg_t {
	uint16_t bus;
	uint16_t channel;
	uint16_t device;
	offs_t offset;
	uint16_t len;
	char data[];
} __attribute__((packed)) ata_msg_t;


int atasrv_registerDevice(ata_dev_t *ataDev);

#endif