/*
 * Phoenix-RTOS
 *
 * USB Mass Storage class driver
 *
 * SCSI transparent command set definitions header
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * %LICENSE%
 */


#ifndef _SCSI_H_
#define _SCSI_H_

#include <unistd.h>


/* Opcodes */
#define SCSI_REQUEST_SENSE 0x03
#define SCSI_INQUIRY       0x12


typedef struct {
	uint8_t opcode;
	uint8_t action_misc0; /* [8:3] action, [3:0] misc0 */
	uint32_t lba;
	uint8_t misc1;
	uint16_t length;
	uint8_t control;
} __attribute__((packed)) scsi_cdb10_t;


typedef struct {
	uint8_t opcode;
	uint8_t misc0[3];
	uint8_t length;
	uint8_t control;
} __attribute__((packed)) scsi_cdb6_t;


typedef struct {
	uint8_t errorcode;
	uint8_t segnum;
	uint8_t misc0_sensekey; /* [8:4] misc0, [4:0] sensekey */
	uint8_t misc[17];
} __attribute__((packed)) scsi_sense_t;


typedef struct {
	uint8_t qualifier_devicetype; /* [8:5] qualifier, [5:0] devicetype */
	uint8_t misc0;
	uint8_t version;
	uint8_t misc1[5];
	char vendorid[8];
	char productid[16];
	uint8_t misc[4];
} __attribute__((packed)) scsi_inquiry_t;


#endif
