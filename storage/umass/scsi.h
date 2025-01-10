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

#define CBW_SIG 0x43425355
#define CSW_SIG 0x53425355


/* Opcodes */
#define SCSI_REQUEST_SENSE 0x03
#define SCSI_INQUIRY       0x12


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dlen;
	uint8_t flags;
	uint8_t lun;
	uint8_t clen;
	uint8_t cmd[16];
} __attribute__((packed)) umass_cbw_t;


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dr;
	uint8_t status;
} __attribute__((packed)) umass_csw_t;


typedef struct {
	uint8_t opcode;
	uint8_t action : 5;
	uint8_t misc0 : 3;
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
	uint8_t misc0 : 4;
	uint8_t sensekey : 4;
	uint8_t misc[17];
} __attribute__((packed)) scsi_sense_t;


typedef struct {
	uint8_t qualifier : 3;
	uint8_t devicetype : 5;
	uint8_t misc0;
	uint8_t version;
	uint8_t misc1[5];
	char vendorid[8];
	char productid[16];
	uint8_t misc[4];
} __attribute__((packed)) scsi_inquiry_t;


#endif
