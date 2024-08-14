/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * CFI
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _CFI_H_
#define _CFI_H_


#include <sys/types.h>

/* Timeouts in us */
#define CFI_TIMEOUT_MAX_PROGRAM(typical, maximum) ((1u << typical) * (1u << maximum))
#define CFI_TIMEOUT_MAX_ERASE(typical, maximum)   ((1u << typical) * (1u << maximum) * 1024u)

#define CFI_SIZE(size) (1u << ((uint32_t)size))

/* Common flash commands */
#define CMD_RD_QUERY  0x98u /* Read/Enter Query */
#define CMD_RD_STATUS 0x70u /* Read Status Register */

/* Intel command set */
#define INTEL_CMD_RESET      0xffu /* Reset/Read Array */
#define INTEL_CMD_WR_BUF     0xe8u /* Write to Buffer */
#define INTEL_CMD_WR_CONFIRM 0xd0u /* Write Confirm */
#define INTEL_CMD_CLR_STATUS 0x50u /* Clear Status Register */
#define INTEL_CMD_BE_CYC1    0x20u /* Block Erase (1st bus cycle) */

/* AMD command set */
#define AMD_CMD_RESET      0xf0u /* Reset/ASO Exit */
#define AMD_CMD_WR_BUF     0x25u /* Write to Buffer */
#define AMD_CMD_WR_CONFIRM 0x29u /* Write Confirm */
#define AMD_CMD_CLR_STATUS 0x71u /* Clear Status Register */
#define AMD_CMD_CE_CYC1    0x80u /* Chip Erase (1st bus cycle) */
#define AMD_CMD_CE_CYC2    0x10u /* Chip Erase (2nd bus cycle) */
#define AMD_CMD_BE_CYC1    0x80u /* Block Erase (1st bus cycle) */
#define AMD_CMD_BE_CYC2    0x30u /* Block Erase (2nd bus cycle) */
#define AMD_CMD_EXIT_QUERY 0xf0u /* Exit Query */


typedef struct {
	uint8_t (*statusRead)(volatile uint8_t *base);
	int (*statusCheck)(volatile uint8_t *base, const char *msg);
	void (*statusClear)(volatile uint8_t *base);
	void (*issueReset)(volatile uint8_t *base);
	void (*issueWriteBuffer)(volatile uint8_t *base, off_t sectorOffs, off_t programOffs, size_t len);
	void (*issueWriteConfirm)(volatile uint8_t *base, off_t sectorOffs);
	void (*issueSectorErase)(volatile uint8_t *base, off_t sectorOffs);
	void (*issueChipErase)(volatile uint8_t *base);
	void (*enterQuery)(volatile uint8_t *base, off_t sectorOffs);
	void (*exitQuery)(volatile uint8_t *base);
} cfi_ops_t;


typedef struct {
	const uint8_t statusRdyMask;
	const uint8_t usePolling;
	const uint8_t chipWidth;
	const uint8_t vendor;
	const uint16_t device;
	const char *name;
	const cfi_ops_t *ops;
} cfi_dev_t;


typedef struct {
	uint8_t wordProgram;
	uint8_t bufWrite;
	uint8_t blkErase;
	uint8_t chipErase;
} __attribute__((packed)) cfi_timeout_t;


typedef struct {
	uint8_t vendorData[0x10];
	uint8_t qry[3];
	uint16_t cmdSet1;
	uint16_t addrExt1;
	uint16_t cmdSet2;
	uint16_t addrExt2;
	struct {
		uint8_t vccMin;
		uint8_t vccMax;
		uint8_t vppMin;
		uint8_t vppMax;
	} __attribute__((packed)) voltages;
	cfi_timeout_t toutTypical;
	cfi_timeout_t toutMax;
	uint8_t chipSz;
	uint16_t fdiDesc;
	uint16_t bufSz;
	uint8_t regionCnt;
	struct {
		uint16_t count;
		uint16_t size;
	} __attribute__((packed)) regions[4];
} __attribute__((packed)) cfi_info_t;


#endif
