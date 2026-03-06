/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver
 *
 * ONFI 4.0 definitions (userspace adaptation)
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _NANDFCTRL2_ONFI_H_
#define _NANDFCTRL2_ONFI_H_

#include <stdint.h>


#define ONFI_TCCS_BASE 500 /* ns, conservative tCCS before parameter page read */
#define ONFI_TVDLY     50  /* ns */


typedef struct {
	uint32_t tADL;
	uint32_t tALH;
	uint32_t tALS;
	uint32_t tAR;
	uint32_t tCEA;
	uint32_t tCEH;
	uint32_t tCH;
	uint32_t tCHZ;
	uint32_t tCLH;
	uint32_t tCLR;
	uint32_t tCLS;
	uint32_t tCOH;
	uint32_t tCR;
	uint32_t tCR2;
	uint32_t tCS;
	uint32_t tCS3;
	uint32_t tDH;
	uint32_t tDS;
	uint32_t tFEAT;
	uint32_t tIR;
	uint32_t tITC;
	uint32_t tRC;
	uint32_t tREA;
	uint32_t tREH;
	uint32_t tRHOH;
	uint32_t tRHW;
	uint32_t tRHZ;
	uint32_t tRLOH;
	uint32_t tRP;
	uint32_t tRR;
	uint32_t tRST;
	uint32_t tRST_EZNAND;
	uint32_t tRSTLUN_EZNAND;
	uint32_t tWB;
	uint32_t tWC;
	uint32_t tWH;
	uint32_t tWHR;
	uint32_t tWP;
	uint32_t tWW;
} onfi_timingMode_t;


typedef struct {
	/* Revision information and features block */
	uint8_t signature[4];
	uint16_t revision;
	uint16_t features;
	uint16_t optionalCmds;
	uint8_t primAdvCmdSupport;
	uint8_t res0;
	uint16_t extParamPageLen;
	uint8_t numOfParamPages;
	uint8_t res1[17];

	/* Manufacturer information block */
	uint8_t devManufacturer[12];
	uint8_t devModel[20];
	uint8_t jedecId;
	uint8_t dateCode[2];
	uint8_t res2[13];

	/* Memory organization block */
	uint32_t bytesPerPage;
	uint16_t spareBytesPerPage;
	uint32_t bytesPerPartialPage;
	uint16_t spareBytesPerPartialPage;
	uint32_t pagesPerBlock;
	uint32_t blocksPerLun;
	uint8_t numLuns;
	uint8_t addrCycles;
	uint8_t bitsPerCell;
	uint16_t maxBadBlocksPerLun;
	uint16_t blockEndurance;
	uint8_t guaranteedValidBlock;
	uint16_t blockEnduranceGvb;
	uint8_t programsPerPage;
	uint8_t partialProgAttr;
	uint8_t eccBits;
	uint8_t planeAddrBits;
	uint8_t multiplaneOpAttr;
	uint8_t ezNandSupport;
	uint8_t res3[12];

	/* Electrical parameters block */
	uint8_t ioPinCapacitance;
	uint16_t timingMode;
	uint16_t progCacheTimingMode;
	uint16_t tPageProg;
	uint16_t tBlkErase;
	uint16_t tPageRead;
	uint16_t tCCS;
	uint8_t nvddrTimingMode;
	uint8_t nvddr2TimingMode;
	uint8_t nvddrFeat;
	uint16_t clkInputPinCap;
	uint16_t ioPinCap;
	uint16_t inputPinCap;
	uint8_t inputPinCapMax;
	uint8_t drvStrength;
	uint16_t tMR;
	uint16_t tADL;
	uint16_t tPageReadEZ;
	uint8_t nvddr23Feat;
	uint8_t nvddr23WarmupCyc;
	uint16_t nvddr3TimingMode;
	uint8_t nvddr2TimingMode2;
	uint8_t res4;

	/* Vendor block */
	uint16_t vendorRevision;
	uint8_t vendorSpecific[88];
	uint16_t crc;
} __attribute__((packed)) onfi_paramPage_t;


/* Return SDR timing parameters for the given mode (0..5), or NULL if invalid. */
const onfi_timingMode_t *onfi_getTimingModeSDR(unsigned int mode);


#endif /* _NANDFCTRL2_ONFI_H_ */
