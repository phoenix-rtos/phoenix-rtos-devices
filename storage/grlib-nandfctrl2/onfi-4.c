/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver
 *
 * ONFI 4.0 functions
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "onfi-4.h"


static uint16_t deserialize16(const uint8_t *from)
{
	return from[0] | (from[1] << 8);
}


static uint32_t deserialize32(const uint8_t *from)
{
	return from[0] | (from[1] << 8) | (from[2] << 16) | (from[3] << 24);
}


void onfi_deserializeParamPage(onfi_paramPage_t *dst, const uint8_t *src)
{
	memcpy(dst->signature, &src[0], sizeof(dst->signature));
	dst->revision = deserialize16(&src[4]);
	dst->features = deserialize16(&src[6]);
	dst->optionalCmds = deserialize16(&src[8]);
	dst->primAdvCmdSupport = src[10];
	dst->res0 = src[11];
	dst->extParamPageLen = deserialize16(&src[12]);
	dst->numOfParamPages = src[14];

	memcpy(dst->res1, &src[15], sizeof(dst->res1));

	/* Manufacturer information block (offsets 32-79) */
	memcpy(dst->devManufacturer, &src[32], sizeof(dst->devManufacturer));
	memcpy(dst->devModel, &src[44], sizeof(dst->devModel));

	dst->jedecId = src[64];
	dst->dateCode[0] = src[65];
	dst->dateCode[1] = src[66];

	memcpy(dst->res2, &src[67], sizeof(dst->res2));

	/* Memory organization block (offsets 80-127) */
	dst->bytesPerPage = deserialize32(&src[80]);
	dst->spareBytesPerPage = deserialize16(&src[84]);
	dst->bytesPerPartialPage = deserialize32(&src[86]);
	dst->spareBytesPerPartialPage = deserialize16(&src[90]);
	dst->pagesPerBlock = deserialize32(&src[92]);
	dst->blocksPerLun = deserialize32(&src[96]);
	dst->numLuns = src[100];
	dst->addrCycles = src[101];
	dst->bitsPerCell = src[102];
	dst->maxBadBlocksPerLun = deserialize16(&src[103]);
	dst->blockEndurance = deserialize16(&src[105]);
	dst->guaranteedValidBlock = src[107];
	dst->blockEnduranceGvb = deserialize16(&src[108]);
	dst->programsPerPage = src[110];
	dst->partialProgAttr = src[111];
	dst->eccBits = src[112];
	dst->planeAddrBits = src[113];
	dst->multiplaneOpAttr = src[114];
	dst->ezNandSupport = src[115];

	memcpy(dst->res3, &src[116], sizeof(dst->res3));

	/* Electrical parameters block (offsets 128-163) */
	dst->ioPinCapacitance = src[128];
	dst->timingMode = deserialize16(&src[129]);
	dst->progCacheTimingMode = deserialize16(&src[131]);
	dst->tPageProg = deserialize16(&src[133]);
	dst->tBlkErase = deserialize16(&src[135]);
	dst->tPageRead = deserialize16(&src[137]);
	dst->tCCS = deserialize16(&src[139]);
	dst->nvddrTimingMode = src[141];
	dst->nvddr2TimingMode = src[142];
	dst->nvddrFeat = src[143];
	dst->clkInputPinCap = deserialize16(&src[144]);
	dst->ioPinCap = deserialize16(&src[146]);
	dst->inputPinCap = deserialize16(&src[148]);
	dst->inputPinCapMax = src[150];
	dst->drvStrength = src[151];
	dst->tMR = deserialize16(&src[152]);
	dst->tADL = deserialize16(&src[154]);
	dst->tPageReadEZ = deserialize16(&src[156]);
	dst->nvddr23Feat = src[158];
	dst->nvddr23WarmupCyc = src[159];
	dst->nvddr3TimingMode = deserialize16(&src[160]);
	dst->nvddr2TimingMode2 = src[162];
	dst->res4 = src[163];

	/* Vendor block (offsets 164-255) */
	dst->vendorRevision = deserialize16(&src[164]);
	memcpy(dst->vendorSpecific, &src[166], sizeof(dst->vendorSpecific));
	dst->crc = deserialize16(&src[254]);
}


const onfi_timingMode_t *onfi_getTimingModeSDR(unsigned int mode)
{
	if (mode > 5U) {
		return NULL;
	}

	/* clang-format off */
	static const onfi_timingMode_t onfi_timings[] = {
		/* Mode 0 */
		{
			.tADL = 400, .tALH = 20, .tALS = 50, .tAR = 25, .tCEA = 100, .tCEH = 20, .tCH = 20, .tCHZ = 100, .tCLH = 20,
			.tCLR = 20, .tCLS = 50, .tCOH = 0, .tCR = 10, .tCR2 = 100, .tCS = 70, .tCS3 = 100, .tDH = 20, .tDS = 40,
            .tFEAT = 1000, .tIR = 10, .tITC = 1000, .tRC = 100, .tREA = 40, .tREH = 30, .tRHOH = 0, .tRHW = 200,
            .tRHZ = 200, .tRLOH = 0, .tRP = 50, .tRR = 40, .tRST = 5000000, .tRST_EZNAND = 250000000,
            .tRSTLUN_EZNAND = 250000000, .tWB = 200, .tWC = 100, .tWH = 30, .tWHR = 120, .tWP = 50, .tWW = 100
		},
		/* Mode 1 */
		{
			.tADL = 400, .tALH = 10, .tALS = 25, .tAR = 10, .tCEA = 45, .tCEH = 20, .tCH = 10, .tCHZ = 50, .tCLH = 10,
			.tCLR = 10, .tCLS = 25, .tCOH = 15, .tCR = 10, .tCR2 = 100, .tCS = 35, .tCS3 = 100, .tDH = 10, .tDS = 20,
            .tFEAT = 1000, .tIR = 0, .tITC = 1000, .tRC = 50, .tREA = 30, .tREH = 15, .tRHOH = 15, .tRHW = 100,
            .tRHZ = 100, .tRLOH = 0, .tRP = 25, .tRR = 20, .tRST = 500000, .tRST_EZNAND = 250000000,
            .tRSTLUN_EZNAND = 500000, .tWB = 100, .tWC = 45, .tWH = 15, .tWHR = 80, .tWP = 25, .tWW = 100
		},
		/* Mode 2 */
		{
			.tADL = 400, .tALH = 10, .tALS = 15, .tAR = 10, .tCEA = 30, .tCEH = 20, .tCH = 10, .tCHZ = 50, .tCLH = 10,
			.tCLR = 10, .tCLS = 15, .tCOH = 15, .tCR = 10, .tCR2 = 100, .tCS = 25, .tCS3 = 100, .tDH = 5, .tDS = 15,
            .tFEAT = 1000, .tIR = 0, .tITC = 1000, .tRC = 35, .tREA = 25, .tREH = 15, .tRHOH = 15, .tRHW = 100,
            .tRHZ = 100, .tRLOH = 0, .tRP = 17, .tRR = 20, .tRST = 500000, .tRST_EZNAND = 250000000,
            .tRSTLUN_EZNAND = 500000, .tWB = 100, .tWC = 35, .tWH = 15, .tWHR = 80, .tWP = 17, .tWW = 100
		},
		/* Mode 3 */
		{
			.tADL = 400, .tALH = 5, .tALS = 10, .tAR = 10, .tCEA = 25, .tCEH = 20, .tCH = 5, .tCHZ = 50, .tCLH = 5,
			.tCLR = 10, .tCLS = 10, .tCOH = 15, .tCR = 10, .tCR2 = 100, .tCS = 25, .tCS3 = 100, .tDH = 5, .tDS = 10,
            .tFEAT = 1000, .tIR = 0, .tITC = 1000, .tRC = 30, .tREA = 20, .tREH = 10, .tRHOH = 15, .tRHW = 100,
            .tRHZ = 100, .tRLOH = 0, .tRP = 15, .tRR = 20, .tRST = 500000, .tRST_EZNAND = 250000000,
            .tRSTLUN_EZNAND = 500000, .tWB = 100, .tWC = 30, .tWH = 10, .tWHR = 80, .tWP = 15, .tWW = 100
		},
		/* Mode 4 */
		{
			.tADL = 400, .tALH = 5, .tALS = 10, .tAR = 10, .tCEA = 25, .tCEH = 20, .tCH = 5, .tCHZ = 30, .tCLH = 5,
			.tCLR = 10, .tCLS = 10, .tCOH = 15, .tCR = 10, .tCR2 = 100, .tCS = 20, .tCS3 = 100, .tDH = 5, .tDS = 10,
            .tFEAT = 1000, .tIR = 0, .tITC = 1000, .tRC = 25, .tREA = 20, .tREH = 10, .tRHOH = 15, .tRHW = 100,
            .tRHZ = 100, .tRLOH = 5, .tRP = 12, .tRR = 20, .tRST = 500000, .tRST_EZNAND = 250000000,
            .tRSTLUN_EZNAND = 500000, .tWB = 100, .tWC = 25, .tWH = 10, .tWHR = 80, .tWP = 12, .tWW = 100
		},
		/* Mode 5 */
		{
			.tADL = 400, .tALH = 5, .tALS = 10, .tAR = 10, .tCEA = 25, .tCEH = 20, .tCH = 5, .tCHZ = 30, .tCLH = 5,
			.tCLR = 10, .tCLS = 10, .tCOH = 15, .tCR = 10, .tCR2 = 100, .tCS = 15, .tCS3 = 10, .tDH = 5, .tDS = 7,
            .tFEAT = 1000, .tIR = 0, .tITC = 1000, .tRC = 20, .tREA = 16, .tREH = 7, .tRHOH = 15, .tRHW = 100,
            .tRHZ = 100, .tRLOH = 5, .tRP = 10, .tRR = 20, .tRST = 500000, .tRST_EZNAND = 250000000,
            .tRSTLUN_EZNAND = 500000, .tWB = 100, .tWC = 20, .tWH = 7, .tWHR = 80, .tWP = 10, .tWW = 100
		},
	};
	/* clang-format on */

	return &onfi_timings[mode];
}
