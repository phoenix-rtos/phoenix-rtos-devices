/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver
 *
 * ONFI 4.0 SDR timing table
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * %LICENSE%
 */

#include <stdio.h>

#include "nandfctrl2-onfi.h"


const onfi_timingMode_t *onfi_getTimingModeSDR(unsigned int mode)
{
	if (mode > 5u) {
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
