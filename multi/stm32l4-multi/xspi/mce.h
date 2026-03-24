/*
 * Phoenix-RTOS
 *
 * STM32 Memory Cipher Engine (MCE) driver header
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MCE_H_
#define _MCE_H_

#include <stdbool.h>
#include <stdint.h>


typedef enum {
	mce_r1 = 0,
	mce_r2,
	mce_r3,
	mce_r4,
	mce_regcount,
} mce_reg_t;


typedef struct {
	uint32_t start;
	uint32_t end;
	uint16_t granularity;
	bool enabled;
} mce_regionInfo_t;


/* Get status of encryption region
 * Returns:
 * * < 0 on error (incl. device is disabled)
 * * 0 on success
 */
int mce_getRegionSetup(int mceDev, mce_reg_t region, mce_regionInfo_t *setup);


/* Enable or disable selected region.
 * NOTE: this assumes the region was previously set up correctly.
 */
int mce_setRegionEnabled(int mceDev, mce_reg_t region, bool enabled);


#endif /* _MCE_H_ */
