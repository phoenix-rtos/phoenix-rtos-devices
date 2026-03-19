/*
 * Phoenix-RTOS
 *
 * imx6ull RTC driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Rafał Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_RTC_
#define _IMX6ULL_RTC_

#define SNVS_BASE 0x20CC000UL

#define RTC_ENABLE  1U
#define RTC_DISABLE 0U

#define RTC_SETSTATE_TIMEOUT 0x1ffffUL
/*
 * The synchronization register can capture the counter value in the
 * middle of the counter update. In this case, it is not guaranteed that all bits are
 * properly sampled by the synchronization register; the value read by the software can
 * be wrong. 2 consecutive register reads are recommended, if their value is close, read
 * value is valid.
 * We set allowed diff between consecutive reads to 5 ms.
 */
#define MAX_READOUT_DIFF (5U * 32UL)
#define RTC_READ_TIMEOUT 10U

#define SNVS_RD_TIME  _IOR('p', 0x09, struct tm) /* Read RTC time */
#define SNVS_SET_TIME _IOW('p', 0x0a, struct tm) /* Set RTC time */

enum {
	SNVS_HP = 0x0,            /* HP Lock Register */
	SNVS_HPCOMR,              /* HP Command Register */
	SNVS_HPCR,                /* HP Control Register */
	SNVS_HPSR = (0x14 / 0x4), /* HP Status Register */

	SNVS_HPRTCMR = (0x24 / 0x4), /* HP Real-Time Counter MSB Register */
	SNVS_HPRTCLR,                /* HP Real-Time Counter LSB Register */
	SNVS_HPTAMR,                 /* HP Time Alarm MSB Register */
	SNVS_HPTALR,                 /* HP Time Alarm LSB Register */

	SNVS_LPLR,                /* LP Lock Register */
	SNVS_LPCR,                /* LP Control Register */
	SNVS_LPSR = (0x4C / 0x4), /* LP Status Register */
	SNVS_LPSRTCMR,            /* LP RTC MSB */
	SNVS_LPSRTCLR,            /* LP RTC LSB */

	SNVS_LPSMCMR = (0x5C / 0x4), /* LP Secure Monotonic Counter MSB Register */
	SNVS_LPSMCLR,                /* LP Secure Monotonic Counter LSB Register */
	SNVS_LPGPR = (0x68 / 0x4),   /* LP General-Purpose Register */

	SNVS_HPVIDR1 = (0xBF8 / 0x4), /* HP Version ID Register 1 */
	SNVS_HPVIDR2                  /* HP Version ID Register 2 */
};

/* SNVS_HPCR bits */
#define SNVS_HPCR_RTCEN 0

/* SNVS_LPCR bits */
#define SNVS_LPCR_EN 0
#define SNVS_LPCR_MC 2


#endif
