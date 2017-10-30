/**
 * Vybrid RTC driver
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file vybrid_macnet/vybrid_macnet.c
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Horacio Mijail Anton Quiles <horacio.anton@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <lib/if.h>
#include <fs/if.h>
#include <dev/if.h>
#include <proc/if.h>
#include <main/if.h>
#include "hal/arm/MVF50GS10MK50.h"
#include <include/errno.h>
#include <std.h>

static SNVS_Type *SNVS_virt;

#define CPU_XTAL32k_CLK_HZ              32768u

int rtc_gettimeofday(struct timespec *t)
{
	u8 i = 0;
	u64 rtc0 = (((u64)SNVS_virt->HPRTCMR) <<32) | (SNVS_virt->HPRTCLR);
	u64 rtc;
	do{
		rtc = (((u64)SNVS_virt->HPRTCMR) <<32) | (SNVS_virt->HPRTCLR);
		if(rtc == rtc0)
			break;
		else {
			rtc0 = rtc;
			i++;
		}
	} while (i<5);

	t->tv_sec = rtc/CPU_XTAL32k_CLK_HZ;
	t->tv_nsec = (((rtc % CPU_XTAL32k_CLK_HZ) * 1000) / CPU_XTAL32k_CLK_HZ) * 1000000; /* Milisecond precision */

	if(i == 5){
		main_printf(ATTR_ERROR, "RTC unstable\n"); /*RM says 3 should be enough*/
		return -EIO;
	} else
		return EOK;

}

int _rtc_init(void)
{
	s32 result;
	result = vm_iomap(SNVS_BASE, sizeof(SNVS_Type), PGHD_DEV_RW, (void *)&SNVS_virt);
	assert(result == EOK);

	/*LP-RTC sanity check*/
	if(((result = SNVS_virt->LPSR) & (SNVS_LPSR_VTD_MASK 	| 	/* Voltage tampering - covers failed battery?*/
										SNVS_LPSR_CTD_MASK 	| 	/* Clock tampering*/
										SNVS_LPSR_PGD_MASK 	| 	/* Power supply glitch / covers failed battery?*/
										SNVS_LPSR_SRTCR_MASK)) != 0){	/* LP-RTC rolled over*/
		main_printf(ATTR_ERROR, "dev: RTC initial value is unreliable. LPSR = 0x%x\n",result);
	}

	/*copy the RTC counter from the Low-Power section to the High-Power one*/
	SNVS_virt->HPCR |= SNVS_HPCR_HP_TS_MASK;

	/*enable HP-RTC*/
	SNVS_virt->HPCR |= SNVS_HPCR_RTC_EN_MASK;



#if 0
	u32 t, t2;
	result = rtc_gettimeofday(&t);
	assert(result == EOK);
	proc_threadSleep(2000000);
	result = rtc_gettimeofday(&t2);
	main_printf(ATTR_DEBUG, "dt = %d\n", t2-t);
#endif

	return EOK;
}
