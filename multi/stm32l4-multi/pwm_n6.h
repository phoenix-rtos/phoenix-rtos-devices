/*
 * Phoenix-RTOS
 *
 * STM32N6 PWM driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Krzysztof Radzewicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef PWM_N6_H_
#define PWM_N6_H_

#include <stdint.h>
#include <limits.h>
#include "stm32n6_regs.h"
#include "stm32n6_base.h"
#include "clockdef_n6.h"

#define PWM_TIMER_NUM 15
#define PWM_CHN_NUM   4

#define PWM_CCMR_REG(chn_id)      (((chn_id) <= 1) ? (tim_ccmr1) : (tim_ccmr2))
#define PWM_CCMR_OCPE_OFF(chn_id) (3 + 8 * ((chn_id) & 0x1))
#define PWM_CCMR_CCS_OFF(chn_id)  (8 * ((chn_id) & 0x1))
#define PWM_CCMR_OCMH_OFF(chn_id) (16 + 8 * ((chn_id) & 0x1))
#define PWM_CCMR_OCML_OFF(chn_id) (4 + 8 * ((chn_id) & 0x1))
#define PWM_CCMR_OCFE_OFF(chn_id) (2 + 8 * ((chn_id) & 0x1))

#define PWM_CCER_CCE_OFF(chn_id)  (4 * (chn_id))
#define PWM_CCER_CCNE_OFF(chn_id) (2 + 4 * (chn_id))
#define PWM_CCER_CCP_OFF(chn_id)  (1 + 4 * (chn_id))
#define PWM_CCER_CCNP_OFF(chn_id) (3 + 4 * (chn_id))

#define PWM_CR2_OIS_OFF(chn_id)  (8 + 2 * (chn_id))
#define PWM_CR2_OISN_OFF(chn_id) (9 + 2 * (chn_id))

#define PWM_CCR_REG(chn_id) (tim_ccr1 + (chn_id))

#define PWM_DIER_CCIE_OFF(chn_id) (1 + (chn_id))

/* Basic timers TIM6/TIM7/TIM18 not available for PWM */
enum pwm_tim_ids {
	pwm_tim1,
	pwm_tim2,
	pwm_tim3,
	pwm_tim4,
	pwm_tim5,
	pwm_tim8,
	pwm_tim9,
	pwm_tim10,
	pwm_tim11,
	pwm_tim12,
	pwm_tim13,
	pwm_tim14,
	pwm_tim15,
	pwm_tim16,
	pwm_tim17,
};

/* Only primary channels (0-3) are meant to be used directly */
enum pwm_chn_ids {
	pwm_ch1,
	pwm_ch2,
	pwm_ch3,
	pwm_ch4,
	pwm_ch1n,
	pwm_ch2n,
	pwm_ch3n,
	pwm_ch4n,
};

typedef enum pwm_tim_ids pwm_tim_id_t;

typedef enum pwm_chn_ids pwm_ch_id_t;


uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer);


/* Returns errors */
int pwm_configure(pwm_tim_id_t timer, uint32_t prescaler, uint16_t top);


/* Returns errors */
int pwm_set(pwm_tim_id_t timer, pwm_ch_id_t chn, uint16_t compare);


/* Returns current duty cycle percentage. Or errors */
int pwm_get(pwm_tim_id_t timer, pwm_ch_id_t chn);


int pwm_setBitSequence(void);


void pwm_init(void);


int pwm_disableTimer(pwm_tim_id_t timer);


int pwm_disableChannel(pwm_tim_id_t timer, pwm_ch_id_t chn);

#endif /* #idndef PWM_N6_H_ */
