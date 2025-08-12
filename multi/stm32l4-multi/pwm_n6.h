/*
 * Phoenix-RTOS
 *
 * STM32N6 Pulse Width Modulation driver
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
#include "stm32l4-multi.h"
#include "stm32n6_regs.h"
#include "stm32n6_base.h"
#include "clockdef_n6.h"

#define PWM_CHN_NUM 4

#define PWM_CCMR_REG(chn_id)      (((chn_id) <= 1) ? (tim_ccmr1) : (tim_ccmr2))
#define PWM_CCMR_OCPE_OFF(chn_id) (3 + 8 * ((chn_id) & 1))
#define PWM_CCMR_CCS_OFF(chn_id)  (8 * ((chn_id) & 1))
#define PWM_CCMR_OCMH_OFF(chn_id) (16 + 8 * ((chn_id) & 1))
#define PWM_CCMR_OCML_OFF(chn_id) (4 + 8 * ((chn_id) & 1))
#define PWM_CCMR_OCFE_OFF(chn_id) (2 + 8 * ((chn_id) & 1))

#define PWM_CCER_CCE_OFF(chn_id)  (4 * (chn_id))
#define PWM_CCER_CCNE_OFF(chn_id) (2 + 4 * (chn_id))
#define PWM_CCER_CCP_OFF(chn_id)  (1 + 4 * (chn_id))
#define PWM_CCER_CCNP_OFF(chn_id) (3 + 4 * (chn_id))

#define PWM_CR2_OIS_OFF(chn_id)  (8 + 2 * (chn_id))
#define PWM_CR2_OISN_OFF(chn_id) (9 + 2 * (chn_id))

#define PWM_CCR_REG(chn_id) (tim_ccr1 + (chn_id))

#define PWM_DIER_CCIE_OFF(chn_id) (1 + (chn_id))

/* Discriminate timer types and features */
#define PWM_TIM_CNT_MODE_SELECT ((1 << pwm_tim1) | (1 << pwm_tim2) | (1 << pwm_tim3) | (1 << pwm_tim4) | (1 << pwm_tim5) | (1 << pwm_tim8))
#define PWM_TIM_BREAK1_MODE     ((1 << pwm_tim1) | (1 << pwm_tim8) | (1 << pwm_tim15) | (1 << pwm_tim16) | (1 << pwm_tim17))
#define PWM_TIM_BREAK2_MODE     ((1 << pwm_tim1) | (1 << pwm_tim8))
#define PWM_TIM_REP_CNT         ((1 << pwm_tim1) | (1 << pwm_tim8) | (1 << pwm_tim15) | (1 << pwm_tim16) | (1 << pwm_tim17))

#define PWM_TIM_BASIC           ((1 << pwm_tim6) | (1 << pwm_tim7) | (1 << pwm_tim18))
#define PWM_TIM_ADVANCED        ((1 << pwm_tim1) | (1 << pwm_tim8))
#define PWM_TIM_GP1             ((1 << pwm_tim2) | (1 << pwm_tim3) | (1 << pwm_tim4) | (1 << pwm_tim5))
#define PWM_TIM_GP2             ((1 << pwm_tim9) | (1 << pwm_tim12) | (1 << pwm_tim15))
#define PWM_TIM_NO_MASTER_SLAVE ((1 << pwm_tim10) | (1 << pwm_tim11) | (1 << pwm_tim13) | (1 << pwm_tim14) | (1 << pwm_tim16) | (1 << pwm_tim17))
#define PWM_TIM_32BIT           ((1 << pwm_tim2) | (1 << pwm_tim3) | (1 << pwm_tim4) | (1 << pwm_tim5))

// #define PWM_TIM_MASTER1_MODE(tim_id) ((1 << (tim_id)) & )

/* Get the base clock frequency for a given timer */
uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer);


/* Configure a TIMx peripheral for PWM generation. User should disable timer first if running. Returns errors */
int pwm_configure(pwm_tim_id_t timer, uint16_t prescaler, uint32_t top);


/* Set compare value for a specific channel on configured timer. Returns errors */
int pwm_set(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare);


/* Returns current duty cycle percentage. Or errors */
int pwm_get(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t *top, uint32_t *compare);


/* Creates PWM bit sequence with given compare values. Configure the timer first. The data buffer must be at least nbits/8 bytes long. Only one active bit sequence per channel. */
int pwm_setBitSequence(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare0, uint32_t compare1, uint32_t nbits, uint8_t *data);


int pwm_init(void);


/* Disable timer, together with all enabled channels. Returns errors */
int pwm_disableTimer(pwm_tim_id_t timer);


/* Disable a specific PWM channel. */
int pwm_disableChannel(pwm_tim_id_t timer, pwm_ch_id_t chn);

#endif /* #idndef PWM_N6_H_ */
