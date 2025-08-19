/*
 * Phoenix-RTOS
 *
 * Temporary STM32N6 PWM driver
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
