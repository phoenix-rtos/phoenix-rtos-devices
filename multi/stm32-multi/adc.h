/*
 * Phoenix-RTOS
 *
 * STM32L1 ADC driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _ADC_H_
#define _ADC_H_


unsigned short adc_conversion(char channel);


int adc_init(void);


#endif
